// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// GtsamLocalSmoother implementation — all GTSAM lives here (PIMPL). Uses a
// BatchFixedLagSmoother: it marginalizes the out-of-window keys via the Schur
// complement (Levenberg-Marquardt over the bounded window), with NO leaf
// requirement — so explicit stereo landmark variables that bridge marginalized
// and surviving poses are handled correctly (iSAM2's marginalizeLeaves cannot).
// Pose nodes are T_WB; GenericStereoFactor carries body_P_sensor (cam-in-body).

#include "slamko_fusion/gtsam_local_smoother.hpp"

#include <cstdio>
#include <unordered_map>
#include <unordered_set>

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/StereoFactor.h>

namespace slamko_fusion {
namespace {

using gtsam::symbol_shorthand::B;  // bias  B(i)
using gtsam::symbol_shorthand::V;  // velocity V(i)
using gtsam::symbol_shorthand::X;  // pose  X(i)

gtsam::Key L(std::uint64_t id) { return gtsam::Symbol('l', id); }

gtsam::Pose3 toPose3(const slamko::SE3& T) {
  return gtsam::Pose3(gtsam::Rot3(T.rotationMatrix()), gtsam::Point3(T.translation()));
}
slamko::SE3 toSE3(const gtsam::Pose3& p) {
  return slamko::SE3(Eigen::Matrix3d(p.rotation().matrix()), Eigen::Vector3d(p.translation()));
}

}  // namespace

struct GtsamLocalSmoother::Impl {
  GtsamSmootherConfig cfg;
  std::unique_ptr<gtsam::BatchFixedLagSmoother> smoother;
  std::shared_ptr<gtsam::PreintegrationCombinedParams> imu_params;
  std::shared_ptr<gtsam::Cal3_S2Stereo> stereo_cal;
  gtsam::Pose3 body_T_cam = gtsam::Pose3::Identity();
  gtsam::SharedNoiseModel stereo_noise;

  std::uint64_t kf = 0;
  bool first = true;
  bool imu_started = false;  // true once the IMU factor chain is anchored
  double latest_t = 0.0;

  // pending batch for the next optimize()
  gtsam::NonlinearFactorGraph pending_graph;
  gtsam::Values pending_values;
  gtsam::FixedLagSmoother::KeyTimestampMap pending_stamps;

  // cached estimate after optimize()
  gtsam::Values estimate;
  gtsam::Pose3 cur_pose;
  gtsam::Vector3 cur_vel = gtsam::Vector3::Zero();
  gtsam::imuBias::ConstantBias cur_bias;

  // landmark bookkeeping: a landmark stays alive while observed (timestamp =
  // latest sighting); the batch smoother Schur-marginalizes it once its newest
  // observation falls out of the lag. A later re-sighting re-inserts its value.
  std::unordered_set<std::uint64_t> lm_active;
  std::unordered_map<std::uint64_t, double> lm_last_seen;
  std::unordered_map<std::uint64_t, int> lm_obs_count;  // sightings while in-window

  // Restart the window: fresh smoother + zeroed counters/estimates, KEEPING the
  // configured calibration (cfg, imu_params, stereo_cal, body_T_cam,
  // stereo_noise). Called on construction and whenever the pipeline reconfigures
  // mid-stream (setExtrinsics on T_BS-resolve, setImuParams on gravity-calib) —
  // matching CeresLocalSmoother's rebuild so the X(i) index stays in step with
  // the pipeline's VI-init restarts (have_last_kf_ = false). Discards the
  // visual-only init KFs, exactly as the ceres baseline does.
  void reset() {
    gtsam::LevenbergMarquardtParams lm;
    // Lag is in KEYFRAMES: keys are stamped with the monotonic KF index (not t),
    // so GTSAM marginalizes a key once (latest_kf_index - its_index) > window_kfs.
    // The +0.5 keeps the marginalization boundary OFF the integer stamps so it
    // never lands exactly on a populated timestamp (which, with integer KF stamps,
    // races our landmark keep-alive bookkeeping → a stereo factor on a just-
    // marginalized landmark → map::at). Our forget loop uses `> window_kfs`, so
    // both agree: age 15 kept, age 16 dropped, the 15.5 boundary hits no stamp.
    smoother =
        std::make_unique<gtsam::BatchFixedLagSmoother>((double)cfg.window_kfs + 0.5, lm);
    kf = 0;
    first = true;
    imu_started = false;
    latest_t = 0.0;
    pending_graph.resize(0);
    pending_values.clear();
    pending_stamps.clear();
    estimate.clear();
    cur_pose = gtsam::Pose3::Identity();
    cur_vel = gtsam::Vector3::Zero();
    cur_bias = gtsam::imuBias::ConstantBias();
    lm_active.clear();
    lm_last_seen.clear();
    lm_obs_count.clear();
  }

  // Disposable-graph recovery (MASTER_PLAN principle #3, GLIM/VILENS): the last
  // solve hit an indeterminate/degenerate system — a tracking-loss frame left a
  // pose rank-deficient — and the BatchFixedLagSmoother's internal Bayes tree is
  // now corrupted (every subsequent calculateEstimate() re-throws on the same
  // stuck variable, so the smoother would be dead for the rest of the run).
  // DISCARD the corrupted graph and re-anchor a FRESH window at the last GOOD
  // estimate (cur_pose/cur_vel/cur_bias); the fast odometry (pipeline PnP) keeps
  // producing poses meanwhile, and the window re-accumulates once tracking
  // resumes. This is the "global graph is disposable; catch-damp-rebuild, never
  // crash" contract the framework is built on — NOT a silent give-up.
  void softRebuild() {
    gtsam::LevenbergMarquardtParams lm;
    smoother =
        std::make_unique<gtsam::BatchFixedLagSmoother>((double)cfg.window_kfs + 0.5, lm);
    pending_graph.resize(0);
    pending_values.clear();
    pending_stamps.clear();
    lm_active.clear();
    lm_last_seen.clear();
    lm_obs_count.clear();
    // Re-anchor the next KF: for IMU mode, first=false + imu_started=false routes
    // it through the re-anchor branch which seeds V/B from the recovered
    // cur_vel/cur_bias (better than the pipeline's dead-reckoned seed). Visual-
    // only mode re-anchors as a clean cold start (first=true). cur_* are KEPT.
    first = !cfg.use_imu;
    imu_started = false;
  }
};

GtsamLocalSmoother::GtsamLocalSmoother(const GtsamSmootherConfig& cfg)
    : impl_(std::make_unique<Impl>()) {
  impl_->cfg = cfg;
  impl_->stereo_noise = gtsam::noiseModel::Isotropic::Sigma(3, cfg.pixel_sigma);
  impl_->reset();
}

GtsamLocalSmoother::~GtsamLocalSmoother() = default;

void GtsamLocalSmoother::setImuParams(const slamko::ImuParams& params) {
  auto P = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
  P->n_gravity = params.gravity;  // locked, possibly tilted (visual-world frame)
  const double an = params.accel_noise_density, gn = params.gyro_noise_density;
  P->setAccelerometerCovariance(gtsam::I_3x3 * (an * an));
  P->setGyroscopeCovariance(gtsam::I_3x3 * (gn * gn));
  P->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);
  P->setBiasAccCovariance(gtsam::I_3x3 * (params.accel_bias_rw * params.accel_bias_rw));
  P->setBiasOmegaCovariance(gtsam::I_3x3 * (params.gyro_bias_rw * params.gyro_bias_rw));
  P->setBiasAccOmegaInit(gtsam::I_6x6 * 1e-5);
  impl_->imu_params = P;
  impl_->reset();  // gravity-calib restart: start the IMU-fused window clean
}

void GtsamLocalSmoother::setStereoCalib(const slamko::StereoCalib& c) {
  impl_->stereo_cal =
      std::make_shared<gtsam::Cal3_S2Stereo>(c.fx, c.fy, 0.0, c.cx, c.cy, c.baseline);
}

void GtsamLocalSmoother::setExtrinsics(const slamko::SE3& body_T_cam) {
  impl_->body_T_cam = toPose3(body_T_cam);
  impl_->reset();  // T_BS-resolve restart (matches CeresLocalSmoother)
}

void GtsamLocalSmoother::insertKeyframe(
    double t, const slamko::SE3& T_WB_init, const Eigen::Vector3d& velocity_init,
    const slamko::ImuBias& bias_init, const std::vector<slamko::ImuSample>& imu,
    const std::vector<slamko::StereoObservation>& obs) {
  auto& I = *impl_;
  const std::uint64_t i = I.kf;
  // Marginalization clock = KF index, NOT wall-clock t (see window_kfs). The IMU
  // preintegration below still uses real dt from imu[k].timestamp — only the
  // fixed-lag horizon is keyframe-based.
  const double stamp = (double)i;
  const gtsam::Pose3 pose0 = toPose3(T_WB_init);
  const gtsam::imuBias::ConstantBias b0(bias_init.accel, bias_init.gyro);  // (acc, gyro)

  // Pose node every KF.
  I.pending_values.insert(X(i), pose0);
  I.pending_stamps[X(i)] = stamp;

  const bool imu_ready = I.cfg.use_imu && I.imu_params && imu.size() >= 2;
  auto insertVB = [&] {
    I.pending_values.insert(V(i), gtsam::Vector3(velocity_init));
    I.pending_values.insert(B(i), b0);
    I.pending_stamps[V(i)] = stamp;
    I.pending_stamps[B(i)] = stamp;
  };

  // Velocity/bias nodes exist ONLY inside the IMU factor chain — NEVER for a
  // visual-only KF. The pipeline streams visual-only KFs through the whole
  // VI-init (before gravity + gyro-bias are ready); inserting unconstrained
  // V/B for those makes the fixed-lag marginalization throw (it tries to
  // marginalize a velocity node with no factor — the "retrieve vN as
  // ConstantBias" failure). So: the first KF (or the first IMU-ready KF after a
  // visual-only run) ANCHORS the chain with full priors; subsequent IMU KFs
  // chain via CombinedImuFactor; visual-only KFs are pose + stereo only.
  // Track whether X(i) ends up connected to SOMETHING (prior, IMU factor, or a
  // stereo factor). A KF inserted during total tracking loss (no stereo) with a
  // broken IMU chain would otherwise be an orphan variable → GTSAM elimination
  // throws and the smoother dies for the rest of the run. See the orphan guard
  // after the observation loop.
  bool pose_anchored = false;
  if (I.first || (imu_ready && !I.imu_started)) {
    I.pending_graph.addPrior(
        X(i), pose0, gtsam::noiseModel::Isotropic::Sigma(6, I.cfg.prior_pose_sigma));
    pose_anchored = true;
    if (I.cfg.use_imu) {
      // BIAS CARRY-FORWARD. A re-anchor (a visual-only KF broke the IMU chain —
      // common on slow stairs with weak tracking) must NOT reset velocity/bias to
      // the pipeline's zero-accel-bias seed: that throws away a converged bias and
      // lets the fresh one absorb gravity-direction error → vertical/yaw creep.
      // Seed the re-anchor from the last optimized estimate (cur_vel/cur_bias),
      // tightly prior'd; only a true cold start (I.first) uses the pipeline seed.
      const bool reanchor = !I.first;  // in this branch, !first ⇒ imu_ready re-anchor
      const gtsam::Vector3 v_seed =
          reanchor ? I.cur_vel : gtsam::Vector3(velocity_init);
      const gtsam::imuBias::ConstantBias b_seed = reanchor ? I.cur_bias : b0;
      I.pending_values.insert(V(i), v_seed);
      I.pending_values.insert(B(i), b_seed);
      I.pending_stamps[V(i)] = stamp;
      I.pending_stamps[B(i)] = stamp;
      I.pending_graph.addPrior(
          V(i), v_seed, gtsam::noiseModel::Isotropic::Sigma(3, I.cfg.prior_vel_sigma));
      I.pending_graph.addPrior(
          B(i), b_seed, gtsam::noiseModel::Isotropic::Sigma(6, I.cfg.prior_bias_sigma));
    }
    if (imu_ready) I.imu_started = true;
  } else if (imu_ready) {
    insertVB();
    gtsam::PreintegratedCombinedMeasurements pim(I.imu_params, b0);
    for (std::size_t k = 1; k < imu.size(); ++k) {
      const double dt = imu[k].timestamp - imu[k - 1].timestamp;
      if (dt > 0.0) pim.integrateMeasurement(imu[k - 1].accel, imu[k - 1].gyro, dt);
    }
    // GTSAM arg order is (pose_i, vel_i, pose_j, vel_j, bias_i, bias_j) — NOT
    // pose,vel,bias grouped per state. Getting it wrong makes GTSAM read V(i) as
    // a bias ("retrieve vN as ConstantBias"); only fires once IMU is exercised.
    I.pending_graph.add(gtsam::CombinedImuFactor(
        X(i - 1), V(i - 1), X(i), V(i), B(i - 1), B(i), pim));
    pose_anchored = true;  // IMU factor connects X(i) to the chain
  } else {
    // Visual-only KF (pre-IMU, or a mid-stream IMU dropout): pose + stereo only.
    // Break the chain so the next IMU-ready KF re-anchors with fresh priors.
    I.imu_started = false;
  }

  int n_stereo = 0;  // stereo factors actually attached to X(i) this insert
  for (const auto& o : obs) {
    if (!o.hasRight() || !I.stereo_cal) continue;  // P1: stereo factors only
    const std::uint64_t id = o.landmark_id;
    // Only admit a landmark on its min_landmark_obs-th sighting — single-view
    // births are dropped (cheap + non-singular); the live set is capped so the
    // per-KF batch solve stays bounded.
    const int seen = ++I.lm_obs_count[id];
    if (!I.lm_active.count(id)) {
      if (seen < I.cfg.min_landmark_obs) continue;
      if (I.cfg.max_landmarks > 0 &&
          (int)I.lm_active.size() >= I.cfg.max_landmarks) continue;
      I.pending_values.insert(L(id), gtsam::Point3(o.world_init));
      // Weak prior keeps a freshly-admitted landmark from making the linear
      // system indeterminant; multi-view stereo dominates it.
      I.pending_graph.addPrior(
          L(id), gtsam::Point3(o.world_init),
          gtsam::noiseModel::Isotropic::Sigma(3, I.cfg.landmark_prior_sigma));
      I.lm_active.insert(id);
    }
    I.pending_stamps[L(id)] = stamp;  // keep alive while observed (KF-index clock)
    I.lm_last_seen[id] = stamp;
    I.pending_graph.add(gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(
        gtsam::StereoPoint2(o.uv_left.x(), o.uv_right.x(), o.uv_left.y()),
        I.stereo_noise, X(i), L(id), I.stereo_cal, I.body_T_cam));
    ++n_stereo;
  }

  // TRACKING-LOSS ROBUSTNESS GUARD. If X(i) got no prior, no IMU factor, and no
  // stereo factor (total tracking loss + broken IMU chain), it is an orphan
  // variable: GTSAM's elimination throws "Leftover keys" / map::at and the
  // smoother never recovers. Anchor it weakly to its dead-reckoned init pose so
  // it stays eliminable. Loose sigma → negligible influence once tracking
  // resumes and real factors reconnect the window.
  if (!pose_anchored && n_stereo == 0) {
    I.pending_graph.addPrior(
        X(i), pose0, gtsam::noiseModel::Isotropic::Sigma(6, I.cfg.orphan_prior_sigma));
  }

  ++I.kf;
  I.first = false;
  I.latest_t = t;
}

bool GtsamLocalSmoother::optimize() {
  auto& I = *impl_;
  try {
    I.smoother->update(I.pending_graph, I.pending_values, I.pending_stamps);
    I.estimate = I.smoother->calculateEstimate();
  } catch (const std::exception& e) {
    std::fprintf(stderr,
                 "[slamko_fusion] smoother update failed (%s) — disposable-graph "
                 "rebuild from last-good nav state\n",
                 e.what());
    I.softRebuild();  // recover, don't die: rebuild the window at the last good estimate
    return false;
  }
  I.pending_graph.resize(0);
  I.pending_values.clear();
  I.pending_stamps.clear();

  const std::uint64_t last = I.kf - 1;
  if (I.estimate.exists(X(last))) I.cur_pose = I.estimate.at<gtsam::Pose3>(X(last));
  if (I.cfg.use_imu) {
    if (I.estimate.exists(V(last))) I.cur_vel = I.estimate.at<gtsam::Vector3>(V(last));
    if (I.estimate.exists(B(last)))
      I.cur_bias = I.estimate.at<gtsam::imuBias::ConstantBias>(B(last));
  }
  // Forget landmarks the smoother has marginalized so a re-sight re-inserts them.
  // Same KF-index clock as the fixed-lag horizon (last = newest KF index).
  for (auto it = I.lm_last_seen.begin(); it != I.lm_last_seen.end();) {
    if ((double)last - it->second > (double)I.cfg.window_kfs) {
      I.lm_active.erase(it->first);
      I.lm_obs_count.erase(it->first);  // a re-sight starts its count fresh
      it = I.lm_last_seen.erase(it);
    } else {
      ++it;
    }
  }
  return true;
}

slamko::SE3 GtsamLocalSmoother::latestPose() const { return toSE3(impl_->cur_pose); }
Eigen::Vector3d GtsamLocalSmoother::latestVelocity() const { return impl_->cur_vel; }
slamko::ImuBias GtsamLocalSmoother::latestBias() const {
  slamko::ImuBias b;
  b.gyro = impl_->cur_bias.gyroscope();
  b.accel = impl_->cur_bias.accelerometer();
  return b;
}

bool GtsamLocalSmoother::landmark(std::uint64_t id, Eigen::Vector3d& out) const {
  if (!impl_->estimate.exists(L(id))) return false;
  out = impl_->estimate.at<gtsam::Point3>(L(id));
  return true;
}

slamko::HealthSignal GtsamLocalSmoother::health() const {
  slamko::HealthSignal h;
  h.odom_stale_gap_s = 0.0;  // loss detection is the vio's stale-gap; richer probe in P1c
  return h;
}

std::size_t GtsamLocalSmoother::numVariables() const { return impl_->estimate.size(); }

}  // namespace slamko_fusion
