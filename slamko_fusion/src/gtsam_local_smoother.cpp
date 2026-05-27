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
};

GtsamLocalSmoother::GtsamLocalSmoother(const GtsamSmootherConfig& cfg)
    : impl_(std::make_unique<Impl>()) {
  impl_->cfg = cfg;
  gtsam::LevenbergMarquardtParams lm;  // defaults are fine for a bounded window
  impl_->smoother = std::make_unique<gtsam::BatchFixedLagSmoother>(cfg.lag, lm);
  impl_->stereo_noise = gtsam::noiseModel::Isotropic::Sigma(3, cfg.pixel_sigma);
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
}

void GtsamLocalSmoother::setStereoCalib(const slamko::StereoCalib& c) {
  impl_->stereo_cal =
      std::make_shared<gtsam::Cal3_S2Stereo>(c.fx, c.fy, 0.0, c.cx, c.cy, c.baseline);
}

void GtsamLocalSmoother::setExtrinsics(const slamko::SE3& body_T_cam) {
  impl_->body_T_cam = toPose3(body_T_cam);
}

void GtsamLocalSmoother::insertKeyframe(
    double t, const slamko::SE3& T_WB_init, const Eigen::Vector3d& velocity_init,
    const slamko::ImuBias& bias_init, const std::vector<slamko::ImuSample>& imu,
    const std::vector<slamko::StereoObservation>& obs) {
  auto& I = *impl_;
  const std::uint64_t i = I.kf;
  const gtsam::Pose3 pose0 = toPose3(T_WB_init);
  const gtsam::imuBias::ConstantBias b0(bias_init.accel, bias_init.gyro);  // (acc, gyro)

  I.pending_values.insert(X(i), pose0);
  I.pending_stamps[X(i)] = t;
  if (I.cfg.use_imu) {
    I.pending_values.insert(V(i), gtsam::Vector3(velocity_init));
    I.pending_values.insert(B(i), b0);
    I.pending_stamps[V(i)] = t;
    I.pending_stamps[B(i)] = t;
  }

  if (I.first) {
    I.pending_graph.addPrior(
        X(i), pose0, gtsam::noiseModel::Isotropic::Sigma(6, I.cfg.prior_pose_sigma));
    if (I.cfg.use_imu) {
      I.pending_graph.addPrior(
          V(i), gtsam::Vector3(velocity_init),
          gtsam::noiseModel::Isotropic::Sigma(3, I.cfg.prior_vel_sigma));
      I.pending_graph.addPrior(
          B(i), b0, gtsam::noiseModel::Isotropic::Sigma(6, I.cfg.prior_bias_sigma));
    }
  } else if (I.cfg.use_imu && I.imu_params && imu.size() >= 2) {
    gtsam::PreintegratedCombinedMeasurements pim(I.imu_params, b0);
    for (std::size_t k = 1; k < imu.size(); ++k) {
      const double dt = imu[k].timestamp - imu[k - 1].timestamp;
      if (dt > 0.0) pim.integrateMeasurement(imu[k - 1].accel, imu[k - 1].gyro, dt);
    }
    I.pending_graph.add(gtsam::CombinedImuFactor(
        X(i - 1), V(i - 1), B(i - 1), X(i), V(i), B(i), pim));
  }

  for (const auto& o : obs) {
    if (!o.hasRight() || !I.stereo_cal) continue;  // P1: stereo factors only
    const std::uint64_t id = o.landmark_id;
    if (!I.lm_active.count(id)) {
      I.pending_values.insert(L(id), gtsam::Point3(o.world_init));
      I.lm_active.insert(id);
    }
    I.pending_stamps[L(id)] = t;  // keep alive while observed
    I.lm_last_seen[id] = t;
    I.pending_graph.add(gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(
        gtsam::StereoPoint2(o.uv_left.x(), o.uv_right.x(), o.uv_left.y()),
        I.stereo_noise, X(i), L(id), I.stereo_cal, I.body_T_cam));
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
    std::fprintf(stderr, "[slamko_fusion] smoother update failed: %s\n", e.what());
    I.pending_graph.resize(0);
    I.pending_values.clear();
    I.pending_stamps.clear();
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
  for (auto it = I.lm_last_seen.begin(); it != I.lm_last_seen.end();) {
    if (I.latest_t - it->second > I.cfg.lag) {
      I.lm_active.erase(it->first);
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
