// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// SessionGraph V0 — see session_graph.hpp for the contract. V0 implements the
// minimal viable backbone: KFs are appended to an internal store, the worker
// thread periodically rebuilds a NonlinearFactorGraph from scratch over the
// recent N KFs + their landmarks + IMU windows, runs LM, and publishes the
// latest correction. iSAM2 / incremental relinearization is the V1 evolution
// — this V0 prioritizes correctness over compute (full-rebuild costs ~10s of
// ms per pass at 50 KFs, async, so it never blocks the live VIO).
//
// All GTSAM lives in this cpp; the header pulls only slamko_core types.

#include "slamko_fusion/session_graph.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/StereoFactor.h>

namespace slamko {

namespace {
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

gtsam::Pose3 toGtsam(const SE3& T) {
  return gtsam::Pose3(gtsam::Rot3(T.so3().unit_quaternion()),
                      gtsam::Point3(T.translation()));
}
SE3 fromGtsam(const gtsam::Pose3& p) {
  return SE3(SO3(p.rotation().toQuaternion()), p.translation());
}
}  // namespace

// --- pimpl ---------------------------------------------------------------

class SessionGraphImpl {
 public:
  explicit SessionGraphImpl(const SessionGraphConfig& cfg,
                            std::atomic<bool>* have_correction_flag,
                            std::atomic<std::uint64_t>* correction_seq)
      : cfg_(cfg), have_correction_flag_(have_correction_flag),
        correction_seq_(correction_seq) {
    stop_ = false;
    worker_ = std::thread([this] { workerLoop(); });
  }

  ~SessionGraphImpl() {
    {
      std::lock_guard<std::mutex> lk(mu_);
      stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

  bool insertKeyframe(SessionKeyframe kf) {
    std::lock_guard<std::mutex> lk(mu_);
    if (kf_by_id_.count(kf.id)) return false;
    const std::uint64_t id = kf.id;
    kf_by_id_.emplace(id, std::move(kf));
    kf_order_.push_back(id);
    ++unprocessed_kfs_;
    if (cfg_.optimize_every_n_kfs > 0 &&
        unprocessed_kfs_ >= cfg_.optimize_every_n_kfs) {
      pending_optimize_ = true;
      cv_.notify_one();
    }
    return true;
  }

  bool insertLoopClosure(SessionLoopClosure lc) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!kf_by_id_.count(lc.kf_from) || !kf_by_id_.count(lc.kf_to)) return false;
    loops_.push_back(lc);
    pending_optimize_ = true;
    cv_.notify_one();
    return true;
  }

  SE3 latestCorrection() const {
    std::lock_guard<std::mutex> lk(corr_mu_);
    return latest_correction_;
  }

  std::size_t keyframeCount() const {
    std::lock_guard<std::mutex> lk(mu_);
    return kf_order_.size();
  }

 private:
  void workerLoop() {
    while (true) {
      std::vector<SessionKeyframe> kfs_snapshot;
      std::vector<SessionLoopClosure> loops_snapshot;
      {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait(lk, [this] { return stop_ || pending_optimize_; });
        if (stop_) return;
        pending_optimize_ = false;
        unprocessed_kfs_ = 0;
        // Snapshot the relinearization window — newest cfg_.relin_window_size KFs.
        const std::size_t n = kf_order_.size();
        const std::size_t start =
            (cfg_.relin_window_size > 0 &&
             n > static_cast<std::size_t>(cfg_.relin_window_size))
                ? n - cfg_.relin_window_size
                : 0;
        kfs_snapshot.reserve(n - start);
        for (std::size_t i = start; i < n; ++i)
          kfs_snapshot.push_back(kf_by_id_.at(kf_order_[i]));
        loops_snapshot = loops_;
      }
      runOptimization(kfs_snapshot, loops_snapshot);
    }
  }

  void runOptimization(const std::vector<SessionKeyframe>& kfs,
                       const std::vector<SessionLoopClosure>& loops) {
    if (kfs.empty()) return;

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    std::unordered_set<std::uint64_t> seen_lms;

    // Stereo calib (shared) + body_T_cam (constant) — reprojection factors.
    auto stereo_cal = std::make_shared<gtsam::Cal3_S2Stereo>(
        cfg_.calib.fx, cfg_.calib.fy, 0.0,
        cfg_.calib.cx, cfg_.calib.cy, cfg_.calib.baseline);
    const gtsam::Pose3 body_T_cam = toGtsam(cfg_.T_BS);
    auto base_noise = gtsam::noiseModel::Isotropic::Sigma(3, cfg_.pixel_sigma);
    auto stereo_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.5), base_noise);

    // Anchor: pin the OLDEST KF in the window (the marginal "prior" stand-in until
    // V1 wires a proper marginal). Tight 1e-6 sigma.
    auto anchor_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    const auto& anchor = kfs.front();
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        X(anchor.id), toGtsam(anchor.T_WB), anchor_noise));

    // Insert pose + velocity + bias variables for every KF in the window.
    for (const auto& kf : kfs) {
      values.insert(X(kf.id), toGtsam(kf.T_WB));
      values.insert(V(kf.id), gtsam::Vector3(kf.velocity_w));
      values.insert(B(kf.id),
                    gtsam::imuBias::ConstantBias(kf.bias.accel, kf.bias.gyro));
    }

    // Stereo reprojection factors over all KF observations. Landmark variables are
    // created lazily on first sight; their initial value is back-projected from
    // the FIRST observation in this window using its KF's pose + stereo baseline.
    for (const auto& kf : kfs) {
      const auto& ko = kf.obs;
      const bool stereo = ko.hasStereo();
      for (int i = 0; i < ko.size(); ++i) {
        if (!stereo || !std::isfinite(ko.uv_right(i, 0))) continue;
        const auto lid = ko.landmark_ids[i];
        const double uL = static_cast<double>(ko.uv(i, 0));
        const double vL = static_cast<double>(ko.uv(i, 1));
        const double uR = static_cast<double>(ko.uv_right(i, 0));
        if (!seen_lms.count(lid)) {
          // Back-project from stereo disparity at this KF (rough; LM will refine).
          const double fx = cfg_.calib.fx;
          const double bL = cfg_.calib.baseline;
          const double disp = std::max(1e-3, uL - uR);
          const double z = fx * bL / disp;
          if (!std::isfinite(z) || z <= 0 || z > 100.0) continue;
          const double x = (uL - cfg_.calib.cx) * z / fx;
          const double y = (vL - cfg_.calib.cy) * z / cfg_.calib.fy;
          const gtsam::Point3 P_cam(x, y, z);
          const gtsam::Pose3 T_WC = toGtsam(kf.T_WB) * body_T_cam;
          const gtsam::Point3 P_world = T_WC.transformFrom(P_cam);
          values.insert(L(lid), P_world);
          seen_lms.insert(lid);
        }
        graph.add(gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(
            gtsam::StereoPoint2(uL, uR, vL),
            stereo_noise, X(kf.id), L(lid), stereo_cal, body_T_cam));
      }
    }

    // CombinedImuFactor between consecutive KFs in the window.
    auto imu_pre = gtsam::PreintegrationCombinedParams::MakeSharedU(
        cfg_.imu_params.gravity.norm());
    imu_pre->accelerometerCovariance =
        (cfg_.imu_params.accel_noise_density * cfg_.imu_params.accel_noise_density) *
        Eigen::Matrix3d::Identity();
    imu_pre->gyroscopeCovariance =
        (cfg_.imu_params.gyro_noise_density * cfg_.imu_params.gyro_noise_density) *
        Eigen::Matrix3d::Identity();
    imu_pre->biasAccCovariance =
        (cfg_.imu_params.accel_bias_rw * cfg_.imu_params.accel_bias_rw) *
        Eigen::Matrix3d::Identity();
    imu_pre->biasOmegaCovariance =
        (cfg_.imu_params.gyro_bias_rw * cfg_.imu_params.gyro_bias_rw) *
        Eigen::Matrix3d::Identity();
    imu_pre->integrationCovariance = 1e-8 * Eigen::Matrix3d::Identity();
    imu_pre->biasAccOmegaInt = 1e-5 * Eigen::Matrix<double, 6, 6>::Identity();

    for (std::size_t i = 1; i < kfs.size(); ++i) {
      const auto& prev = kfs[i - 1];
      const auto& cur  = kfs[i];
      if (cur.imu_since_prev.size() < 2) continue;
      gtsam::imuBias::ConstantBias gtsam_b0(prev.bias.accel, prev.bias.gyro);
      gtsam::PreintegratedCombinedMeasurements pim(imu_pre, gtsam_b0);
      for (std::size_t k = 1; k < cur.imu_since_prev.size(); ++k) {
        const double dt = cur.imu_since_prev[k].timestamp -
                          cur.imu_since_prev[k - 1].timestamp;
        if (dt <= 0) continue;
        pim.integrateMeasurement(cur.imu_since_prev[k - 1].accel,
                                 cur.imu_since_prev[k - 1].gyro, dt);
      }
      graph.add(gtsam::CombinedImuFactor(
          X(prev.id), V(prev.id), X(cur.id), V(cur.id),
          B(prev.id), B(cur.id), pim));
    }

    // Loop-closure BetweenFactors. Only those whose both endpoints are in the
    // current window are added — older loops anchored a refined window already.
    std::unordered_set<std::uint64_t> in_window;
    for (const auto& kf : kfs) in_window.insert(kf.id);
    for (const auto& lc : loops) {
      if (!in_window.count(lc.kf_from) || !in_window.count(lc.kf_to)) continue;
      auto loop_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << lc.sigma_r, lc.sigma_r, lc.sigma_r,
           lc.sigma_t, lc.sigma_t, lc.sigma_t).finished());
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          X(lc.kf_from), X(lc.kf_to), toGtsam(lc.T_from_to), loop_noise));
    }

    gtsam::LevenbergMarquardtParams params;
    params.maxIterations = cfg_.max_iters;
    params.verbosity = gtsam::NonlinearOptimizerParams::SILENT;
    gtsam::LevenbergMarquardtOptimizer opt(graph, values, params);
    gtsam::Values refined;
    try {
      refined = opt.optimize();
    } catch (...) {
      return;  // disposable graph — never crash the live system
    }

    // The correction: latest KF's BA pose vs. its VIO-submitted pose. The VIO
    // composes this with its live worldPose going forward.
    const auto& latest = kfs.back();
    if (!refined.exists(X(latest.id))) return;
    const SE3 ba = fromGtsam(refined.at<gtsam::Pose3>(X(latest.id)));
    // correction = ba * vio_inverse  — composes onto VIO's worldPose to produce
    // the refined pose: T_refined = correction * T_vio.
    SE3 correction(ba.matrix() * latest.T_WB.matrix().inverse());
    {
      std::lock_guard<std::mutex> lk(corr_mu_);
      latest_correction_ = correction;
    }
    have_correction_flag_->store(true);
    correction_seq_->fetch_add(1, std::memory_order_release);
  }

  SessionGraphConfig cfg_;
  std::atomic<bool>* have_correction_flag_;
  std::atomic<std::uint64_t>* correction_seq_;

  // Producer-thread state (locked by mu_).
  mutable std::mutex mu_;
  std::condition_variable cv_;
  bool stop_ = false;
  bool pending_optimize_ = false;
  int  unprocessed_kfs_ = 0;
  std::unordered_map<std::uint64_t, SessionKeyframe> kf_by_id_;
  std::vector<std::uint64_t> kf_order_;
  std::vector<SessionLoopClosure> loops_;

  // Correction state (locked by corr_mu_; separate so VIO never blocks the worker).
  mutable std::mutex corr_mu_;
  SE3 latest_correction_;

  std::thread worker_;
};

// --- public class --------------------------------------------------------

SessionGraph::SessionGraph(const SessionGraphConfig& cfg)
    : impl_(std::make_unique<SessionGraphImpl>(cfg, &have_correction_,
                                                &correction_seq_)) {}

SessionGraph::~SessionGraph() = default;

bool SessionGraph::insertKeyframe(SessionKeyframe kf) {
  return impl_->insertKeyframe(std::move(kf));
}

bool SessionGraph::insertLoopClosure(SessionLoopClosure lc) {
  return impl_->insertLoopClosure(std::move(lc));
}

SE3 SessionGraph::latestCorrection() const { return impl_->latestCorrection(); }

std::size_t SessionGraph::keyframeCount() const { return impl_->keyframeCount(); }

}  // namespace slamko
