// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// GtsamGlobalSmoother — see header. Builds a NonlinearFactorGraph over KF body poses
// (T_WB) + landmark points from the input's observations, gauges with a tight prior
// on `anchor_kf`, adds an optional BetweenFactor for the loop closure measurement,
// LM-optimizes, returns refined values. Stereo only in v1; mono rows (NaN-x on
// uv_right) are skipped.

#include "slamko_fusion/gtsam_global_smoother.hpp"

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

namespace slamko_fusion {

using gtsam::symbol_shorthand::B;  // IMU bias (ConstantBias)
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::V;  // Velocity3 (world frame)
using gtsam::symbol_shorthand::X;

namespace {
gtsam::Pose3 to_gtsam(const slamko::SE3& T) {
  return gtsam::Pose3(gtsam::Rot3(T.so3().unit_quaternion()),
                      gtsam::Point3(T.translation()));
}
slamko::SE3 from_gtsam(const gtsam::Pose3& p) {
  return slamko::SE3(slamko::SO3(p.rotation().toQuaternion()), p.translation());
}
}  // namespace

slamko::GlobalBAOutput GtsamGlobalSmoother::optimize(const slamko::GlobalBAInput& in) {
  slamko::GlobalBAOutput out;

  // Index initial values by id; observations + the loop refer to these.
  std::unordered_map<std::uint64_t, gtsam::Pose3> kf_initial;
  for (const auto& kv : in.keyframes) kf_initial[kv.first] = to_gtsam(kv.second);
  std::unordered_map<std::uint64_t, gtsam::Point3> lm_initial;
  for (const auto& kv : in.landmarks) lm_initial[kv.first] = gtsam::Point3(kv.second);

  if (!kf_initial.count(in.anchor_kf)) return out;  // bad anchor → unchanged (converged=false)

  auto stereo_cal = std::make_shared<gtsam::Cal3_S2Stereo>(
      in.calib.fx, in.calib.fy, 0.0, in.calib.cx, in.calib.cy, in.calib.baseline);
  const gtsam::Pose3 body_T_cam = to_gtsam(in.T_BS);
  // Robust m-estimator on the reprojection residual: Huber with k=1.5σ. Real Atlas
  // data has outlier observations (bootstrap KFs whose stored pose lagged behind the
  // fixed-lag smoother's later refinement → stale, wildly-mis-reprojecting uv) that
  // would otherwise dominate the cost and pull the LM solver off into a bad basin.
  // Huber downweights residuals beyond 1.5σ linearly instead of quadratically, which
  // keeps the well-conditioned majority in control. Validated on V1_03 (per-submap BA
  // initial cost ~10⁵-10⁹ → final 10⁵-10⁶ for healthy submaps; outlier submaps don't
  // wreck the rest of the trajectory).
  auto base_noise   = gtsam::noiseModel::Isotropic::Sigma(3, in.pixel_sigma);
  auto stereo_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.5), base_noise);
  // Tight prior on the anchor KF — removes the global gauge freedom without making
  // the system rank-deficient via Fix(). 1e-6 sigma is effectively a hard pin while
  // still being a normal factor (LM can handle it).
  auto anchor_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  // Track variables actually constrained by at least one factor — we only insert
  // those into Values so the system stays well-conditioned (free unobserved vars
  // would make LM indeterminate).
  std::unordered_set<std::uint64_t> used_kfs, used_lms;

  // Stereo reprojection factors. Variable = T_WB (body in world); GenericStereoFactor
  // uses body_T_cam internally to compose the camera pose for projection.
  for (const auto& o : in.observations) {
    if (!o.hasRight()) continue;  // v1 = stereo only
    if (!kf_initial.count(o.kf_id) || !lm_initial.count(o.landmark_id)) continue;
    graph.add(gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(
        gtsam::StereoPoint2(o.uv_left.x(), o.uv_right.x(), o.uv_left.y()),
        stereo_noise, X(o.kf_id), L(o.landmark_id), stereo_cal, body_T_cam));
    used_kfs.insert(o.kf_id);
    used_lms.insert(o.landmark_id);
  }

  // IMU factors (Phase B.2): when `imu_windows` is non-empty, add a CombinedImuFactor
  // between each (kf_from, kf_to) pair, plus per-KF Velocity3 + ConstantBias
  // variables and a tight prior on the anchor's velocity + bias to close the
  // 15-DOF VI gauge. Visual-only BA on its own degrades ATE on real data because
  // visual factors leave scale + gravity-aligned rotation + bias drift poorly
  // constrained (see PLAN_BA_GLOBAL.md D.1). IMU factors lock those down.
  std::unordered_map<std::uint64_t, Eigen::Vector3d> v_initial;
  for (const auto& kv : in.velocities) v_initial[kv.first] = kv.second;
  std::unordered_map<std::uint64_t, slamko::ImuBias>  b_initial;
  for (const auto& kv : in.biases)     b_initial[kv.first] = kv.second;
  std::unordered_set<std::uint64_t> used_vs, used_bs;

  const bool use_imu = !in.imu_windows.empty();
  if (use_imu) {
    // Gravity convention: in.imu_params.gravity is the world-frame gravity vector
    // (typical slamko default (0,0,-9.81)). GTSAM's MakeSharedU wants the gravity
    // magnitude; the "U"p convention puts n_gravity = (0,0,-g), matching slamko.
    const double g_mag = in.imu_params.gravity.norm();
    auto imu_pre = gtsam::PreintegrationCombinedParams::MakeSharedU(g_mag);
    const double accel_var = in.imu_params.accel_noise_density *
                             in.imu_params.accel_noise_density;
    const double gyro_var  = in.imu_params.gyro_noise_density *
                             in.imu_params.gyro_noise_density;
    const double accel_bias_var = in.imu_params.accel_bias_rw *
                                  in.imu_params.accel_bias_rw;
    const double gyro_bias_var  = in.imu_params.gyro_bias_rw *
                                  in.imu_params.gyro_bias_rw;
    imu_pre->accelerometerCovariance = accel_var * Eigen::Matrix3d::Identity();
    imu_pre->gyroscopeCovariance     = gyro_var  * Eigen::Matrix3d::Identity();
    imu_pre->biasAccCovariance       = accel_bias_var * Eigen::Matrix3d::Identity();
    imu_pre->biasOmegaCovariance     = gyro_bias_var  * Eigen::Matrix3d::Identity();
    // Tiny integration error term — preintegration cov accumulator gets the dt^2
    // scaling for free; this is the per-step rounding floor.
    imu_pre->integrationCovariance = 1e-8 * Eigen::Matrix3d::Identity();
    imu_pre->biasAccOmegaInt       = 1e-5 * Eigen::Matrix<double, 6, 6>::Identity();

    for (const auto& w : in.imu_windows) {
      if (w.samples.size() < 2) continue;
      if (!kf_initial.count(w.kf_from) || !kf_initial.count(w.kf_to)) continue;
      if (!v_initial.count(w.kf_from)  || !v_initial.count(w.kf_to))  continue;
      if (!b_initial.count(w.kf_from)  || !b_initial.count(w.kf_to))  continue;

      const auto& b0 = b_initial.at(w.kf_from);
      gtsam::imuBias::ConstantBias gtsam_b0(b0.accel, b0.gyro);
      gtsam::PreintegratedCombinedMeasurements pim(imu_pre, gtsam_b0);
      // Forster integration convention: between samples[i-1] and samples[i], use
      // samples[i-1].(accel,gyro) over dt = ts[i]-ts[i-1]. Matches slamko_vio's
      // local smoother — so the local + global BA preintegrate identically.
      for (std::size_t i = 1; i < w.samples.size(); ++i) {
        const double dt = w.samples[i].timestamp - w.samples[i - 1].timestamp;
        if (dt <= 0) continue;
        pim.integrateMeasurement(w.samples[i - 1].accel, w.samples[i - 1].gyro, dt);
      }
      graph.add(gtsam::CombinedImuFactor(
          X(w.kf_from), V(w.kf_from), X(w.kf_to), V(w.kf_to),
          B(w.kf_from), B(w.kf_to), pim));
      used_kfs.insert(w.kf_from);
      used_kfs.insert(w.kf_to);
      used_vs.insert(w.kf_from);
      used_vs.insert(w.kf_to);
      used_bs.insert(w.kf_from);
      used_bs.insert(w.kf_to);
    }

    // Velocity + bias priors on the anchor KF — closes the remaining VI-gauge
    // degrees of freedom that the pose prior leaves open.
    if (v_initial.count(in.anchor_kf) && b_initial.count(in.anchor_kf) &&
        used_vs.count(in.anchor_kf) && used_bs.count(in.anchor_kf)) {
      auto vel_anchor_noise = gtsam::noiseModel::Isotropic::Sigma(
          3, in.anchor_vel_sigma);
      auto bias_anchor_noise = gtsam::noiseModel::Isotropic::Sigma(
          6, in.anchor_bias_sigma);
      graph.add(gtsam::PriorFactor<gtsam::Vector3>(
          V(in.anchor_kf), v_initial.at(in.anchor_kf), vel_anchor_noise));
      const auto& ba = b_initial.at(in.anchor_kf);
      graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
          B(in.anchor_kf), gtsam::imuBias::ConstantBias(ba.accel, ba.gyro),
          bias_anchor_noise));
    }
  }

  // Loop-closure relative-pose factor (bridges submaps when they share no landmarks
  // — slamko's epoch-disjoint default).
  if (in.has_loop &&
      kf_initial.count(in.loop_kf_from) && kf_initial.count(in.loop_kf_to)) {
    auto loop_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << in.loop_sigma_r, in.loop_sigma_r, in.loop_sigma_r,
                             in.loop_sigma_t, in.loop_sigma_t, in.loop_sigma_t).finished());
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        X(in.loop_kf_from), X(in.loop_kf_to), to_gtsam(in.T_from_to), loop_noise));
    used_kfs.insert(in.loop_kf_from);
    used_kfs.insert(in.loop_kf_to);
  }

  // Gauge anchor: tight prior on the chosen KF (pulls it into used_kfs too).
  used_kfs.insert(in.anchor_kf);
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(in.anchor_kf), kf_initial.at(in.anchor_kf), anchor_noise));

  // Seed Values with the used variables only.
  for (const auto id : used_kfs) values.insert(X(id), kf_initial.at(id));
  for (const auto id : used_lms) values.insert(L(id), lm_initial.at(id));
  for (const auto id : used_vs)
    values.insert(V(id), gtsam::Vector3(v_initial.at(id)));
  for (const auto id : used_bs) {
    const auto& b = b_initial.at(id);
    values.insert(B(id), gtsam::imuBias::ConstantBias(b.accel, b.gyro));
  }

  out.initial_cost = graph.error(values);

  gtsam::LevenbergMarquardtParams lm;
  lm.maxIterations    = in.max_iters;
  lm.relativeErrorTol = in.rel_tol;
  lm.verbosity        = gtsam::NonlinearOptimizerParams::SILENT;
  gtsam::LevenbergMarquardtOptimizer opt(graph, values, lm);
  const gtsam::Values refined = opt.optimize();

  out.final_cost = graph.error(refined);
  out.iterations = static_cast<int>(opt.iterations());
  out.converged  = (out.final_cost < out.initial_cost);

  // Extract refined values in the same order as the input (caller resolves by id).
  out.keyframes.reserve(in.keyframes.size());
  for (const auto& kv : in.keyframes) {
    if (refined.exists(X(kv.first)))
      out.keyframes.emplace_back(kv.first,
                                 from_gtsam(refined.at<gtsam::Pose3>(X(kv.first))));
    else
      out.keyframes.emplace_back(kv.first, kv.second);  // unconstrained → unchanged
  }
  out.landmarks.reserve(in.landmarks.size());
  for (const auto& kv : in.landmarks) {
    if (refined.exists(L(kv.first)))
      out.landmarks.emplace_back(
          kv.first, Eigen::Vector3d(refined.at<gtsam::Point3>(L(kv.first))));
    else
      out.landmarks.emplace_back(kv.first, kv.second);
  }
  // IMU outputs: refined velocities + biases, same id ordering as input. Empty when
  // visual-only (use_imu=false) so the caller can detect "IMU did not run" by
  // out.velocities.empty().
  out.velocities.reserve(in.velocities.size());
  for (const auto& kv : in.velocities) {
    if (refined.exists(V(kv.first)))
      out.velocities.emplace_back(
          kv.first, Eigen::Vector3d(refined.at<gtsam::Vector3>(V(kv.first))));
    else
      out.velocities.emplace_back(kv.first, kv.second);  // unconstrained → unchanged
  }
  out.biases.reserve(in.biases.size());
  for (const auto& kv : in.biases) {
    if (refined.exists(B(kv.first))) {
      const auto b = refined.at<gtsam::imuBias::ConstantBias>(B(kv.first));
      slamko::ImuBias out_b;
      out_b.accel = b.accelerometer();
      out_b.gyro  = b.gyroscope();
      out.biases.emplace_back(kv.first, out_b);
    } else {
      out.biases.emplace_back(kv.first, kv.second);
    }
  }
  return out;
}

}  // namespace slamko_fusion
