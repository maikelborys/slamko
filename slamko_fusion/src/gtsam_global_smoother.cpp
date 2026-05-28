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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/StereoFactor.h>

namespace slamko_fusion {

using gtsam::symbol_shorthand::L;
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
  auto stereo_noise = gtsam::noiseModel::Isotropic::Sigma(3, in.pixel_sigma);
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
  return out;
}

}  // namespace slamko_fusion
