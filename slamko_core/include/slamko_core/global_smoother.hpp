// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// GlobalSmoother — the slamko contract for global BUNDLE ADJUSTMENT on loop closure
// (docs/PLAN_BA_GLOBAL.md). slamko's loop closure is anchors-only pose-graph (works,
// reproducible, ~33% ATE improvement); OKVIS-class cm alignment comes from
// un-marginalizing the loop keyframes back into REPROJECTION ERRORS and reoptimizing
// landmarks + poses + IMU together. That joint optimization lives behind THIS
// contract; the concrete `GtsamGlobalSmoother` (slamko_fusion) hosts the GTSAM graph.
//
// Inputs/outputs are framed in a single GLOBAL frame (the caller transforms per-
// submap data using anchors before feeding it in, and writes back after). This keeps
// the smoother focused on the BA math and lets the supervisor (slamko_loop) own the
// per-submap anchor bookkeeping.
//
// Stereo and monocular observations coexist (per-row NaN-x on uv_right marks "mono"
// — the StereoObservation convention). v1: stereo factors only; mono factors are
// added when needed. Hard Rule #2: this header pulls only Eigen + core types — no
// GTSAM, no ROS — so slamko_loop / slamko_vio can hold a `GlobalSmoother*` without
// inheriting any backend dep.

#pragma once

#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "slamko_core/se3.hpp"
#include "slamko_core/stereo_observation.hpp"

namespace slamko {

// One 2D observation of a landmark from a keyframe (left image always, right
// optional). NaN-x on uv_right marks the row as mono-only (StereoObservation's
// convention). Tagged by ids so the smoother resolves the variables irrespective
// of input order.
struct GlobalBAObservation {
  std::uint64_t kf_id = 0;
  std::uint64_t landmark_id = 0;
  Eigen::Vector2f uv_left  = Eigen::Vector2f::Zero();
  Eigen::Vector2f uv_right = Eigen::Vector2f(
      std::numeric_limits<float>::quiet_NaN(), 0.f);

  bool hasRight() const { return std::isfinite(uv_right.x()); }
};

struct GlobalBAInput {
  // Initial estimates in the GLOBAL frame. Maps from id → value so the smoother
  // can look variables up by the observation tags without assuming any ordering.
  std::vector<std::pair<std::uint64_t, SE3>>           keyframes;   // (kf_id, T_W_body)
  std::vector<std::pair<std::uint64_t, Eigen::Vector3d>> landmarks; // (lm_id, p_world)
  std::vector<GlobalBAObservation> observations;

  // Optional loop-closure relative-pose constraint (BetweenFactor): T_from→to body.
  // Used when the two submaps share NO landmarks (slamko's epoch-disjoint default —
  // visual factors alone wouldn't link the two halves). Skip by leaving has_loop=false.
  std::uint64_t loop_kf_from = 0;
  std::uint64_t loop_kf_to   = 0;
  SE3           T_from_to;          // relative body pose (loop measurement)
  double        loop_sigma_t = 0.05;  // m       — high-confidence weld defaults
  double        loop_sigma_r = 0.02;  // rad
  bool          has_loop = false;

  // Calibration + cam→body extrinsic (constant across the input — slamko is
  // single-rig). The GTSAM stereo factor uses body_T_cam internally, so the
  // pose variable stays T_WB (body in world).
  StereoCalib   calib;
  SE3           T_BS;               // cam→body

  // Gauge anchor: this keyframe's pose is fixed (PriorFactor with tight sigmas)
  // to remove the global-frame freedom. Must be one of the `keyframes` ids.
  std::uint64_t anchor_kf = 0;

  // Noise model parameters.
  double pixel_sigma   = 1.0;        // stereo reprojection sigma (px)
  // LM stopping criteria + cap. BA on a closure typically converges in 10-30 iters.
  int    max_iters     = 30;
  double rel_tol       = 1e-5;
};

struct GlobalBAOutput {
  bool   converged = false;
  double initial_cost = 0.0;
  double final_cost   = 0.0;
  int    iterations   = 0;

  // Refined values in the GLOBAL frame, same ids/order as the input.
  std::vector<std::pair<std::uint64_t, SE3>>           keyframes;
  std::vector<std::pair<std::uint64_t, Eigen::Vector3d>> landmarks;
};

class GlobalSmoother {
 public:
  virtual ~GlobalSmoother() = default;
  virtual GlobalBAOutput optimize(const GlobalBAInput& in) = 0;
};

}  // namespace slamko
