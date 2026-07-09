// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// CuvslamProvider — thin provider adapter wrapping cuVSLAM v16+ (open-source,
// built from the slamko/trusted-health fork at ~/coding/cuVSLAM_src) behind the
// slamko_core ProviderSample contract. ODOMETRY-ONLY by construction: only the
// cuvslam::Odometry class is instantiated, never cuvslam::Slam — so the tail
// re-anchor jump machinery (tail.cpp) does not exist in this process and the
// double-loop-closure rule holds (slamko_loop is the only source of global
// constraints). Design + measurements: docs/PLAN_CUVSLAM_PROVIDER_01.md §6 and
// docs/RESEARCH_CUVSLAM_OPENSOURCE_01.md §5.
//
// Quality model (Hard Rule #3 — degradation is covariance, never an if-branch):
//  - healthy: cuVSLAM 6x6 covariance, reordered+re-based (cuvslam_conversions.hpp),
//    scaled by cov_scale (NEES-measured ~80x overconfident on EuRoC, 2026-07-09).
//  - SUSPECT (fork pnp_health: inliers < min_inliers OR info_condition > max_cond —
//    validated on the wall bag: teleports have inliers=1 / cond~1e12 vs clean-data
//    floor inliers p5=14 / cond max 4.7e4): covariance multiplied by
//    suspect_cov_mult (bounded — a weak edge, never a free hinge).
//  - hard invalid (world_from_rig nullopt): track() returns nullopt. The caller's
//    stale-gap machinery (HOLD / seal-on-doubt) owns the response.
//
// PIMPL: this header has no cuVSLAM types, so slamko_ros/slamko_vio consumers stay
// CUDA/cuVSLAM-free unless SLAMKO_WITH_CUVSLAM is ON (the nvblox pattern).

#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "slamko_core/odometry_provider.hpp"

namespace slamko {

struct CuvslamProviderConfig {
  // Rectified pinhole stereo (D455 IR / EuRoC post-rectification / Gazebo).
  int width = 0, height = 0;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  double baseline_m = 0;  // right camera at (+baseline, 0, 0) in the left/rig frame

  bool async_sba = true;   // production default; sync for deterministic A/Bs
  bool health = true;      // pnp_health gates (needs the slamko/trusted-health fork)

  // Gates measured 2026-07-09 (RESEARCH_CUVSLAM_OPENSOURCE_01 §5): clean-data floor
  // inliers p5=14 / cond max 4.7e4; teleport frames inliers=1 / cond 1e12.
  int min_inliers = 10;
  double max_info_condition = 1e6;

  double cov_scale = 80.0;        // NEES-measured overconfidence correction
  double suspect_cov_mult = 25.0; // bounded down-weight (ProviderChain quality_mult_max)
};

struct CuvslamHealth {
  int observations = 0, inliers = 0;
  float mean_residual = 0.f, final_cost = -1.f, info_condition = 0.f;
  bool suspect = false;  // gate verdict for the LAST tracked frame
  bool valid = false;    // pose returned at all
};

class CuvslamProvider {
 public:
  CuvslamProvider();
  ~CuvslamProvider();
  CuvslamProvider(const CuvslamProvider&) = delete;
  CuvslamProvider& operator=(const CuvslamProvider&) = delete;

  // Builds the rig + tracker and warms up the GPU. Returns false on failure.
  bool init(const CuvslamProviderConfig& cfg);

  // One synchronized mono8 stereo pair (row-major, pitch = bytes per row).
  // Returns the pose sample in the ROS basis with gated covariance, or nullopt on
  // hard tracking loss (caller's stale-gap / seal-on-doubt machinery reacts).
  std::optional<ProviderSample> track(double t_s, const std::uint8_t* left,
                                      const std::uint8_t* right, int pitch);

  const CuvslamHealth& lastHealth() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace slamko
