// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Stereo observation + rectified-stereo calibration crossing into the Tier-2
// LocalSmoother. One observation = a landmark seen in the current keyframe's
// left (and optionally right) image. uv_right.x() = NaN means "no stereo match
// this frame" (left-only residual). world_init seeds a freshly-created landmark.

#pragma once

#include <cstdint>
#include <limits>

#include <Eigen/Core>

namespace slamko {

struct StereoCalib {
  double fx = 0, fy = 0, cx = 0, cy = 0;
  double baseline = 0;  // metres (positive); right cam at +baseline along x
};

struct StereoObservation {
  std::uint64_t landmark_id = 0;
  Eigen::Vector2d uv_left  = Eigen::Vector2d::Zero();
  Eigen::Vector2d uv_right = Eigen::Vector2d(
      std::numeric_limits<double>::quiet_NaN(), 0.0);   // NaN-x = no stereo match
  Eigen::Vector3d world_init = Eigen::Vector3d::Zero(); // seed for a new landmark

  bool hasRight() const { return std::isfinite(uv_right.x()); }
};

}  // namespace slamko
