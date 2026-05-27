// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Closed-form triangulation for a rectified stereo pair (header-only).
//
// For a stereo pair with horizontal baseline b, focal length (fx, fy),
// principal point (cx, cy), and matched observation (xL, y) in the left
// camera and (xR, y) in the right camera (note same row by rectification):
//
//   Z = fx * b / (xL - xR)         (only valid when disparity > 0)
//   X = (xL - cx) * Z / fx
//   Y = (y  - cy) * Z / fy
//
// Output is the 3D point in the left-camera optical frame (x-right, y-down,
// z-forward). Returns false if disparity is non-positive or Z is outside
// the valid depth window.

#pragma once

#include <cmath>

#include <Eigen/Core>

#include "slamko_vio/types.hpp"

namespace slamko_vio {

inline bool triangulate_stereo(
    float xL, float xR, float y,
    const StereoIntrinsics& K,
    float min_depth_m, float max_depth_m,
    Eigen::Vector3f& out_point) {
  const float d = xL - xR;
  if (!(d > 1.0e-3f)) return false;
  const float Z = K.fx * K.baseline_m / d;
  if (!(Z > min_depth_m) || !(Z < max_depth_m)) return false;
  out_point.x() = (xL - K.cx) * Z / K.fx;
  out_point.y() = (y  - K.cy) * Z / K.fy;
  out_point.z() = Z;
  return true;
}

}  // namespace slamko_vio
