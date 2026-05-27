// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// EstimationFrame — one of the two types crossing the Tier-2 → Tier-3 boundary
// (the other is SubMap). The fast local fusion emits these; the global layer
// consumes them. Carries the body nav state at a keyframe time plus the
// custom_data escape hatch, so the pipeline extends without changing the type.

#pragma once

#include <cstdint>

#include <Eigen/Core>

#include "slamko_core/custom_data.hpp"
#include "slamko_core/se3.hpp"

namespace slamko {

struct EstimationFrame {
  std::uint64_t id = 0;
  double timestamp = 0.0;

  SE3 T_WB;                                            // body-in-world (locked convention)
  Eigen::Vector3d velocity_W = Eigen::Vector3d::Zero();  // v in world, m/s
  Eigen::Matrix<double, 6, 1> bias =
      Eigen::Matrix<double, 6, 1>::Zero();             // (b_g[3], b_a[3])

  CustomData custom_data;  // descriptor index, tracks, sensor blobs, ...
};

}  // namespace slamko
