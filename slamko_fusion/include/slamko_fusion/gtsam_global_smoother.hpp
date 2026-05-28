// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// GtsamGlobalSmoother — concrete slamko::GlobalSmoother that hosts the loop-closure
// BUNDLE ADJUSTMENT as a GTSAM NonlinearFactorGraph. Builds a graph over keyframe
// body poses (T_WB) and landmark points (3D world) from the input's observations,
// adds a PriorFactor to anchor the gauge, optionally a BetweenFactor for the loop
// closure measurement, runs Levenberg-Marquardt, returns the refined values.
//
// Stereo observations only in v1 (the slamko tracker emits stereo by default; mono
// rows in the input — NaN-x — are skipped, a clean degradation rather than an error).
// IMU factors are NOT included here — visual BA already closes the dominant geometry
// gap; IMU comes in B.2 once per-KF IMU samples are stored in the SubMap.
//
// GTSAM is fully hidden inside the .cpp (Hard Rule #2): this header only refers to
// slamko_core types so slamko_loop / slamko_vio can hold a pointer to this class
// without inheriting any backend dep. See docs/PLAN_BA_GLOBAL.md.

#pragma once

#include "slamko_core/global_smoother.hpp"

namespace slamko_fusion {

class GtsamGlobalSmoother : public slamko::GlobalSmoother {
 public:
  GtsamGlobalSmoother() = default;
  ~GtsamGlobalSmoother() override = default;

  slamko::GlobalBAOutput optimize(const slamko::GlobalBAInput& in) override;
};

}  // namespace slamko_fusion
