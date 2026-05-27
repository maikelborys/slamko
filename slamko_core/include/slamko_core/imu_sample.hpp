// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// IMU types crossing the Tier-1 → Tier-2 boundary. A LocalSmoother backend owns
// its own preintegration (GTSAM's CombinedImuFactor needs gtsam's own
// PreintegratedCombinedMeasurements; klt_vo's LocalBA uses its own), so the
// contract passes RAW samples + the noise/gravity params — not a preintegrated
// result. Gravity is locked at init (stereo gives metric scale).

#pragma once

#include <Eigen/Core>

namespace slamko {

struct ImuSample {
  double timestamp = 0.0;
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();  // m/s², body frame
  Eigen::Vector3d gyro  = Eigen::Vector3d::Zero();  // rad/s, body frame
};

struct ImuBias {
  Eigen::Vector3d gyro  = Eigen::Vector3d::Zero();  // b_g
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();  // b_a
};

// Continuous-time noise densities + bias random-walk + the locked gravity vector
// (world frame; |g| fixed to 9.81, direction from the VI init).
struct ImuParams {
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  double accel_noise_density = 2.0e-3;    // m/s²/√Hz
  double gyro_noise_density  = 1.6968e-4; // rad/s/√Hz
  double accel_bias_rw       = 3.0e-3;    // m/s³/√Hz
  double gyro_bias_rw        = 1.9393e-5; // rad/s²/√Hz
};

}  // namespace slamko
