// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Common IMU data types: a single sample and an ordered deque of samples.

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include <Eigen/Core>

namespace slamko_vio {

// One IMU sample. Time is seconds (double). Accel in m/s² in the IMU body
// frame; gyro in rad/s in the IMU body frame. EuRoC convention (or whatever
// convention the publisher provides — slamko_vio assumes ROS canonical
// x-forward, y-left, z-up for the IMU body frame).
struct ImuSample {
  double t;            // seconds, same epoch as image stamps
  Eigen::Vector3d a;   // accelerometer reading (specific force), m/s²
  Eigen::Vector3d w;   // gyroscope reading, rad/s
};

using ImuBuffer = std::deque<ImuSample>;

// IMU noise / bias-random-walk model. Defaults are the ADIS16448 numbers
// from EuRoC's imu0/sensor.yaml.
struct ImuNoise {
  double accel_noise_density   = 2.0e-3;   // m/s² / √Hz
  double accel_random_walk     = 3.0e-3;   // m/s³ / √Hz
  double gyro_noise_density    = 1.6968e-4;  // rad/s / √Hz
  double gyro_random_walk      = 1.9393e-5;  // rad/s² / √Hz
  double rate_hz               = 200.0;    // sampling rate, used to convert
                                            // density → per-sample variance
};

struct ImuBias {
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();   // accelerometer bias
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();   // gyroscope bias
};

// Gravity vector (m/s²) in the world frame. ROS-canonical world has +z up,
// so gravity points in -z.
inline Eigen::Vector3d default_gravity() {
  return Eigen::Vector3d(0.0, 0.0, -9.81);
}

}  // namespace slamko_vio
