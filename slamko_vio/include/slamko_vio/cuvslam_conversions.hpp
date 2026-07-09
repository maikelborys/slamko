// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// cuVSLAM <-> slamko frame/covariance conversions — pure Eigen, NO cuVSLAM types,
// so they unit-test without the GPU library (the dossier's "test FIRST" trap list,
// PLAN_CUVSLAM_PROVIDER_01 §2.3: an unverified reorder transposes the cross-covariance
// blocks and silently corrupts every edge weight downstream).
//
// Two conventions to bridge:
//  1. FRAME: cuVSLAM world/rig use the OpenCV basis (x-right, y-down, z-forward);
//     slamko/ROS use x-forward, y-left, z-up. The change of basis is the fixed
//     rotation C = ros_from_cv; poses conjugate T_ros = C T_cv C^-1 (both the world
//     and the body frame are re-based by the same C, so the odom stream stays
//     self-consistent — what the relative-edge chain needs).
//  2. COVARIANCE: cuVSLAM emits a row-major 6x6 in (rx, ry, rz, x, y, z) order
//     (rotation-first, cuvslam2.h PoseWithCovariance); slamko ProviderSample and
//     nav_msgs use (x, y, z, rx, ry, rz) (translation-first). Reorder = swap the
//     3x3 blocks; then rotate each block into the ROS basis: B' = C B C^T.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slamko_core/se3.hpp"

namespace slamko::cuvslam_conv {

// Fixed basis change: ROS vector = C * OpenCV vector.
// x_ros = z_cv (forward), y_ros = -x_cv (left), z_ros = -y_cv (up).
inline const Eigen::Matrix3d& rosFromCv() {
  static const Eigen::Matrix3d C = [] {
    Eigen::Matrix3d m;
    m << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    return m;
  }();
  return C;
}

// cuVSLAM world_from_rig (quat xyzw + translation, OpenCV basis on both sides)
// -> slamko SE3 in the ROS basis (conjugation: both frames re-based by C).
inline SE3 poseToRos(const std::array<float, 4>& q_xyzw, const std::array<float, 3>& t) {
  const Eigen::Quaterniond q(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
  const Eigen::Vector3d tr(t[0], t[1], t[2]);
  const Eigen::Matrix3d& C = rosFromCv();
  return SE3(Eigen::Matrix3d(C * q.normalized().toRotationMatrix() * C.transpose()), C * tr);
}

// cuVSLAM row-major 6x6 covariance (rot-first, OpenCV basis, float) ->
// [trans;rot] (nav_msgs order), ROS basis, double.
inline Eigen::Matrix<double, 6, 6> covToRos(const float* cov_rowmajor_36) {
  Eigen::Matrix<double, 6, 6> cv;
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c) cv(r, c) = static_cast<double>(cov_rowmajor_36[r * 6 + c]);
  // Block swap rot-first -> trans-first (P * cv * P^T with the 3+3 permutation).
  Eigen::Matrix<double, 6, 6> sw;
  sw.block<3, 3>(0, 0) = cv.block<3, 3>(3, 3);  // trans-trans
  sw.block<3, 3>(0, 3) = cv.block<3, 3>(3, 0);  // trans-rot
  sw.block<3, 3>(3, 0) = cv.block<3, 3>(0, 3);  // rot-trans
  sw.block<3, 3>(3, 3) = cv.block<3, 3>(0, 0);  // rot-rot
  // Rotate every block into the ROS basis: blockdiag(C, C) * sw * blockdiag(C, C)^T.
  const Eigen::Matrix3d& C = rosFromCv();
  Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
  D.block<3, 3>(0, 0) = C;
  D.block<3, 3>(3, 3) = C;
  return D * sw * D.transpose();
}

}  // namespace slamko::cuvslam_conv
