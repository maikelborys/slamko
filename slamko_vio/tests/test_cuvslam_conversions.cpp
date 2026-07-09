// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// The dossier's "unit-test FIRST" traps (PLAN_CUVSLAM_PROVIDER_01 §2.3): a wrong
// covariance reorder transposes the cross-covariance blocks and silently corrupts
// every edge information downstream; a wrong basis conjugation shows up as a
// constant rotation bias the Sim3-aligned ATE would hide.

#include <gtest/gtest.h>

#include "slamko_vio/cuvslam_conversions.hpp"

using slamko::cuvslam_conv::covToRos;
using slamko::cuvslam_conv::poseToRos;
using slamko::cuvslam_conv::rosFromCv;

TEST(CuvslamConversions, BasisIsProperRotation) {
  const Eigen::Matrix3d& C = rosFromCv();
  EXPECT_NEAR((C * C.transpose() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(C.determinant(), 1.0, 1e-12);
  // cv forward (z) -> ros forward (x); cv down (y) -> ros -z (up is -down)
  EXPECT_NEAR((C * Eigen::Vector3d(0, 0, 1) - Eigen::Vector3d(1, 0, 0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR((C * Eigen::Vector3d(0, 1, 0) - Eigen::Vector3d(0, 0, -1)).norm(), 0.0, 1e-12);
}

TEST(CuvslamConversions, PoseConjugationPreservesRelativeMotion) {
  // Two cv-frame poses; the ros-frame relative motion must equal the conjugated
  // cv relative motion (the invariant the ProviderChain relies on).
  const Eigen::Quaternionf qa(Eigen::AngleAxisf(0.3f, Eigen::Vector3f(0.2f, 0.5f, 0.8f).normalized()));
  const Eigen::Quaternionf qb(Eigen::AngleAxisf(-0.7f, Eigen::Vector3f(0.9f, -0.1f, 0.4f).normalized()));
  const auto A = poseToRos({qa.x(), qa.y(), qa.z(), qa.w()}, {0.1f, -0.2f, 1.5f});
  const auto B = poseToRos({qb.x(), qb.y(), qb.z(), qb.w()}, {-0.4f, 0.9f, 0.3f});

  // Build the same relative in cv space, then conjugate once.
  const Eigen::Matrix3d C = rosFromCv();
  Eigen::Isometry3d Ta = Eigen::Isometry3d::Identity(), Tb = Eigen::Isometry3d::Identity();
  Ta.linear() = qa.cast<double>().toRotationMatrix();
  Ta.translation() = Eigen::Vector3d(0.1, -0.2, 1.5);
  Tb.linear() = qb.cast<double>().toRotationMatrix();
  Tb.translation() = Eigen::Vector3d(-0.4, 0.9, 0.3);
  const Eigen::Isometry3d rel_cv = Ta.inverse() * Tb;
  Eigen::Matrix3d Rr = C * rel_cv.linear() * C.transpose();
  Eigen::Vector3d tr = C * rel_cv.translation();

  const slamko::SE3 rel_ros = A.inverse() * B;
  EXPECT_NEAR((rel_ros.so3().unit_quaternion().toRotationMatrix() - Rr).norm(), 0.0, 1e-5);
  EXPECT_NEAR((rel_ros.translation() - tr).norm(), 0.0, 1e-5);
}

TEST(CuvslamConversions, CovarianceReorderAndBasis) {
  // Distinct labeled blocks: rot-rot=1*, rot-trans=2*, trans-rot=2^T, trans-trans=3*.
  Eigen::Matrix<float, 6, 6> cv = Eigen::Matrix<float, 6, 6>::Zero();
  cv.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 1.f;   // rot first in cuVSLAM
  cv.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 3.f;   // trans
  Eigen::Matrix3f X;
  X << 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f;
  cv.block<3, 3>(0, 3) = X;              // rot-trans
  cv.block<3, 3>(3, 0) = X.transpose();  // symmetric
  // row-major buffer
  float buf[36];
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c) buf[r * 6 + c] = cv(r, c);

  const Eigen::Matrix<double, 6, 6> ros = covToRos(buf);
  const Eigen::Matrix3d C = rosFromCv();

  // trans block (scaled identity) is basis-invariant -> lands at top-left as 3*I.
  EXPECT_NEAR((ros.block<3, 3>(0, 0) - 3.0 * Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-6);
  EXPECT_NEAR((ros.block<3, 3>(3, 3) - 1.0 * Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-6);
  // cross block: trans-rot in ros = C * (trans-rot in cv) * C^T = C X^T C^T.
  const Eigen::Matrix3d expect_cross = C * X.transpose().cast<double>() * C.transpose();
  EXPECT_NEAR((ros.block<3, 3>(0, 3) - expect_cross).norm(), 0.0, 1e-6);
  // symmetry preserved
  EXPECT_NEAR((ros - ros.transpose()).norm(), 0.0, 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
