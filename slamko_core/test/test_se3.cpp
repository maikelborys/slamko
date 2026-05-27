// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Manifold correctness: the SE(3)/SO(3) math is the only algorithm core ships,
// so it must be exact. Round-trips, group axioms, the adjoint identity, and the
// near-zero Taylor branch.

#include <gtest/gtest.h>

#include "slamko_core/se3.hpp"

using slamko::SE3;
using slamko::SO3;
using slamko::Vector6d;

namespace {
constexpr double kTol = 1e-10;
}

TEST(SO3, HatVeeRoundTrip) {
  const Eigen::Vector3d w(0.3, -1.2, 0.7);
  EXPECT_LT((SO3::vee(SO3::hat(w)) - w).norm(), kTol);
}

TEST(SO3, ExpLogRoundTrip) {
  const Eigen::Vector3d w(0.4, -0.9, 1.3);
  const Eigen::Vector3d back = SO3::exp(w).log();
  EXPECT_LT((back - w).norm(), kTol);
}

TEST(SO3, ExpIsOrthonormal) {
  const SO3 R = SO3::exp(Eigen::Vector3d(0.2, 0.5, -0.8));
  const Eigen::Matrix3d M = R.matrix();
  EXPECT_LT((M * M.transpose() - Eigen::Matrix3d::Identity()).norm(), kTol);
  EXPECT_NEAR(M.determinant(), 1.0, kTol);
}

TEST(SO3, InverseComposesToIdentity) {
  const SO3 R = SO3::exp(Eigen::Vector3d(1.1, -0.3, 0.6));
  const SO3 I = R * R.inverse();
  EXPECT_LT(I.log().norm(), kTol);
}

TEST(SE3, ExpLogRoundTrip) {
  Vector6d xi;
  xi << 0.5, -1.2, 0.3, 0.4, -0.7, 1.1;  // [rho; omega]
  const Vector6d back = SE3::exp(xi).log();
  EXPECT_LT((back - xi).norm(), kTol);
}

TEST(SE3, InverseTransformsBack) {
  Vector6d xi;
  xi << 1.0, 2.0, -0.5, 0.3, 0.9, -0.2;
  const SE3 T = SE3::exp(xi);
  const Eigen::Vector3d p(0.7, -1.3, 2.2);
  const Eigen::Vector3d roundtrip = T.inverse() * (T * p);
  EXPECT_LT((roundtrip - p).norm(), kTol);
}

TEST(SE3, CompositionMatchesMatrixProduct) {
  const SE3 A = SE3::exp((Vector6d() << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6).finished());
  const SE3 B = SE3::exp((Vector6d() << -0.3, 0.7, 0.1, -0.2, 0.4, -0.5).finished());
  const Eigen::Matrix4d lhs = (A * B).matrix();
  const Eigen::Matrix4d rhs = A.matrix() * B.matrix();
  EXPECT_LT((lhs - rhs).norm(), kTol);
}

TEST(SE3, CompositionAssociative) {
  const SE3 A = SE3::exp((Vector6d() << 0.1, -0.4, 0.2, 0.3, 0.1, -0.6).finished());
  const SE3 B = SE3::exp((Vector6d() << 0.5, 0.2, -0.3, -0.1, 0.7, 0.2).finished());
  const SE3 C = SE3::exp((Vector6d() << -0.2, 0.6, 0.4, 0.5, -0.3, 0.1).finished());
  const Eigen::Matrix4d lhs = ((A * B) * C).matrix();
  const Eigen::Matrix4d rhs = (A * (B * C)).matrix();
  EXPECT_LT((lhs - rhs).norm(), kTol);
}

// Adjoint identity: T * exp(xi) * T^-1 == exp(Adj_T * xi).
TEST(SE3, AdjointIdentity) {
  const SE3 T = SE3::exp((Vector6d() << 0.6, -0.2, 0.9, 0.3, -0.5, 0.4).finished());
  Vector6d xi;
  xi << 0.02, -0.03, 0.01, 0.04, 0.01, -0.02;  // small twist
  const Eigen::Matrix4d lhs = (T * SE3::exp(xi) * T.inverse()).matrix();
  const Eigen::Matrix4d rhs = SE3::exp(T.adjoint() * xi).matrix();
  EXPECT_LT((lhs - rhs).norm(), 1e-8);
}

TEST(SE3, Matrix4dRoundTrip) {
  const SE3 T = SE3::exp((Vector6d() << 0.3, 1.1, -0.7, 0.2, -0.4, 0.8).finished());
  const SE3 back(T.matrix());
  EXPECT_LT((back.matrix() - T.matrix()).norm(), kTol);
}

// Near-zero rotation must use the Taylor branch and stay finite/accurate.
TEST(SE3, TinyTwistTaylorBranch) {
  Vector6d xi;
  xi << 1e-12, -2e-12, 3e-12, 4e-13, -1e-13, 2e-13;
  const SE3 T = SE3::exp(xi);
  EXPECT_TRUE(T.matrix().allFinite());
  const Vector6d back = T.log();
  EXPECT_LT((back - xi).norm(), 1e-15);
}

TEST(SO3, IdentityLogIsZero) {
  EXPECT_LT(SO3().log().norm(), kTol);
}
