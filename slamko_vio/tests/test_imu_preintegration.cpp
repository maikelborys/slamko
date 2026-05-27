// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Unit tests for slamko_vio::ImuPreintegration. Hand-crafted ground-truth motions
// confirm:
//   1. Static body (no rotation, no accel besides gravity-free trivial) keeps
//      Δ-state at identity.
//   2. Constant rotation around z reproduces the analytical Exp(ω·t).
//   3. Constant linear accel in body produces ΔV = a·t and ΔP = 0.5·a·t².
//   4. First-order bias correction matches a direct re-integration to 1e-3.

#include "slamko_vio/imu_preintegration.hpp"
#include "slamko_vio/imu_types.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using namespace slamko_vio;

namespace {

ImuNoise quiet_noise() {
  ImuNoise n;
  // Make noise tiny so covariance stays small; tests focus on mean dynamics.
  n.accel_noise_density = 1.0e-6;
  n.gyro_noise_density  = 1.0e-6;
  n.rate_hz             = 200.0;
  return n;
}

}  // namespace

TEST(ImuPreintegration, Static) {
  ImuBias b;
  ImuPreintegration pi(b, quiet_noise());
  const double dt = 1.0 / 200.0;
  for (int k = 0; k < 200; ++k) {
    pi.integrate(dt, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }
  EXPECT_NEAR(pi.dt(), 1.0, 1.0e-9);
  EXPECT_LT((pi.dR() - Eigen::Matrix3d::Identity()).norm(), 1.0e-9);
  EXPECT_LT(pi.dV().norm(), 1.0e-9);
  EXPECT_LT(pi.dP().norm(), 1.0e-9);
}

TEST(ImuPreintegration, ConstantRotation) {
  // Body spins at 1 rad/s around +z for 1 s. ΔR should be Rz(1 rad).
  ImuBias b;
  ImuPreintegration pi(b, quiet_noise());
  const Eigen::Vector3d w(0.0, 0.0, 1.0);
  const Eigen::Vector3d a = Eigen::Vector3d::Zero();
  const double dt = 1.0 / 1000.0;
  for (int k = 0; k < 1000; ++k) pi.integrate(dt, w, a);

  Eigen::Matrix3d Rz_expected;
  Rz_expected << std::cos(1.0), -std::sin(1.0), 0.0,
                 std::sin(1.0),  std::cos(1.0), 0.0,
                 0.0,            0.0,           1.0;
  EXPECT_LT((pi.dR() - Rz_expected).norm(), 1.0e-4);
}

TEST(ImuPreintegration, ConstantAccel) {
  // Body has constant accel = 1 m/s² along +x, no rotation, no gravity (we
  // model preintegrated quantities, not full state — gravity is added in the
  // factor).
  ImuBias b;
  ImuPreintegration pi(b, quiet_noise());
  const Eigen::Vector3d w = Eigen::Vector3d::Zero();
  const Eigen::Vector3d a(1.0, 0.0, 0.0);
  const double dt = 1.0 / 200.0;
  for (int k = 0; k < 200; ++k) pi.integrate(dt, w, a);

  EXPECT_LT((pi.dV() - Eigen::Vector3d(1.0, 0.0, 0.0)).norm(), 1.0e-6);
  EXPECT_LT((pi.dP() - Eigen::Vector3d(0.5, 0.0, 0.0)).norm(), 1.0e-6);
}

TEST(ImuPreintegration, BiasCorrectionFirstOrder) {
  // Integrate twice: once at linearisation bias 0, once at a small bias.
  // The first-order corrected output (from the 0-bias run) should match the
  // direct re-integration to second-order in the bias delta.
  const Eigen::Vector3d dbg(0.001, -0.002, 0.0015);   // 1.5e-3 rad/s scale
  const Eigen::Vector3d dba(0.005,  0.003, -0.002);   // 5e-3 m/s² scale

  ImuBias b_lin;
  ImuBias b_new;
  b_new.bg = dbg;
  b_new.ba = dba;

  // A meaningful motion: spin + accel.
  std::vector<ImuSample> samples;
  const double dt = 1.0 / 200.0;
  const int    N  = 200;
  samples.reserve(N + 1);
  for (int k = 0; k <= N; ++k) {
    ImuSample s;
    s.t = k * dt;
    s.w = Eigen::Vector3d(0.0, 0.3, 0.0);
    s.a = Eigen::Vector3d(0.5, 0.0, 0.0);
    samples.push_back(s);
  }

  ImuPreintegration pi_lin(b_lin, quiet_noise());
  pi_lin.integrate_span(samples.data(), samples.size());

  ImuPreintegration pi_new(b_new, quiet_noise());
  pi_new.integrate_span(samples.data(), samples.size());

  Eigen::Matrix3d dR_corr;
  Eigen::Vector3d dV_corr, dP_corr;
  pi_lin.corrected(b_new, dR_corr, dV_corr, dP_corr);

  // 2nd-order in (db) ≈ |db|² · (norm of bias-Jacobian²). Allow 1e-3.
  const double tol = 1.0e-3;
  EXPECT_LT((dR_corr - pi_new.dR()).norm(), tol);
  EXPECT_LT((dV_corr - pi_new.dV()).norm(), tol);
  EXPECT_LT((dP_corr - pi_new.dP()).norm(), tol);
}

TEST(ImuPreintegration, So3RoundTrip) {
  // Exp followed by Log returns the original vector for non-pathological
  // rotations.
  const Eigen::Vector3d phi(0.1, -0.2, 0.3);
  const Eigen::Matrix3d R = so3::Exp(phi);
  const Eigen::Vector3d phi_back = so3::Log(R);
  EXPECT_LT((phi - phi_back).norm(), 1.0e-9);
}

TEST(ImuPreintegration, JrTimesJrInvIsIdentity) {
  const Eigen::Vector3d phi(0.4, -0.1, 0.2);
  const Eigen::Matrix3d M = so3::J_r(phi) * so3::J_r_inv(phi);
  EXPECT_LT((M - Eigen::Matrix3d::Identity()).norm(), 1.0e-9);
}
