// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Unit test for slamko_vio::p3p::solve. Generates a ground-truth (R, t),
// projects 3 random world points through it, and verifies that one of the
// returned solutions recovers (R, t) within tolerance.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <random>

#include "slamko_vio/p3p_solver.hpp"

namespace {

bool match_pose(const double R[9], const double t[3],
                const Eigen::Matrix3d& R_gt, const Eigen::Vector3d& t_gt,
                double tol_R, double tol_t) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (std::abs(R[i*3 + j] - R_gt(i, j)) > tol_R) return false;
    }
  }
  for (int i = 0; i < 3; ++i) {
    if (std::abs(t[i] - t_gt(i)) > tol_t) return false;
  }
  return true;
}

void run_one(std::mt19937_64& rng, bool& found, double& worst_R, double& worst_t) {
  std::uniform_real_distribution<double> dist_ang(-0.6, 0.6);   // rad
  std::uniform_real_distribution<double> dist_t(-1.0, 1.0);
  std::uniform_real_distribution<double> dist_pw_xy(-2.0, 2.0);
  std::uniform_real_distribution<double> dist_pw_z(3.0, 8.0);   // in front

  // Ground-truth pose (world → camera).
  const Eigen::AngleAxisd Rx(dist_ang(rng), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd Ry(dist_ang(rng), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd Rz(dist_ang(rng), Eigen::Vector3d::UnitZ());
  const Eigen::Matrix3d R_gt = (Rz * Ry * Rx).matrix();
  const Eigen::Vector3d t_gt(dist_t(rng), dist_t(rng), dist_t(rng) * 0.3);

  // Three world points whose camera-frame images are in front of the camera.
  Eigen::Matrix<double, 3, 3> Pw;
  for (int tries = 0; tries < 20; ++tries) {
    for (int k = 0; k < 3; ++k) {
      Pw(0, k) = dist_pw_xy(rng);
      Pw(1, k) = dist_pw_xy(rng);
      Pw(2, k) = dist_pw_z(rng);
    }
    // Verify all 3 camera-frame z's > 1.
    Eigen::Matrix<double, 3, 3> Xc;
    Xc.col(0) = R_gt * Pw.col(0) + t_gt;
    Xc.col(1) = R_gt * Pw.col(1) + t_gt;
    Xc.col(2) = R_gt * Pw.col(2) + t_gt;
    if (Xc(2,0) > 1.0 && Xc(2,1) > 1.0 && Xc(2,2) > 1.0) break;
  }
  // Compute bearings.
  double Pw_arr[3][3], f_arr[3][3];
  for (int k = 0; k < 3; ++k) {
    Pw_arr[k][0] = Pw(0, k); Pw_arr[k][1] = Pw(1, k); Pw_arr[k][2] = Pw(2, k);
    const Eigen::Vector3d xc = R_gt * Pw.col(k) + t_gt;
    const double n = xc.norm();
    f_arr[k][0] = xc(0) / n; f_arr[k][1] = xc(1) / n; f_arr[k][2] = xc(2) / n;
  }

  double R_out[4][9], t_out[4][3];
  const int n_sol = slamko_vio::p3p::solve(Pw_arr, f_arr, R_out, t_out);
  ASSERT_GT(n_sol, 0) << "P3P returned no solutions";

  found = false;
  double best_R_err = 1e9, best_t_err = 1e9;
  for (int s = 0; s < n_sol; ++s) {
    double R_err = 0.0;
    for (int j = 0; j < 9; ++j) {
      const int r = j / 3, c = j % 3;
      const double d = R_out[s][j] - R_gt(r, c);
      R_err += d*d;
    }
    R_err = std::sqrt(R_err);
    const double t_err = std::sqrt(
        (t_out[s][0]-t_gt(0))*(t_out[s][0]-t_gt(0)) +
        (t_out[s][1]-t_gt(1))*(t_out[s][1]-t_gt(1)) +
        (t_out[s][2]-t_gt(2))*(t_out[s][2]-t_gt(2)));
    if (R_err < best_R_err) { best_R_err = R_err; best_t_err = t_err; }
    if (R_err < 1e-3 && t_err < 1e-3) { found = true; }
  }
  worst_R = best_R_err;
  worst_t = best_t_err;
}

}  // namespace

TEST(P3P, RecoversGroundTruthOnSyntheticData) {
  std::mt19937_64 rng(42);
  int total = 0;
  int recovered = 0;
  double max_R_err = 0.0;
  double max_t_err = 0.0;
  for (int trial = 0; trial < 30; ++trial) {
    bool found = false;
    double bR = 1e9, bt = 1e9;
    run_one(rng, found, bR, bt);
    ++total;
    if (found) {
      ++recovered;
      if (bR > max_R_err) max_R_err = bR;
      if (bt > max_t_err) max_t_err = bt;
    } else {
      ADD_FAILURE() << "trial " << trial << " not recovered: best_R_err="
                    << bR << " best_t_err=" << bt;
    }
  }
  // At least 90% of random configurations should pass.
  EXPECT_GE(recovered * 10, total * 9)
      << "recovered " << recovered << "/" << total
      << " (max R err = " << max_R_err << ", max t err = " << max_t_err << ")";
}
