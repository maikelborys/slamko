// SPDX-License-Identifier: Apache-2.0
// Standalone debug harness for slamko_vio::p3p::solve.

#define KLT_VO_P3P_DEBUG 0

#include <cmath>
#include <cstdio>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slamko_vio/p3p_solver.hpp"

int main() {
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> dist_ang(-0.6, 0.6);
  std::uniform_real_distribution<double> dist_t(-1.0, 1.0);
  std::uniform_real_distribution<double> dist_pw_xy(-2.0, 2.0);
  std::uniform_real_distribution<double> dist_pw_z(3.0, 8.0);

  const Eigen::AngleAxisd Rx(dist_ang(rng), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd Ry(dist_ang(rng), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd Rz(dist_ang(rng), Eigen::Vector3d::UnitZ());
  const Eigen::Matrix3d R_gt = (Rz * Ry * Rx).matrix();
  const Eigen::Vector3d t_gt(dist_t(rng), dist_t(rng), dist_t(rng) * 0.3);

  printf("R_gt =\n%.6f %.6f %.6f\n%.6f %.6f %.6f\n%.6f %.6f %.6f\n",
         R_gt(0,0), R_gt(0,1), R_gt(0,2),
         R_gt(1,0), R_gt(1,1), R_gt(1,2),
         R_gt(2,0), R_gt(2,1), R_gt(2,2));
  printf("t_gt = %.6f %.6f %.6f\n", t_gt(0), t_gt(1), t_gt(2));

  Eigen::Matrix<double, 3, 3> Pw;
  for (int k = 0; k < 3; ++k) {
    Pw(0, k) = dist_pw_xy(rng);
    Pw(1, k) = dist_pw_xy(rng);
    Pw(2, k) = dist_pw_z(rng);
  }
  printf("Pw =\n");
  for (int k = 0; k < 3; ++k) {
    printf("  %.4f %.4f %.4f\n", Pw(0,k), Pw(1,k), Pw(2,k));
  }

  double Pw_arr[3][3], f_arr[3][3];
  for (int k = 0; k < 3; ++k) {
    Pw_arr[k][0] = Pw(0, k); Pw_arr[k][1] = Pw(1, k); Pw_arr[k][2] = Pw(2, k);
    const Eigen::Vector3d xc = R_gt * Pw.col(k) + t_gt;
    const double n = xc.norm();
    f_arr[k][0] = xc(0) / n; f_arr[k][1] = xc(1) / n; f_arr[k][2] = xc(2) / n;
    printf("f[%d] = %.6f %.6f %.6f  (Xc z=%.3f)\n", k,
           f_arr[k][0], f_arr[k][1], f_arr[k][2], xc(2));
  }

  double R_out[4][9], t_out[4][3];
  const int n_sol = slamko_vio::p3p::solve(Pw_arr, f_arr, R_out, t_out);
  printf("n_solutions = %d\n", n_sol);
  for (int s = 0; s < n_sol; ++s) {
    printf("Solution %d:\n", s);
    printf("  R = %.6f %.6f %.6f / %.6f %.6f %.6f / %.6f %.6f %.6f\n",
           R_out[s][0], R_out[s][1], R_out[s][2],
           R_out[s][3], R_out[s][4], R_out[s][5],
           R_out[s][6], R_out[s][7], R_out[s][8]);
    printf("  t = %.6f %.6f %.6f\n", t_out[s][0], t_out[s][1], t_out[s][2]);

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
    printf("  R_err = %.6g  t_err = %.6g\n", R_err, t_err);
  }
  return 0;
}
