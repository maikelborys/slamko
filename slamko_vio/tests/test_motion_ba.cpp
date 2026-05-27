// SPDX-License-Identifier: Apache-2.0
// Unit test for slamko_vio::motion_ba::solve_motion_only_cpu.
//
// Synthesize a stereo scene with known (R_gt, t_gt), project to noisy left+
// right pixel observations, perturb the initial pose, and verify the LM
// solver recovers the ground truth within tolerance.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <random>
#include <vector>

#include "slamko_vio/motion_ba_solver.hpp"

namespace {

// Convert a 3×3 Eigen R to a row-major float[9].
void eigen_to_R(const Eigen::Matrix3d& R_eig, float R[9]) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      R[i*3 + j] = (float)R_eig(i, j);
}

void run_one(std::mt19937_64& rng, bool& ok, double& R_err, double& t_err,
             double& cost_drop_ratio) {
  std::uniform_real_distribution<double> dist_ang(-0.3, 0.3);
  std::uniform_real_distribution<double> dist_t(-0.5, 0.5);
  std::uniform_real_distribution<double> dist_xy(-2.0, 2.0);
  std::uniform_real_distribution<double> dist_z(3.0, 8.0);
  std::uniform_real_distribution<double> dist_noise(-0.5, 0.5);  // px
  std::uniform_real_distribution<double> dist_pert_ang(-0.05, 0.05);
  std::uniform_real_distribution<double> dist_pert_t(-0.1, 0.1);

  // EuRoC-like intrinsics.
  const float fx = 458.654f, fy = 457.296f;
  const float cx = 367.215f, cy = 248.375f;
  const float baseline = 0.110f;

  // Ground-truth pose (world → camera).
  const Eigen::AngleAxisd Rx(dist_ang(rng), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd Ry(dist_ang(rng), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd Rz(dist_ang(rng), Eigen::Vector3d::UnitZ());
  const Eigen::Matrix3d R_gt = (Rz * Ry * Rx).matrix();
  const Eigen::Vector3d t_gt(dist_t(rng), dist_t(rng), dist_t(rng) * 0.3);

  // Build N observations.
  const int N = 50;
  std::vector<slamko_vio::motion_ba::Obs> obs;
  obs.reserve(N);
  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d Pw(dist_xy(rng), dist_xy(rng), dist_z(rng));
    const Eigen::Vector3d Xc = R_gt * Pw + t_gt;
    if (Xc.z() < 1.0) continue;
    slamko_vio::motion_ba::Obs o;
    o.Xw[0] = (float)Pw.x();
    o.Xw[1] = (float)Pw.y();
    o.Xw[2] = (float)Pw.z();
    // Left projection with noise.
    const double inv_z = 1.0 / Xc.z();
    o.u_l = (float)(fx * Xc.x() * inv_z + cx + dist_noise(rng));
    o.v_l = (float)(fy * Xc.y() * inv_z + cy + dist_noise(rng));
    // Right: x - baseline.
    o.u_r = (float)(fx * (Xc.x() - baseline) * inv_z + cx + dist_noise(rng));
    o.v_r = (float)(fy * Xc.y() * inv_z + cy + dist_noise(rng));
    o.has_right = true;
    obs.push_back(o);
  }
  ASSERT_GT((int)obs.size(), 20);

  // Initial estimate: GT + small perturbation.
  Eigen::Vector3d perturb_aa(dist_pert_ang(rng), dist_pert_ang(rng), dist_pert_ang(rng));
  Eigen::AngleAxisd perturb(perturb_aa.norm() > 1e-12 ? perturb_aa.norm() : 0,
                            perturb_aa.norm() > 1e-12 ? perturb_aa.normalized()
                                                      : Eigen::Vector3d::UnitX());
  Eigen::Matrix3d R_init_eig = perturb * R_gt;
  Eigen::Vector3d t_init_eig = t_gt + Eigen::Vector3d(dist_pert_t(rng),
                                                       dist_pert_t(rng),
                                                       dist_pert_t(rng));
  float R[9], t[3];
  eigen_to_R(R_init_eig, R);
  t[0] = (float)t_init_eig.x();
  t[1] = (float)t_init_eig.y();
  t[2] = (float)t_init_eig.z();

  // Initial cost.
  auto cost_at = [&](const float Rx[9], const float tx[3]) {
    double total = 0.0;
    for (const auto& o : obs) {
      float r[4], J[24];
      float s = slamko_vio::motion_ba::compute_residual_jacobian(
          o.Xw, Rx, tx, fx, fy, cx, cy, baseline,
          o.u_l, o.v_l, o.u_r, o.v_r, o.has_right, r, J);
      if (s < 0.f) total += 1e9;
      else total += s;
    }
    return total;
  };
  const double cost_init = cost_at(R, t);

  // Solve.
  bool solver_ok = slamko_vio::motion_ba::solve_motion_only_cpu(
      obs.data(), (int)obs.size(),
      fx, fy, cx, cy, baseline,
      /*cauchy_c=*/3.0f, /*max_iters=*/15,
      R, t);
  ASSERT_TRUE(solver_ok);

  const double cost_final = cost_at(R, t);
  cost_drop_ratio = cost_final / cost_init;

  // Compare to GT.
  double Re = 0.0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      const double d = R[i*3 + j] - R_gt(i, j);
      Re += d*d;
    }
  R_err = std::sqrt(Re);
  t_err = std::sqrt(
      (t[0]-t_gt.x())*(t[0]-t_gt.x()) +
      (t[1]-t_gt.y())*(t[1]-t_gt.y()) +
      (t[2]-t_gt.z())*(t[2]-t_gt.z()));
  ok = (R_err < 5e-3) && (t_err < 5e-3);  // ~0.3° rot, 5 mm trans
}

}  // namespace

TEST(MotionBA, RecoversGroundTruthFromPerturbedInit) {
  std::mt19937_64 rng(42);
  int total = 0, recovered = 0;
  double max_R = 0.0, max_t = 0.0, max_cost_ratio = 0.0;
  for (int trial = 0; trial < 30; ++trial) {
    bool ok = false;
    double R_err, t_err, cost_drop_ratio;
    run_one(rng, ok, R_err, t_err, cost_drop_ratio);
    ++total;
    if (ok) {
      ++recovered;
      if (R_err > max_R) max_R = R_err;
      if (t_err > max_t) max_t = t_err;
      if (cost_drop_ratio > max_cost_ratio) max_cost_ratio = cost_drop_ratio;
    } else {
      ADD_FAILURE() << "trial " << trial << " R_err=" << R_err
                    << " t_err=" << t_err
                    << " cost_ratio=" << cost_drop_ratio;
    }
  }
  EXPECT_GE(recovered * 10, total * 9)
      << "Recovered " << recovered << "/" << total
      << " (max R err = " << max_R << ", max t err = " << max_t
      << ", max cost ratio = " << max_cost_ratio << ")";
}
