// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// SdfRegistration — point-to-SDF ICP: align a DENSE observation (depth point cloud)
// against an already-mapped SIGNED-DISTANCE FIELD. WHY this is the strong geometric
// loop/recovery channel the sparse ScanContext is not: at a revisit (or after a brusque-
// motion / blackout divergence), the robot is physically at a place whose 3D surfaces are
// ALREADY in the map (nvblox TSDF/ESDF). The current depth cloud, placed at the drifted
// pose estimate, sits OFFSET from those surfaces; minimizing the SDF value at each cloud
// point (the residual = signed distance to the nearest surface, 0 when aligned) refines
// the pose so the cloud snaps onto the existing geometry — and that refinement IS the
// drift/loop correction. Unlike appearance (viewpoint-dependent) and sparse ScanContext
// (thin FOV overlap), a room's dense 3D shape is the same regardless of facing, so this
// recovers different-heading revisits with shared structure. It is the OKVIS SubmapIcpError
// / Voxgraph field-align pattern (field-gradient > naive point-ICP), the documented
// stronger channel (RESEARCH_LIFELONG_NAV_ARCH §5).
//
// DECOUPLED (Hard Rule #2): the math is templated on a DistanceField FUNCTOR
//   f(const Eigen::Vector3d& p_map) -> std::pair<double dist, Eigen::Vector3d grad>
// so it is unit-testable against an analytic field and the nvblox ESDF query (slamko_tsdf)
// plugs in behind the same functor. Gauss-Newton on SE3 with a Huber robust kernel; points
// whose |SDF| exceeds the correspondence gate are dropped (occlusions / unmapped area).

#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "slamko_core/se3.hpp"

namespace slamko {

struct SdfRegistrationConfig {
  int max_iters = 25;
  double huber_delta = 0.10;             // m — robust kernel knee on the SDF residual
  double max_correspondence_dist = 0.5;  // m — drop a point whose |SDF| exceeds this
  double convergence_dx = 1e-4;          // stop when the SE3 step norm falls below this
  int min_inliers = 30;                  // below this the result is not trustworthy
};

struct SdfRegistrationResult {
  SE3 T_refined;          // refined map <- query transform
  double rms = 0.0;       // final RMS |SDF| over inliers (m) — the fitness (lower = better)
  int inliers = 0;        // points within the correspondence gate at convergence
  int iters = 0;
  bool converged = false; // step fell below convergence_dx AND inliers >= min_inliers
};

// Register `query_pts_local` (in the query/sensor frame) to the surface implied by `field`,
// starting from `T_init` (map <- query). DistanceField: p_map -> {signed_dist, gradient}.
template <class DistanceField>
SdfRegistrationResult registerToSdf(const std::vector<Eigen::Vector3d>& query_pts_local,
                                    const SE3& T_init, const DistanceField& field,
                                    const SdfRegistrationConfig& cfg = {}) {
  SdfRegistrationResult res;
  res.T_refined = T_init;
  if (query_pts_local.empty()) return res;

  SE3 T = T_init;
  for (int it = 0; it < cfg.max_iters; ++it) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
    const Eigen::Matrix3d R = T.so3().matrix();
    double sse = 0.0;
    int inl = 0;
    for (const auto& p : query_pts_local) {
      const Eigen::Vector3d x = T * p;            // point in map frame
      auto df = field(x);
      const double r = df.first;                  // signed distance (residual, target 0)
      if (std::abs(r) > cfg.max_correspondence_dist) continue;  // outlier / unmapped
      const Eigen::Vector3d& g = df.second;       // ∇dist (≈ unit surface normal)
      // d x / d ξ = R · [ I | -[p]_× ]  (twist [rho; omega], right perturbation T·exp(ξ)).
      Eigen::Matrix<double, 3, 6> dxdxi;
      dxdxi.leftCols<3>() = R;
      dxdxi.rightCols<3>() = -R * SO3::hat(p);
      const Eigen::Matrix<double, 1, 6> J = g.transpose() * dxdxi;  // dr/dξ
      // Huber weight on the residual.
      const double a = std::abs(r);
      const double w = a <= cfg.huber_delta ? 1.0 : cfg.huber_delta / a;
      H.noalias() += w * J.transpose() * J;
      b.noalias() += w * J.transpose() * r;
      sse += w * r * r;
      ++inl;
    }
    res.inliers = inl;
    res.iters = it + 1;
    if (inl < cfg.min_inliers) break;  // not enough overlap — bail (caller rejects)
    res.rms = std::sqrt(sse / inl);
    // Damp slightly (Levenberg) for conditioning, then solve δξ = -H⁻¹ b.
    H.diagonal().array() += 1e-9;
    const Eigen::Matrix<double, 6, 1> dxi = H.ldlt().solve(-b);
    T = T * SE3::exp(dxi);
    res.T_refined = T;
    if (dxi.norm() < cfg.convergence_dx) {
      res.converged = inl >= cfg.min_inliers;
      break;
    }
  }
  return res;
}

}  // namespace slamko
