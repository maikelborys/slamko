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
  // Final Gauss-Newton Hessian H = Σ wᵢ Jᵢᵀ Jᵀ (information, [rho; omega] order). A degenerate
  // direction (a straight corridor's along-axis) shows as a SMALL eigenvalue — the read side of
  // the degeneracy-aware covariance for the loop edge (inflate cov along small-eigenvalue axes,
  // Zhang/X-ICP), and the gate that rejects a rank-deficient (geometrically unconstrained) match.
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
};

// Per-iteration result of a BATCH distance-field query (NaN dist = invalid / unmapped).
struct SdfBatch {
  std::vector<double> dist;            // signed distance per query point
  std::vector<Eigen::Vector3d> grad;   // ∇dist per point (≈ unit surface normal)
  std::vector<char> valid;             // 0 = no usable map at this point (drop it)
};

// Register `query_pts_local` (query/sensor frame) to the surface implied by `batch_field`,
// from `T_init` (map <- query). BatchField: f(const std::vector<Vector3d>& pts_map) ->
// SdfBatch (queried ALL AT ONCE so a GPU/nvblox-ESDF backend does one copy per iteration).
template <class BatchField>
SdfRegistrationResult registerToSdfBatch(const std::vector<Eigen::Vector3d>& query_pts_local,
                                         const SE3& T_init, BatchField&& batch_field,
                                         const SdfRegistrationConfig& cfg = {}) {
  SdfRegistrationResult res;
  res.T_refined = T_init;
  if (query_pts_local.empty()) return res;
  const std::size_t n = query_pts_local.size();

  SE3 T = T_init;
  std::vector<Eigen::Vector3d> xs(n);
  for (int it = 0; it < cfg.max_iters; ++it) {
    const Eigen::Matrix3d R = T.so3().matrix();
    for (std::size_t i = 0; i < n; ++i) xs[i] = T * query_pts_local[i];
    const SdfBatch f = batch_field(xs);

    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
    double sse = 0.0;
    int inl = 0;
    for (std::size_t i = 0; i < n; ++i) {
      if (!f.valid[i]) continue;
      const double r = f.dist[i];
      if (std::abs(r) > cfg.max_correspondence_dist) continue;  // outlier / unmapped
      const Eigen::Vector3d& p = query_pts_local[i];
      // d x / d ξ = R · [ I | -[p]_× ]  (twist [rho; omega], right perturbation T·exp(ξ)).
      Eigen::Matrix<double, 3, 6> dxdxi;
      dxdxi.leftCols<3>() = R;
      dxdxi.rightCols<3>() = -R * SO3::hat(p);
      const Eigen::Matrix<double, 1, 6> J = f.grad[i].transpose() * dxdxi;
      const double a = std::abs(r);
      const double w = a <= cfg.huber_delta ? 1.0 : cfg.huber_delta / a;  // Huber
      H.noalias() += w * J.transpose() * J;
      b.noalias() += w * J.transpose() * r;
      sse += w * r * r;
      ++inl;
    }
    res.inliers = inl;
    res.iters = it + 1;
    if (inl < cfg.min_inliers) break;  // not enough overlap — bail (caller rejects)
    res.rms = std::sqrt(sse / inl);
    res.information = H;               // pre-damping Hessian = the degeneracy-aware information
    H.diagonal().array() += 1e-9;  // Levenberg damping for conditioning
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

// Convenience wrapper for a PER-POINT analytic field f(p)->{dist,grad} (tests / CPU fields).
template <class DistanceField>
SdfRegistrationResult registerToSdf(const std::vector<Eigen::Vector3d>& query_pts_local,
                                    const SE3& T_init, const DistanceField& field,
                                    const SdfRegistrationConfig& cfg = {}) {
  return registerToSdfBatch(
      query_pts_local, T_init,
      [&field](const std::vector<Eigen::Vector3d>& xs) {
        SdfBatch out;
        out.dist.resize(xs.size());
        out.grad.resize(xs.size());
        out.valid.assign(xs.size(), 1);
        for (std::size_t i = 0; i < xs.size(); ++i) {
          auto df = field(xs[i]);
          out.dist[i] = df.first;
          out.grad[i] = df.second;
        }
        return out;
      },
      cfg);
}

}  // namespace slamko
