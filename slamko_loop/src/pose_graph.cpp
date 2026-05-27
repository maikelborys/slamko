// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// PoseGraph — see header for the convention + Jacobian derivation. Gauss-Newton
// with Levenberg-Marquardt damping over slamko_core's SE(3), plus the disposable-
// graph robustness of Hard Rule #4 (catch → damp → drop bad edge → rebuild).

#include "slamko_loop/pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace slamko {

namespace {

// Edge residual r = Log( Z⁻¹ · X_from⁻¹ · X_to ); zero when the edge is satisfied.
Vector6d edgeResidual(const PoseGraphEdge& e, const SE3& Xf, const SE3& Xt) {
  return (e.T_from_to.inverse() * Xf.inverse() * Xt).log();
}

bool allFinite(const Eigen::VectorXd& v) {
  for (int i = 0; i < v.size(); ++i)
    if (!std::isfinite(v(i))) return false;
  return true;
}

}  // namespace

void PoseGraph::setNode(std::uint64_t id, const SE3& anchor, bool fixed) {
  nodes_[id] = anchor;
  fixed_[id] = fixed;
}

SE3 PoseGraph::node(std::uint64_t id) const {
  auto it = nodes_.find(id);
  return it == nodes_.end() ? SE3() : it->second;
}

double PoseGraph::totalCost() const {
  double c = 0.0;
  for (const auto& e : edges_) {
    auto f = nodes_.find(e.from_id), t = nodes_.find(e.to_id);
    if (f == nodes_.end() || t == nodes_.end()) continue;
    const Vector6d r = edgeResidual(e, f->second, t->second);
    c += r.dot(e.information * r);
  }
  return c;
}

bool PoseGraph::linearizeSolve(
    double lambda, std::unordered_map<std::uint64_t, Vector6d>& dx) const {
  dx.clear();
  // Index the FREE (non-fixed) nodes into contiguous 6-blocks.
  std::unordered_map<std::uint64_t, int> idx;
  for (const auto& kv : nodes_) {
    auto fx = fixed_.find(kv.first);
    if (fx != fixed_.end() && fx->second) continue;  // gauge node, not a variable
    const int k = static_cast<int>(idx.size());
    idx[kv.first] = k;
  }
  const int n = static_cast<int>(idx.size());
  if (n == 0) return true;  // fully constrained — nothing to solve

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6 * n, 6 * n);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(6 * n);

  for (const auto& e : edges_) {
    auto fIt = nodes_.find(e.from_id), tIt = nodes_.find(e.to_id);
    if (fIt == nodes_.end() || tIt == nodes_.end()) continue;
    const SE3& Xf = fIt->second;
    const SE3& Xt = tIt->second;

    const Vector6d r = edgeResidual(e, Xf, Xt);
    // ∂r/∂δ_from = -Adj(X_to⁻¹·X_from) ; ∂r/∂δ_to = I (J_r⁻¹≈I, see header).
    const Matrix6d A = -((Xt.inverse() * Xf).adjoint());
    const Matrix6d& Om = e.information;

    auto fFree = idx.find(e.from_id);
    auto tFree = idx.find(e.to_id);
    const bool hasF = fFree != idx.end();
    const bool hasT = tFree != idx.end();
    const int fi = hasF ? 6 * fFree->second : -1;
    const int ti = hasT ? 6 * tFree->second : -1;

    const Matrix6d OmA = Om * A;             // B = I, so Om*B = Om
    if (hasF) {
      H.block<6, 6>(fi, fi) += A.transpose() * OmA;
      b.segment<6>(fi)      += A.transpose() * (Om * r);
    }
    if (hasT) {
      H.block<6, 6>(ti, ti) += Om;           // Bᵀ Ω B = Ω
      b.segment<6>(ti)      += Om * r;        // Bᵀ Ω r = Ω r
    }
    if (hasF && hasT) {
      H.block<6, 6>(fi, ti) += A.transpose() * Om;   // Aᵀ Ω B
      H.block<6, 6>(ti, fi) += OmA;                  // Bᵀ Ω A = Ω A
    }
  }

  // LM damping on the diagonal keeps H SPD even when the graph is under-excited.
  H.diagonal().array() += lambda;

  Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
  if (ldlt.info() != Eigen::Success) return false;
  const Eigen::VectorXd delta = ldlt.solve(-b);
  if (ldlt.info() != Eigen::Success || !allFinite(delta)) return false;

  for (const auto& kv : idx)
    dx[kv.first] = delta.segment<6>(6 * kv.second);
  return true;
}

PoseGraphResult PoseGraph::optimize(const PoseGraphConfig& cfg) {
  PoseGraphResult res;
  if (nodes_.empty()) { res.converged = true; return res; }

  // Ensure a gauge: if nothing is fixed, pin the lowest id (deterministic).
  bool anyFixed = false;
  std::uint64_t lowest = std::numeric_limits<std::uint64_t>::max();
  for (const auto& kv : fixed_) {
    anyFixed = anyFixed || kv.second;
    lowest = std::min(lowest, kv.first);
  }
  if (!anyFixed && !nodes_.empty()) fixed_[lowest] = true;

  res.initial_cost = totalCost();

  // Outer loop: optimize, then optionally drop the single worst outlier edge and
  // re-optimize (the disposable-graph defense — a bad loop closure can't stick).
  for (int drop = 0; ; ++drop) {
    double lambda = cfg.init_lambda;
    double cost = totalCost();

    for (int iter = 0; iter < cfg.max_iters; ++iter) {
      bool stepped = false, solve_ok = false;
      double dx2 = 0.0;
      for (int tryi = 0; tryi < cfg.max_lm_tries; ++tryi) {
        std::unordered_map<std::uint64_t, Vector6d> dx;
        if (!linearizeSolve(lambda, dx)) {
          lambda *= cfg.lambda_up; ++res.lambda_bumps; continue;
        }
        solve_ok = true;
        // Trial step on a snapshot; accept only if the cost does not increase
        // (a relative floor absorbs round-off near the minimum).
        const auto saved = nodes_;
        double d2 = 0.0;
        for (const auto& kv : dx) {
          nodes_[kv.first] = nodes_[kv.first] * SE3::exp(kv.second);
          d2 += kv.second.squaredNorm();
        }
        const double newcost = totalCost();
        if (std::isfinite(newcost) && newcost <= cost * (1.0 + 1e-12) + 1e-15) {
          cost = newcost;
          lambda = std::max(lambda / cfg.lambda_up, cfg.init_lambda);
          dx2 = d2; stepped = true; break;
        }
        nodes_ = saved;                       // reject: restore + damp harder
        lambda *= cfg.lambda_up; ++res.lambda_bumps;
      }
      // No improving step found: if the linear system DID solve, we're sitting at a
      // (local) minimum to numerical precision → converged. If it never solved at
      // any damping, that's a genuine failure — leave converged=false.
      if (!stepped) { res.converged = solve_ok; break; }
      ++res.iterations;
      if (dx2 < cfg.converge_dx2) { res.converged = true; break; }
    }

    // Outlier handling: find the worst edge by chi², drop it if it exceeds the gate.
    if (cfg.outlier_chi2 <= 0.0 || drop >= cfg.max_outlier_drops) break;
    int worst = -1; double worst_chi2 = cfg.outlier_chi2;
    for (int i = 0; i < static_cast<int>(edges_.size()); ++i) {
      auto f = nodes_.find(edges_[i].from_id), t = nodes_.find(edges_[i].to_id);
      if (f == nodes_.end() || t == nodes_.end()) continue;
      const Vector6d r = edgeResidual(edges_[i], f->second, t->second);
      const double chi2 = r.dot(edges_[i].information * r);
      if (chi2 > worst_chi2) { worst_chi2 = chi2; worst = i; }
    }
    if (worst < 0) break;                     // no edge above the gate — done
    edges_.erase(edges_.begin() + worst);
    ++res.edges_dropped;
    res.converged = false;                    // re-prove convergence post-drop
  }

  res.final_cost = totalCost();
  if (res.iterations == 0) res.converged = true;  // already at the optimum
  return res;
}

}  // namespace slamko
