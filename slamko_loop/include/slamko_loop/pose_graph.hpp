// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// PoseGraph — the tiny, self-contained SE(3) pose-graph backend (P2.5). The nodes
// are submap ANCHORS (submap-local → global); the edges are loop-closure /
// revisit constraints expressed AS FACTORS — a relative-pose measurement
// `Z = anchor_from⁻¹ · anchor_to`. A Gauss-Newton sweep on the manifold finds the
// anchors that best satisfy all edges, distributing accumulated loop-closure error
// across the graph instead of jumping one anchor (which is what the closed-form
// weld does for a single constraint). This is what lets the never-lost spine merge
// MULTIPLE sealed submaps consistently and re-optimize continuous relocalizations.
//
// WHY HERE (not GTSAM): slamko_loop depends on slamko_core ONLY (Hard Rule #2), and
// the global graph is DISPOSABLE (Hard Rule #4) — it must catch → damp → drop a bad
// edge → rebuild, never crash the tracker. A ~200-line Eigen GN over `se3.hpp` keeps
// that property auditable and dodges the GTSAM SONAME fragility the fusion tier hit.
//
// CONVENTIONS (load-bearing):
//   * Right perturbation: a node updates as  X ← X · Exp(δ),  δ = [ρ;ω] (se3.hpp).
//   * Edge measurement Z (from→to): ideally  Z = X_from⁻¹ · X_to.
//     Residual  r = Log( Z⁻¹ · X_from⁻¹ · X_to ),  zero when the edge is satisfied.
//   * Jacobians use the small-residual approximation  J_r⁻¹(r) ≈ I  (right-Jac inv),
//     giving  ∂r/∂δ_from = -Adj(X_to⁻¹ · X_from)  and  ∂r/∂δ_to = I. This is EXACT at
//     a consistent optimum (all r→0, so the weighting drops out) — the regime the
//     never-lost weld lives in — and a standard, robust approximation off it. The win
//     condition here is map-merge robustness, not sub-cm MAP optimality (see
//     slamko-robustness-over-accuracy). The full SE(3) J_r⁻¹ is a drop-in later.
//   * Exactly one node must be FIXED (the gauge); else the global SE(3) is free. With
//     a single fixed node + one edge the solve reduces ALGEBRAICALLY to the closed-
//     form weld  X_to = X_from · Z  — the backward-compat property the tests assert.

#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "slamko_core/se3.hpp"

namespace slamko {

// A loop-closure / revisit constraint as a factor: the measured relative pose
// between two submap anchors, with an information (inverse-covariance) weight.
struct PoseGraphEdge {
  std::uint64_t from_id = 0;
  std::uint64_t to_id = 0;
  SE3 T_from_to;                              // measured  X_from⁻¹ · X_to
  Matrix6d information = Matrix6d::Identity(); // [ρ;ω] ordering; default isotropic
};

struct PoseGraphConfig {
  int    max_iters    = 25;
  double converge_dx2 = 1e-16;  // stop when ‖δ‖² (all free nodes) drops below this
  double init_lambda  = 1e-9;   // LM damping seed (added to H diagonal each solve)
  double lambda_up    = 10.0;   // multiply λ by this when a solve step fails/diverges
  int    max_lm_tries = 8;      // damping retries before declaring non-convergence
  // Outlier rejection (Hard Rule #4): after convergence, if > 0, drop the single
  // worst edge whose chi² (rᵀΩr) exceeds this and re-optimize; repeat up to
  // max_outlier_drops. 0 disables (keep every edge).
  double outlier_chi2     = 0.0;
  int    max_outlier_drops = 0;
};

struct PoseGraphResult {
  bool   converged = false;
  int    iterations = 0;
  double initial_cost = 0.0;   // Σ rᵀΩr before optimization
  double final_cost = 0.0;     // Σ rᵀΩr after
  int    edges_dropped = 0;
  int    lambda_bumps = 0;     // how many times damping had to be raised
};

class PoseGraph {
 public:
  // Add/replace a node (submap id → anchor). Mark exactly one node fixed = the
  // gauge. If none is marked fixed when optimize() runs, the lowest id is fixed.
  void setNode(std::uint64_t id, const SE3& anchor, bool fixed = false);
  void addEdge(const PoseGraphEdge& e) { edges_.push_back(e); }
  void clear() { nodes_.clear(); fixed_.clear(); edges_.clear(); }

  bool   hasNode(std::uint64_t id) const { return nodes_.count(id) != 0; }
  SE3    node(std::uint64_t id) const;            // identity if absent
  std::size_t nodeCount() const { return nodes_.size(); }
  std::size_t edgeCount() const { return edges_.size(); }
  const std::vector<PoseGraphEdge>& edges() const { return edges_; }

  // Gauss-Newton on the manifold. Never throws: a non-SPD/ill-conditioned step
  // raises LM damping and retries; a gross-outlier edge is dropped and the graph
  // rebuilt. On return, node() reflects the optimized anchors.
  PoseGraphResult optimize(const PoseGraphConfig& cfg = {});

 private:
  double totalCost() const;                       // Σ rᵀΩr over all edges
  // One GN linearize+solve; writes the per-node increments into dx (keyed by id),
  // returns false if the linear system could not be solved at this λ.
  bool linearizeSolve(double lambda,
                      std::unordered_map<std::uint64_t, Vector6d>& dx) const;

  std::unordered_map<std::uint64_t, SE3> nodes_;
  std::unordered_map<std::uint64_t, bool> fixed_;
  std::vector<PoseGraphEdge> edges_;
};

}  // namespace slamko
