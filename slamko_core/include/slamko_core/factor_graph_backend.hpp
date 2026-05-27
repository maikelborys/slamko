// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// FactorGraphBackend — owns the nodes, the solve, and the marginalization.
// Swappable adapter (MASTER_PLAN §8.2): GtsamBackend (iSAM2, default) or
// CeresBackend (visual-only S1 inner loop / fallback). Frontends only ever see
// this interface, so the solver stays interchangeable.
//
// marginalCovariance() is the observability monitor that feeds the health
// signals (slamko_core/health.hpp); marginalizeOlderThan() is the Schur-prior +
// FEJ window edge — the biggest quality upgrade over klt_vo's gauge-by-constant
// (MASTER_PLAN §3).

#pragma once

#include "slamko_core/factor.hpp"
#include "slamko_core/node_key.hpp"

#include <Eigen/Core>

namespace slamko {

class FactorGraphBackend {
 public:
  virtual ~FactorGraphBackend() = default;

  // Create a node of the given type with an initial value. `constant` pins it
  // (gauge anchor / fixed extrinsics). Returns its NodeKey (id assigned here).
  virtual NodeKey addNode(NodeType type, const Eigen::VectorXd& init,
                          bool constant = false) = 0;

  virtual void addFactor(const FactorPtr& factor) = 0;

  // Run (incremental) optimization. Returns false on numerical failure — the
  // caller (never-lost supervisor) decides whether to damp/rebuild; the
  // backend itself must not crash (MASTER_PLAN §0, principle 3).
  virtual bool optimize() = 0;

  virtual Eigen::VectorXd value(NodeKey key) const = 0;

  // Marginal covariance of a node — the observability / degeneracy probe.
  virtual Eigen::MatrixXd marginalCovariance(NodeKey key) const = 0;

  // Drop nodes older than `stamp`, replacing them with a Schur-complement
  // linear prior on the separator (First-Estimates-Jacobian consistency).
  virtual void marginalizeOlderThan(double stamp) = 0;
};

}  // namespace slamko
