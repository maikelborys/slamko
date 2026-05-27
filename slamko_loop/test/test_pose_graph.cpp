// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// PoseGraph (P2.5) tests — fully synthetic, no ROS / no solver dep beyond Eigen.
// Hand-built anchors + relative-pose edges drive the Gauss-Newton sweep; exact SE3
// assertions validate (1) a single edge reduces ALGEBRAICALLY to the closed-form
// weld, (2) a consistent loop is recovered exactly, (3) conflicting edges settle on
// the geodesic mean, (4) an under-excited node is left untouched (LM stability),
// (5) a gross-outlier edge is dropped and the consensus recovered (Hard Rule #4).

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "slamko_core/se3.hpp"
#include "slamko_loop/pose_graph.hpp"

using slamko::Matrix6d;
using slamko::PoseGraph;
using slamko::PoseGraphConfig;
using slamko::PoseGraphEdge;
using slamko::PoseGraphResult;
using slamko::SE3;
using slamko::Vector6d;

namespace {

SE3 makeSE3(double tx, double ty, double tz, double angle, Eigen::Vector3d axis) {
  const Eigen::Matrix3d R =
      Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  return SE3(R, Eigen::Vector3d(tx, ty, tz));
}

void expectSE3Near(const SE3& a, const SE3& b, double tol) {
  const Eigen::Matrix4d A = a.matrix(), B = b.matrix();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      EXPECT_NEAR(A(i, j), B(i, j), tol) << "(" << i << "," << j << ")";
}

// The relative-pose measurement an edge from→to should carry: X_from⁻¹ · X_to.
PoseGraphEdge between(std::uint64_t f, std::uint64_t t, const SE3& Xf, const SE3& Xt) {
  PoseGraphEdge e;
  e.from_id = f; e.to_id = t;
  e.T_from_to = Xf.inverse() * Xt;
  return e;
}

}  // namespace

// (1) One fixed sealed node + one free active node + one edge Z = C. The GN solution
// MUST equal the closed-form weld anchor_active = anchor_sealed · C — the backward-
// compat property that lets the supervisor swap the composition for the graph.
TEST(PoseGraph, SingleEdgeReducesToClosedFormWeld) {
  const SE3 anchor_sealed = makeSE3(1.0, -2.0, 0.5, 0.4, {0, 0, 1});
  const SE3 C = makeSE3(0.2, 0.1, 0.0, 0.15, {0, 1, 0});  // active→sealed-local

  PoseGraph g;
  g.setNode(0, anchor_sealed, /*fixed=*/true);
  g.setNode(1, SE3(), /*fixed=*/false);  // start the active anchor at identity
  PoseGraphEdge e; e.from_id = 0; e.to_id = 1; e.T_from_to = C;
  g.addEdge(e);

  const PoseGraphResult r = g.optimize();
  EXPECT_TRUE(r.converged);
  EXPECT_LT(r.final_cost, 1e-14);
  expectSE3Near(g.node(1), anchor_sealed * C, 1e-9);  // == closed form
  expectSE3Near(g.node(0), anchor_sealed, 1e-12);     // gauge node unmoved
}

// (2) A consistent 3-submap loop (A fixed). Edges measured from the TRUE anchors;
// B,C start perturbed. The solver recovers the true anchors (cost → 0) — multi-node
// loop closure distributing the constraint.
TEST(PoseGraph, ConsistentLoopRecoveredExactly) {
  const SE3 XA = makeSE3(0, 0, 0, 0.0, {0, 0, 1});
  const SE3 XB = makeSE3(1.0, 0.0, 0.0, 0.30, {0, 0, 1});
  const SE3 XC = makeSE3(1.0, 1.0, 0.2, -0.20, {0, 1, 0});

  PoseGraph g;
  g.setNode(0, XA, true);
  g.setNode(1, XB * makeSE3(0.08, -0.05, 0.03, 0.06, {1, 0, 0}), false);  // perturbed
  g.setNode(2, XC * makeSE3(-0.04, 0.07, -0.02, -0.05, {0, 1, 0}), false);
  g.addEdge(between(0, 1, XA, XB));
  g.addEdge(between(1, 2, XB, XC));
  g.addEdge(between(2, 0, XC, XA));

  const PoseGraphResult r = g.optimize();
  EXPECT_TRUE(r.converged);
  EXPECT_LT(r.final_cost, 1e-12);
  EXPECT_LT(r.final_cost, r.initial_cost);
  expectSE3Near(g.node(1), XB, 1e-6);
  expectSE3Near(g.node(2), XC, 1e-6);
}

// (3) Two conflicting edges from a fixed node to one free node (equal information).
// The GN optimum balances them: the two from-frame residuals are equal-and-opposite
// (Σ rₖ = 0). We assert that optimality invariant directly — it's what the solver
// guarantees — plus that the solution genuinely blended (didn't sit on either edge)
// and that the cost dropped from the Z1-seeded start.
TEST(PoseGraph, ConflictingEdgesBalanceResiduals) {
  const SE3 F = makeSE3(0.5, 0.0, 0.0, 0.10, {0, 0, 1});
  const SE3 Z1 = makeSE3(1.0, 0.0, 0.0, 0.05, {0, 0, 1});
  const SE3 Z2 = makeSE3(1.0, 0.12, 0.0, 0.12, {0, 0, 1});  // a nearby, conflicting hit

  PoseGraph g;
  g.setNode(0, F, true);
  g.setNode(1, F * Z1, false);  // seeded exactly on edge 1 (r1 = 0, r2 ≠ 0 initially)
  PoseGraphEdge e1; e1.from_id = 0; e1.to_id = 1; e1.T_from_to = Z1;
  PoseGraphEdge e2; e2.from_id = 0; e2.to_id = 1; e2.T_from_to = Z2;
  g.addEdge(e1); g.addEdge(e2);

  const PoseGraphResult r = g.optimize();
  EXPECT_TRUE(r.converged);
  EXPECT_LT(r.final_cost, r.initial_cost);  // moved off the Z1 seed

  const SE3 Y = F.inverse() * g.node(1);
  const Vector6d r1 = (Z1.inverse() * Y).log();
  const Vector6d r2 = (Z2.inverse() * Y).log();
  EXPECT_LT((r1 + r2).norm(), 1e-7);   // balanced — the GN optimality condition
  EXPECT_GT(r1.norm(), 1e-3);          // genuinely blended, not parked on an edge
}

// (4) A free node with NO edge is under-excited — H has only the λ term for its
// block. The solver must stay stable and leave it where it started (δ = 0).
TEST(PoseGraph, UnexcitedNodeIsStable) {
  PoseGraph g;
  const SE3 anchor_sealed = makeSE3(0, 0, 0, 0.0, {0, 0, 1});
  const SE3 lonely = makeSE3(3.0, 4.0, 5.0, 0.7, {1, 1, 0});
  g.setNode(0, anchor_sealed, true);
  g.setNode(1, lonely, false);  // no edge touches node 1
  PoseGraphEdge e; e.from_id = 0; e.to_id = 0; e.T_from_to = SE3();  // self-loop noop

  const PoseGraphResult r = g.optimize();
  EXPECT_TRUE(r.converged);
  expectSE3Near(g.node(1), lonely, 1e-9);  // untouched, no blow-up
}

// (5) Three agreeing edges + one gross outlier to the same free node. With outlier
// rejection on, the worst edge is dropped and the good consensus recovered — the
// disposable-graph defense (Hard Rule #4): a bad loop closure can't stick.
TEST(PoseGraph, OutlierEdgeDroppedAndConsensusRecovered) {
  const SE3 F = makeSE3(0.0, 0.0, 0.0, 0.0, {0, 0, 1});
  const SE3 Zgood = makeSE3(2.0, 0.0, 0.0, 0.10, {0, 0, 1});
  const SE3 Zbad  = makeSE3(2.0, 9.0, 0.0, 2.50, {0, 0, 1});  // wildly inconsistent

  PoseGraph g;
  g.setNode(0, F, true);
  g.setNode(1, F * Zgood, false);
  for (int k = 0; k < 3; ++k) {                      // three agreeing constraints
    PoseGraphEdge e; e.from_id = 0; e.to_id = 1; e.T_from_to = Zgood;
    g.addEdge(e);
  }
  PoseGraphEdge bad; bad.from_id = 0; bad.to_id = 1; bad.T_from_to = Zbad;
  g.addEdge(bad);

  PoseGraphConfig cfg;
  cfg.outlier_chi2 = 1.0;       // a residual² above 1 (rad²+m²) is an outlier
  cfg.max_outlier_drops = 2;
  const PoseGraphResult r = g.optimize(cfg);

  EXPECT_GE(r.edges_dropped, 1);
  EXPECT_EQ(g.edgeCount(), 3u);                      // the outlier was removed
  expectSE3Near(g.node(1), F * Zgood, 1e-5);         // recovered the good consensus
}
