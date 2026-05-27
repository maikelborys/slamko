// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Stress + robustness tests for the P2.5 pose-graph backend and the never-lost
// supervisor at scale. Where the unit tests prove the math on toy graphs, these
// hammer the properties that matter for a NEVER-LOST system: large graphs converge,
// many loops stay consistent, many outliers all get dropped, the solver is
// deterministic and idempotent, an under-constrained (gauge-free) component never
// blows up, and the supervisor survives many seal→branch→weld cycles + flapping
// health without thrashing or unbounded edge growth. All synthetic, no ROS.

#include <cmath>
#include <deque>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "slamko_core/features.hpp"
#include "slamko_core/relocalizer.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"

#include "slamko_loop/never_lost_supervisor.hpp"
#include "slamko_loop/pose_graph.hpp"

using slamko::EstimationFrame;
using slamko::Features;
using slamko::HealthSignal;
using slamko::NeverLostSupervisor;
using slamko::PoseGraph;
using slamko::PoseGraphConfig;
using slamko::PoseGraphEdge;
using slamko::RecoveryAction;
using slamko::RelocResult;
using slamko::SE3;
using slamko::SubMap;
using slamko::SupervisorConfig;
using slamko::SupervisorState;
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
    for (int j = 0; j < 4; ++j) EXPECT_NEAR(A(i, j), B(i, j), tol);
}

bool isFinite(const SE3& T) { return T.matrix().allFinite(); }

PoseGraphEdge between(std::uint64_t f, std::uint64_t t, const SE3& Xf, const SE3& Xt) {
  PoseGraphEdge e;
  e.from_id = f; e.to_id = t;
  e.T_from_to = Xf.inverse() * Xt;
  return e;
}

// Right-perturb a pose by a Gaussian tangent step (deterministic via the passed rng).
SE3 perturb(std::mt19937& rng, const SE3& X, double sigma) {
  std::normal_distribution<double> n(0.0, sigma);
  Vector6d xi;
  for (int i = 0; i < 6; ++i) xi(i) = n(rng);
  return X * SE3::exp(xi);
}

}  // namespace

// --- PoseGraph at scale -------------------------------------------------------

// A 30-submap chain + one loop-closure edge (0↔29), every free node perturbed. The
// global solve must recover all anchors (consistent ⇒ zero-residual optimum).
TEST(PoseGraphStress, LongChainWithLoopClosure) {
  const int N = 30;
  std::vector<SE3> X(N);
  X[0] = SE3();
  for (int k = 1; k < N; ++k)
    X[k] = X[k - 1] * makeSE3(0.5, 0.1 * std::sin(0.3 * k), 0.02 * k, 0.08,
                              {0.2, 1.0, 0.3});

  PoseGraph g;
  std::mt19937 rng(7);
  g.setNode(0, X[0], /*fixed=*/true);
  for (int k = 1; k < N; ++k) g.setNode(k, perturb(rng, X[k], 0.03), false);
  for (int k = 0; k + 1 < N; ++k) g.addEdge(between(k, k + 1, X[k], X[k + 1]));
  g.addEdge(between(0, N - 1, X[0], X[N - 1]));  // loop closure

  PoseGraphConfig cfg; cfg.max_iters = 100;
  const auto r = g.optimize(cfg);
  EXPECT_TRUE(r.converged);
  EXPECT_LT(r.final_cost, 1e-6);
  EXPECT_LT(r.final_cost, r.initial_cost);
  expectSE3Near(g.node(N / 2), X[N / 2], 1e-4);
  expectSE3Near(g.node(N - 1), X[N - 1], 1e-4);
}

// A 5×5 grid with horizontal + vertical edges = 16 independent loops. Perturb all
// free nodes; the solve must converge to the consistent configuration.
TEST(PoseGraphStress, GridWithManyLoops) {
  const int G = 5;
  auto id = [G](int i, int j) { return static_cast<std::uint64_t>(i * G + j); };
  auto truth = [](int i, int j) {
    return makeSE3(1.0 * i, 1.0 * j, 0.0, 0.04 * (i + j), {0, 0, 1});
  };

  PoseGraph g;
  std::mt19937 rng(11);
  for (int i = 0; i < G; ++i)
    for (int j = 0; j < G; ++j)
      g.setNode(id(i, j), (i == 0 && j == 0) ? truth(0, 0)
                                             : perturb(rng, truth(i, j), 0.03),
                /*fixed=*/i == 0 && j == 0);
  for (int i = 0; i < G; ++i)
    for (int j = 0; j < G; ++j) {
      if (j + 1 < G) g.addEdge(between(id(i, j), id(i, j + 1), truth(i, j), truth(i, j + 1)));
      if (i + 1 < G) g.addEdge(between(id(i, j), id(i + 1, j), truth(i, j), truth(i + 1, j)));
    }

  PoseGraphConfig cfg; cfg.max_iters = 120;
  const auto r = g.optimize(cfg);
  EXPECT_TRUE(r.converged);
  EXPECT_LT(r.final_cost, 1e-5);
  expectSE3Near(g.node(id(G - 1, G - 1)), truth(G - 1, G - 1), 1e-3);
}

// A star of 6 good edges + 5 gross outliers to one free node. Outlier rejection must
// drop every outlier (worst-χ² first) and recover the good consensus.
TEST(PoseGraphStress, ManyOutliersAllDropped) {
  const SE3 F = SE3();
  const SE3 Zgood = makeSE3(2.0, 0.0, 0.0, 0.10, {0, 0, 1});
  PoseGraph g;
  g.setNode(0, F, true);
  g.setNode(1, F * Zgood, false);
  for (int k = 0; k < 6; ++k) { PoseGraphEdge e; e.from_id = 0; e.to_id = 1; e.T_from_to = Zgood; g.addEdge(e); }
  // five distinct, wildly-wrong edges
  const double bad[5][4] = {{9, 0, 0, 2.0}, {0, 9, 0, -2.2}, {0, 0, 9, 1.8},
                            {7, -7, 0, 2.5}, {-8, 0, 8, -1.5}};
  for (auto& b : bad) {
    PoseGraphEdge e; e.from_id = 0; e.to_id = 1;
    e.T_from_to = makeSE3(b[0], b[1], b[2], b[3], {0, 1, 0});
    g.addEdge(e);
  }

  PoseGraphConfig cfg; cfg.outlier_chi2 = 1.0; cfg.max_outlier_drops = 5;
  const auto r = g.optimize(cfg);
  EXPECT_EQ(r.edges_dropped, 5);
  EXPECT_EQ(g.edgeCount(), 6u);             // only the good edges remain
  expectSE3Near(g.node(1), F * Zgood, 1e-4);
}

// The solver uses no RNG — two identical graphs must produce bit-identical anchors
// (the determinism the flaky VIO harness lacks; here it's guaranteed).
TEST(PoseGraphStress, Deterministic) {
  auto build = [](PoseGraph& g) {
    std::mt19937 rng(99);  // same seed both times ⇒ identical perturbation
    const int N = 12;
    std::vector<SE3> X(N);
    X[0] = SE3();
    for (int k = 1; k < N; ++k) X[k] = X[k - 1] * makeSE3(0.4, 0.05, 0.0, 0.06, {0, 0, 1});
    g.setNode(0, X[0], true);
    for (int k = 1; k < N; ++k) g.setNode(k, perturb(rng, X[k], 0.05), false);
    for (int k = 0; k + 1 < N; ++k) g.addEdge(between(k, k + 1, X[k], X[k + 1]));
    g.addEdge(between(0, N - 1, X[0], X[N - 1]));
  };
  PoseGraph a, b;
  build(a); build(b);
  a.optimize(); b.optimize();
  for (std::uint64_t k = 0; k < 12; ++k) {
    const Eigen::Matrix4d Ma = a.node(k).matrix(), Mb = b.node(k).matrix();
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) EXPECT_DOUBLE_EQ(Ma(i, j), Mb(i, j));
  }
}

// Re-optimizing a converged graph is a near-no-op: anchors stay put, no drift.
TEST(PoseGraphStress, IdempotentReoptimize) {
  PoseGraph g;
  std::mt19937 rng(3);
  const SE3 XB = makeSE3(1, 0, 0, 0.3, {0, 0, 1});
  const SE3 XC = makeSE3(1, 1, 0, -0.2, {0, 1, 0});
  g.setNode(0, SE3(), true);
  g.setNode(1, perturb(rng, XB, 0.05), false);
  g.setNode(2, perturb(rng, XC, 0.05), false);
  g.addEdge(between(0, 1, SE3(), XB));
  g.addEdge(between(1, 2, XB, XC));
  g.addEdge(between(2, 0, XC, SE3()));
  g.optimize();
  const SE3 b1 = g.node(1), c1 = g.node(2);

  const auto r2 = g.optimize();        // re-run on the converged graph
  EXPECT_LE(r2.iterations, 2);
  expectSE3Near(g.node(1), b1, 1e-10);
  expectSE3Near(g.node(2), c1, 1e-10);
}

// A gauge-free floating component (nodes 2,3 connected only to each other, not to the
// fixed gauge): its absolute pose is unobservable. LM damping must keep it FINITE and
// still satisfy the internal relative constraint — never NaN/blow up (Hard Rule #4).
TEST(PoseGraphStress, UnderConstrainedComponentStaysFinite) {
  PoseGraph g;
  const SE3 XB = makeSE3(1, 0, 0, 0.1, {0, 0, 1});
  const SE3 X2 = makeSE3(5, 5, 5, 0.3, {0, 1, 0});
  const SE3 X3 = makeSE3(6, 5, 5, 0.2, {1, 0, 0});
  g.setNode(0, SE3(), true);
  g.setNode(1, makeSE3(0.9, 0.1, 0, 0.05, {0, 0, 1}), false);  // perturbed
  g.setNode(2, X2, false);
  g.setNode(3, X3, false);
  g.addEdge(between(0, 1, SE3(), XB));        // anchored component
  g.addEdge(between(2, 3, X2, X3));           // floating pair, mutual constraint only

  const auto r = g.optimize();
  EXPECT_TRUE(r.converged);
  EXPECT_TRUE(isFinite(g.node(1)) && isFinite(g.node(2)) && isFinite(g.node(3)));
  expectSE3Near(g.node(1), XB, 1e-4);                                   // observable part solved
  expectSE3Near(g.node(2).inverse() * g.node(3), X2.inverse() * X3, 1e-4);  // relative preserved
}

// A loop with large (~1.2 rad) rotations — exercises log/exp away from the small-angle
// regime + the convergence basin of the J_r⁻¹≈I approximation on a consistent graph.
TEST(PoseGraphStress, LargeRotationLoop) {
  const SE3 XA = SE3();
  const SE3 XB = makeSE3(1, 0, 0, 1.2, {0, 0, 1});
  const SE3 XC = makeSE3(0, 1, 0, 1.2, {0, 1, 0});
  PoseGraph g;
  std::mt19937 rng(5);
  g.setNode(0, XA, true);
  g.setNode(1, perturb(rng, XB, 0.05), false);
  g.setNode(2, perturb(rng, XC, 0.05), false);
  g.addEdge(between(0, 1, XA, XB));
  g.addEdge(between(1, 2, XB, XC));
  g.addEdge(between(2, 0, XC, XA));

  PoseGraphConfig cfg; cfg.max_iters = 80;
  const auto r = g.optimize(cfg);
  EXPECT_TRUE(r.converged);
  expectSE3Near(g.node(1), XB, 1e-4);
  expectSE3Near(g.node(2), XC, 1e-4);
}

// --- Supervisor at scale ------------------------------------------------------

namespace {

class InjectableRelocalizer : public slamko::Relocalizer {
 public:
  std::string name() const override { return "injectable"; }
  void addSubMap(const SubMap&) override {}
  RelocResult relocalize(const Features&) const override {
    if (queue_.empty()) return RelocResult{};
    RelocResult r = queue_.front(); queue_.pop_front(); return r;
  }
  void push(std::uint64_t sealed_id, const SE3& T, int inliers, double conf) {
    RelocResult r; r.found = true; r.submap_id = sealed_id; r.T_query_match = T;
    r.num_inliers = inliers; r.confidence = conf; queue_.push_back(r);
  }
  mutable std::deque<RelocResult> queue_;
};

RecoveryAction stepGap(NeverLostSupervisor& s, double gap, double& t) {
  HealthSignal h; h.odom_stale_gap_s = gap;
  EstimationFrame odom;  // identity T_WB
  t += 0.1;
  return s.step(h, odom, t);
}

SupervisorConfig pgCfg() { SupervisorConfig c; c.use_pose_graph = true; return c; }

}  // namespace

// Many seal→branch→weld→recover cycles (Atlas scale). Each cycle welds the fresh
// branch to sealed submap 0; the graph must grow by exactly one node + one edge per
// cycle and never crash.
TEST(SupervisorStress, ManySealBranchWeldCycles) {
  InjectableRelocalizer reloc;
  NeverLostSupervisor s(pgCfg(), &reloc);
  s.submitQueryFeatures(Features{});
  double t = 0;
  const SE3 T = makeSE3(0.5, 0.2, 0.0, 0.1, {0, 0, 1});
  const int cycles = 10;

  for (int c = 0; c < cycles; ++c) {
    for (int i = 0; i < 3; ++i) stepGap(s, 1.5, t);     // loss → seal + branch
    for (int i = 0; i < 3; ++i) reloc.push(/*sealed_id=*/0, T, 30, 1.0);
    for (int i = 0; i < 5; ++i) stepGap(s, 0.5, t);     // cluster → weld
    for (int i = 0; i < 4; ++i) stepGap(s, 0.0, t);     // healthy → recover to OK
    ASSERT_EQ(s.state(), SupervisorState::OK) << "cycle " << c;
  }

  EXPECT_EQ(s.archive().sealedCount(), static_cast<std::size_t>(cycles));
  EXPECT_EQ(s.poseGraph().nodeCount(), static_cast<std::size_t>(cycles + 1));  // 0..cycles
  EXPECT_EQ(s.poseGraph().edgeCount(), static_cast<std::size_t>(cycles));      // 0→k each cycle
  EXPECT_TRUE(isFinite(s.mapToOdom()));
  expectSE3Near(s.mapToOdom(), T, 1e-6);   // X0=I ⇒ active anchor = consensus = T
}

// weld_once_per_target bounds graph growth: 30 agreeing hits to ONE target in a
// single episode must produce exactly ONE weld / ONE edge. With the guard off, the
// same stream re-welds and appends many edges — proving the flag is what bounds it.
TEST(SupervisorStress, WeldOnceBoundsEdgeGrowth) {
  const SE3 T = makeSE3(1, 0, 0, 0.1, {0, 0, 1});

  auto runFlood = [&](bool guard) {
    SupervisorConfig c = pgCfg();
    c.weld_once_per_target = guard;
    InjectableRelocalizer reloc;
    NeverLostSupervisor s(c, &reloc);
    s.submitQueryFeatures(Features{});
    double t = 0;
    for (int i = 0; i < 3; ++i) stepGap(s, 1.5, t);   // seal + branch
    for (int i = 0; i < 30; ++i) reloc.push(0, T, 30, 1.0);
    int welds = 0;
    for (int i = 0; i < 30; ++i) welds += stepGap(s, 0.5, t).welded ? 1 : 0;
    return std::pair<int, std::size_t>(welds, s.poseGraph().edgeCount());
  };

  const auto on = runFlood(true);
  EXPECT_EQ(on.first, 1);                  // exactly one weld
  EXPECT_EQ(on.second, 1u);                // exactly one edge

  const auto off = runFlood(false);
  EXPECT_GT(off.first, 1);                 // legacy: re-welds every cluster cycle
  EXPECT_GT(off.second, 1u);               // …appending duplicate edges
}

// Health flapping around the thresholds must NOT seal: sealing needs lost_dwell_frames
// CONSECUTIVE over-threshold steps, so an alternating signal never triggers.
TEST(SupervisorStress, FlappingHealthDoesNotThrash) {
  NeverLostSupervisor s(SupervisorConfig{}, nullptr);
  double t = 0;
  for (int i = 0; i < 40; ++i) stepGap(s, (i % 2 == 0) ? 1.5 : 0.0, t);
  EXPECT_EQ(s.archive().sealedCount(), 0u);
  EXPECT_EQ(s.archive().activeId(), 0u);
  EXPECT_NE(s.state(), SupervisorState::Lost);
}
