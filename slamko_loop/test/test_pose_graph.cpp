// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Unit test for the thin global PoseGraph: a synthetic closed loop where the
// odometry chain has a constant per-edge yaw bias (so the dead-reckoned estimate
// drifts and the loop does NOT close), plus ONE loop-closure edge carrying the
// true relative pose. After optimization the drift must be redistributed: the
// loop closes and the trajectory error vs ground truth collapses.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "slamko_loop/pose_graph.hpp"
#include "slamko_core/se3.hpp"

using namespace slamko;

namespace {

// Ground-truth: N poses evenly placed on a circle of radius R, heading tangent,
// closing back on the start (so the last->first relative is a known constraint).
std::vector<SE3> circleGroundTruth(int N, double R) {
  std::vector<SE3> gt;
  gt.reserve(N);
  for (int i = 0; i < N; ++i) {
    const double a = 2.0 * M_PI * i / N;        // angle around circle
    Eigen::Vector3d t(R * std::cos(a), R * std::sin(a), 0.0);
    // heading = tangent (yaw = a + 90deg), rotation about world Z.
    const double yaw = a + M_PI / 2.0;
    SO3 Rz(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())));
    gt.emplace_back(Rz, t);
  }
  return gt;
}

double meanTransError(const PoseGraph& pg, const std::vector<SE3>& gt) {
  double s = 0;
  for (size_t i = 0; i < gt.size(); ++i)
    s += (pg.pose(i).translation() - gt[i].translation()).norm();
  return s / gt.size();
}

}  // namespace

TEST(PoseGraph, ClosesDriftedLoop) {
  const int N = 24;
  const double R = 5.0;
  const auto gt = circleGroundTruth(N, R);

  // Per-edge constant yaw bias => the integrated estimate spirals away from gt.
  const double yaw_bias = 0.02;  // rad per edge (~1.1 deg)
  const SO3 bias(Eigen::Quaterniond(Eigen::AngleAxisd(yaw_bias, Eigen::Vector3d::UnitZ())));

  PoseGraph pg;
  pg.addKeyframe(0, gt[0]);
  pg.setAnchor(0);

  // Dead-reckon: est[i+1] = est[i] * (true_rel_i * bias). Store odometry edges = the
  // biased relatives (the graph is self-consistent with the drift).
  SE3 est = gt[0];
  std::vector<SE3> drift(N);
  drift[0] = est;
  for (int i = 0; i < N - 1; ++i) {
    const SE3 true_rel = gt[i].inverse() * gt[i + 1];
    const SE3 meas_rel(true_rel.so3() * bias, true_rel.translation());  // biased odom
    est = est * meas_rel;
    drift[i + 1] = est;
    pg.addKeyframe(i + 1, est);
    pg.addOdometryEdge(i, i + 1, meas_rel, /*sigma_t=*/0.02, /*sigma_r=*/0.01);
  }

  // Drift sanity: open loop, the estimate has visibly walked away from gt.
  const double err_before = meanTransError(pg, gt);
  EXPECT_GT(err_before, 0.3) << "test setup should produce real drift";

  // ONE loop-closure edge: last -> first, the TRUE relative (high confidence weld).
  const SE3 loop_meas = gt[N - 1].inverse() * gt[0];
  pg.addLoopEdge(N - 1, 0, loop_meas, /*sigma_t=*/0.02, /*sigma_r=*/0.01);

  const auto res = pg.optimize();
  ASSERT_TRUE(res.converged);
  EXPECT_EQ(res.num_loops, 1);
  EXPECT_EQ(res.num_odom, N - 1);
  EXPECT_LT(res.final_cost, res.initial_cost);

  // The loop must now close: estimated last->first relative ≈ the measurement.
  const SE3 rel_after = pg.pose(N - 1).inverse() * pg.pose(0);
  const double loop_gap =
      (loop_meas.inverse() * rel_after).log().norm();  // twist norm (m + rad)
  // Weighted LS optimum: 23 odom edges balance 1 loop edge, so the loop residual
  // settles at a small non-zero equilibrium (~0.02), not exactly 0. The strong
  // signal is the trajectory-error collapse below; this just confirms closure.
  EXPECT_LT(loop_gap, 0.05) << "loop did not close (gap=" << loop_gap << ")";

  // And the trajectory error vs gt collapses (drift redistributed around the loop).
  const double err_after = meanTransError(pg, gt);
  EXPECT_LT(err_after, 0.25 * err_before)
      << "before=" << err_before << " after=" << err_after;
}

// #12 TRUNK mechanism (METHODOLOGY_01 step 1): a tracking-loss gap = ONE bad odometry
// edge (dead-reckoned junk OKVIS IMU-bridged). If that edge carries the DR-gate
// uncertainty (HIGH covariance), the optimizer YIELDS there and the loop straightens the
// rest of the trajectory; if it stays provider-stiff, the bad measurement smears error
// across every pose. Soft-on-the-gap must beat stiff-on-the-gap vs ground truth. This is
// the brutal-on-suave "warp" fix, isolated to its pose-graph mechanism (no GPU/bags).
TEST(PoseGraph, DrGateSoftEdgeAbsorbsLossGap) {
  const int N = 24;
  const double R = 5.0;
  const auto gt = circleGroundTruth(N, R);
  const int gap = N / 2;  // the loss-gap edge is (gap-1)->(gap)

  auto run = [&](bool soft_gap) -> double {
    PoseGraph pg;
    pg.addKeyframe(0, gt[0]);
    pg.setAnchor(0);
    SE3 est = gt[0];
    for (int i = 0; i < N - 1; ++i) {
      const SE3 true_rel = gt[i].inverse() * gt[i + 1];
      SE3 meas_rel = true_rel;
      if (i == gap - 1) {
        // Corrupt the gap edge with spurious yaw + translation = the dead-reckoned
        // across-gap junk a tracking loss leaves behind.
        const SO3 bad(Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ())));
        meas_rel = SE3(true_rel.so3() * bad,
                       true_rel.translation() + Eigen::Vector3d(0.5, 0.3, 0.0));
      }
      est = est * meas_rel;
      pg.addKeyframe(i + 1, est);
      if (i == gap - 1 && soft_gap)
        pg.addOdometryEdge(i, i + 1, meas_rel, /*sigma_t=*/0.8, /*sigma_r=*/0.4);  // DR-gate cov
      else
        pg.addOdometryEdge(i, i + 1, meas_rel, /*sigma_t=*/0.02, /*sigma_r=*/0.01);  // stiff
    }
    const SE3 loop_meas = gt[N - 1].inverse() * gt[0];
    pg.addLoopEdge(N - 1, 0, loop_meas, /*sigma_t=*/0.02, /*sigma_r=*/0.01);
    pg.optimize();
    return meanTransError(pg, gt);
  };

  const double err_stiff = run(false);
  const double err_soft = run(true);
  EXPECT_LT(err_soft, err_stiff)
      << "DR-gate soft gap edge must beat provider-stiff: soft=" << err_soft
      << " stiff=" << err_stiff;
  // And the absolute quality with the soft gap should be good (loop straightened it).
  EXPECT_LT(err_soft, 0.5) << "soft-gap trajectory should be near gt (err=" << err_soft << ")";
}

// Atlas DISJOINT-ISLANDS model (METHODOLOGY_01 etapa 1a): a tracking loss BREAKS the
// chain into a new map that floats freely (no bridging edge) until a feature match welds
// it back. Per-component gauge must (a) keep each island well-posed (no singular solve)
// and (b) let a later weld FUSE one island onto the other. This is the "honest dangling"
// behaviour the user asked for — the brutal bag should render as fragments, not one warp.
TEST(PoseGraph, DisjointIslandsGaugedThenWeldFuses) {
  PoseGraph pg;
  // Island A: nodes 0,1,2 — straight chain along +x from the origin.
  pg.addKeyframe(0, SE3());
  pg.addKeyframe(1, SE3(SO3(), Eigen::Vector3d(1, 0, 0)));
  pg.addKeyframe(2, SE3(SO3(), Eigen::Vector3d(2, 0, 0)));
  pg.addOdometryEdge(0, 1, SE3(SO3(), Eigen::Vector3d(1, 0, 0)));
  pg.addOdometryEdge(1, 2, SE3(SO3(), Eigen::Vector3d(1, 0, 0)));
  // Island B: nodes 10,11,12 placed FAR away (its OKVIS-drifted estimate) — NO edge to A.
  pg.addKeyframe(10, SE3(SO3(), Eigen::Vector3d(50, 50, 0)));
  pg.addKeyframe(11, SE3(SO3(), Eigen::Vector3d(51, 50, 0)));
  pg.addKeyframe(12, SE3(SO3(), Eigen::Vector3d(52, 50, 0)));
  pg.addOdometryEdge(10, 11, SE3(SO3(), Eigen::Vector3d(1, 0, 0)));
  pg.addOdometryEdge(11, 12, SE3(SO3(), Eigen::Vector3d(1, 0, 0)));

  const auto res1 = pg.optimize();
  ASSERT_TRUE(res1.converged);
  // Each island stays finite at its OWN gauge (no singular collapse, no fake-coherence).
  EXPECT_NEAR(pg.pose(2).translation().x(), 2.0, 1e-3);
  EXPECT_NEAR(pg.pose(12).translation().x(), 52.0, 1e-3);
  EXPECT_NEAR(pg.pose(12).translation().y(), 50.0, 1e-3);

  // A feature MATCH welds B's node 10 onto A's node 2 (co-located place): meas 2->10 = +x.
  // The weld merges the components -> ONE gauge (node 0) -> B bends onto A (its far gauge
  // releases). This is the geometric fusion-for-free.
  pg.addLoopEdge(2, 10, SE3(SO3(), Eigen::Vector3d(1, 0, 0)), 0.02, 0.01);
  const auto res2 = pg.optimize();
  ASSERT_TRUE(res2.converged);
  EXPECT_NEAR(pg.pose(2).translation().x(), 2.0, 0.3);  // A holds (merged-component gauge)
  EXPECT_LT((pg.pose(10).translation() - Eigen::Vector3d(3, 0, 0)).norm(), 0.4)
      << "welded island did not fuse onto A: " << pg.pose(10).translation().transpose();
}

// De-risk the offline-driver WELD MATH before trusting it on real submaps: given a
// relocalization result T_query_match (query body pose in the matched submap's local
// frame) and the matched KF's local pose matchedKF.T_WB, the loop edge from query→matched
// is T_from_to = T_query_match.inverse() * matchedKF.T_WB. This must be ANCHOR-INVARIANT
// (anchors cancel in a relative measurement) and recover the true relative pose.
TEST(PoseGraph, WeldMathRecoversRelative) {
  // Ground truth: query and matched KF body poses in some submap-local frame.
  const SE3 query_local(SO3(Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()))),
                        Eigen::Vector3d(1.0, 2.0, 0.5));
  const SE3 matched_local(SO3(Eigen::Quaterniond(Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()))),
                          Eigen::Vector3d(3.0, -1.0, 0.2));
  // A relocalizer that worked perfectly returns T_query_match = query pose in matched-local.
  const SE3 T_query_match = query_local;
  const SE3 matchedKF_TWB = matched_local;
  // Weld edge (driver formula):
  const SE3 T_from_to = T_query_match.inverse() * matchedKF_TWB;
  // True relative query→matched:
  const SE3 truth = query_local.inverse() * matched_local;
  EXPECT_LT((T_from_to.inverse() * truth).log().norm(), 1e-9) << "weld formula wrong";

  // Anchor-invariance: an arbitrary anchor applied to BOTH must not change the edge.
  const SE3 anchor(SO3(Eigen::Quaterniond(Eigen::AngleAxisd(1.1, Eigen::Vector3d(1,1,1).normalized()))),
                   Eigen::Vector3d(7, -3, 4));
  const SE3 T_query_match_a = anchor * query_local;   // same query in a re-anchored local frame
  const SE3 matchedKF_TWB_a = anchor * matched_local;
  const SE3 T_from_to_a = T_query_match_a.inverse() * matchedKF_TWB_a;
  EXPECT_LT((T_from_to_a.inverse() * T_from_to).log().norm(), 1e-9) << "weld not anchor-invariant";
}

TEST(PoseGraph, NoOpWithoutEdges) {
  PoseGraph pg;
  pg.addKeyframe(0, SE3());
  pg.addKeyframe(1, SE3());
  const auto res = pg.optimize();
  EXPECT_FALSE(res.converged);  // no edges => nothing to solve
  EXPECT_EQ(res.num_nodes, 2);
}

// ---- Cross-session merge: BETWEEN edges to FIXED prior nodes (the rotation fix) ----
// docs/RESEARCH_XSESSION_ROTATION_01.md. A 2nd session of the same place, expressed in its
// own frame (ROTATED 20° + per-edge internal drift), is merged onto a FIXED prior by
// cross-session feature matches modelled as relative BETWEEN edges (meas = the relative pose
// of the matched keyframe pair) — NOT the buggy unary prior toward a frozen global frame.
// The prior must NOT deform → setFixed on all its nodes (multi-fixed gauge). Two facts:
//   (1) ≥2 spatially-separated between-edges DISTRIBUTE the session's internal drift (both
//       ends pinned) → low error everywhere; ONE match pins one end and the drift piles up
//       at the far end (the doubled/offset tail the user saw).
//   (2) the prior map stays rigid throughout.
namespace {
SE3 yawPose(double x, double y, double yaw) {
  return SE3(SO3::exp(Eigen::Vector3d(0, 0, yaw)), Eigen::Vector3d(x, y, 0));
}
// Prior (fixed straight line) + a rotated, internally-drifted query session. `n_match`
// cross-session BETWEEN edges (1 = far end only at start; 2 = both ends). Returns far-end
// query position error vs its prior twin.
double mergeFarEndError(int N, double drift_deg, int n_match) {
  PoseGraph pg;
  std::vector<SE3> P(N);
  for (int i = 0; i < N; ++i) {
    P[i] = yawPose(i * 1.0, 0, 0);
    pg.addKeyframe(1000 + i, P[i]);
    pg.setFixed(1000 + i);
  }
  const SO3 drift = SO3::exp(Eigen::Vector3d(0, 0, drift_deg * M_PI / 180.0));
  SE3 est = yawPose(0, 0, 20.0 * M_PI / 180.0) * P[0];  // session frame rotated 20°
  pg.addKeyframe(0, est);
  for (int i = 0; i + 1 < N; ++i) {
    const SE3 rel = P[i].inverse() * P[i + 1];
    const SE3 biased(rel.so3() * drift, rel.translation());
    est = est * biased;
    pg.addKeyframe(i + 1, est);
    pg.addOdometryEdge(i, i + 1, biased, 0.05, 0.02);
  }
  pg.addLoopEdge(1000 + 0, 0, SE3(), 0.02, 0.01);              // match at the start
  if (n_match >= 2)
    pg.addLoopEdge(1000 + (N - 1), N - 1, SE3(), 0.02, 0.01);  // + match at the far end
  pg.optimize();
  // The prior map must stay RIGID no matter what.
  for (int i = 0; i < N; ++i)
    if ((pg.pose(1000 + i).translation() - P[i].translation()).norm() > 1e-9)
      ADD_FAILURE() << "prior node " << i << " moved (not rigid)";
  return (pg.pose(N - 1).translation() - P[N - 1].translation()).norm();
}
}  // namespace

TEST(PoseGraph, CrossSessionTwoBetweenEdgesDistributeDrift) {
  // Both ends pinned → the session is bent onto the rigid prior, drift distributed.
  EXPECT_LT(mergeFarEndError(8, /*drift_deg=*/3.0, /*n_match=*/2), 0.15);
}

TEST(PoseGraph, CrossSessionOneMatchPilesDriftAtFarEnd) {
  const int N = 8;
  const double far_1 = mergeFarEndError(N, /*drift_deg=*/3.0, /*n_match=*/1);
  const double far_2 = mergeFarEndError(N, /*drift_deg=*/3.0, /*n_match=*/2);
  // One match pins the start but the internal drift accumulates to the far end;
  // a 2nd separated match distributes it → the far-end error collapses.
  EXPECT_GT(far_1, 0.3) << "one match should leave a far-end error (drift tail)";
  EXPECT_LT(far_2, 0.4 * far_1) << ">=2 separated matches must distribute the drift";
}
