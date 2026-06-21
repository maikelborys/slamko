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
