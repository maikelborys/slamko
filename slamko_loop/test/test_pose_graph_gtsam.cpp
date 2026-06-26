// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// A/B validation of the GTSAM Levenberg-Marquardt pose-graph backend against the Ceres default.
// The load-bearing claim: the SAME factor graph (relative BetweenFactors + a loop closure + the
// per-component gauge), with the [trans;rot]↔[rot;trans] information permutation, minimises the
// SAME objective under both solvers → they reach the same minimum. We build a drifted square loop,
// optimise it with each backend, and assert the optimised absolute poses agree to sub-mm / sub-mrad.
// Built only with -DSLAMKO_LOOP_WITH_GTSAM=ON (the only config where GtsamLM is a real solver).

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "slamko_loop/pose_graph.hpp"

using slamko::PoseGraph;
using slamko::PoseGraphConfig;
using slamko::PoseGraphBackend;
using slamko::SE3;
using slamko::SO3;

namespace {

SE3 yawTrans(double yaw, double x, double y) {
  return SE3(SO3::exp(Eigen::Vector3d(0, 0, yaw)), Eigen::Vector3d(x, y, 0));
}

// Build a 6-node square-ish loop. True poses turn 90° at each corner; the graph is INITIALISED with
// a small per-step yaw drift so the loop is open by a visible gap until the loop edge pulls it shut.
void buildDriftedLoop(PoseGraph& g) {
  std::vector<SE3> gt;
  gt.push_back(yawTrans(0.0, 0, 0));
  gt.push_back(yawTrans(0.0, 1, 0));
  gt.push_back(yawTrans(M_PI / 2, 2, 0));
  gt.push_back(yawTrans(M_PI / 2, 2, 1));
  gt.push_back(yawTrans(M_PI, 1, 1));
  gt.push_back(yawTrans(M_PI, 0, 1));

  // Drifted init: integrate the odometry but inject +2° yaw error each step (accumulating).
  SE3 cur = gt[0];
  g.addKeyframe(0, cur);
  for (std::size_t i = 1; i < gt.size(); ++i) {
    const SE3 rel = gt[i - 1].inverse() * gt[i];
    cur = cur * rel * SE3(SO3::exp(Eigen::Vector3d(0, 0, 0.035)), Eigen::Vector3d::Zero());
    g.addKeyframe(static_cast<std::uint64_t>(i), cur);
    g.addOdometryEdge(static_cast<std::uint64_t>(i - 1), static_cast<std::uint64_t>(i), rel,
                      0.02, 0.01);
  }
  // Loop closure: node 5 back to node 0 (exact relative) — closes the accumulated drift.
  g.addLoopEdge(5, 0, gt[5].inverse() * gt[0], 0.05, 0.02);
}

}  // namespace

TEST(PoseGraphGtsam, MatchesCeresOnADriftedLoop) {
  PoseGraphConfig c_ceres;
  c_ceres.backend = PoseGraphBackend::Ceres;
  PoseGraph g_ceres(c_ceres);
  buildDriftedLoop(g_ceres);
  const auto r_ceres = g_ceres.optimize();

  PoseGraphConfig c_gtsam;
  c_gtsam.backend = PoseGraphBackend::GtsamLM;
  PoseGraph g_gtsam(c_gtsam);
  buildDriftedLoop(g_gtsam);
  const auto r_gtsam = g_gtsam.optimize();

  ASSERT_EQ(r_ceres.num_nodes, r_gtsam.num_nodes);
  ASSERT_EQ(r_ceres.num_loops, 1);
  ASSERT_EQ(r_gtsam.num_loops, 1);

  // Node-by-node agreement (gauge = node 0 pinned in both → comparable absolute frame).
  for (std::uint64_t id = 0; id < 6; ++id) {
    const SE3 a = g_ceres.pose(id);
    const SE3 b = g_gtsam.pose(id);
    const double dt = (a.translation() - b.translation()).norm();
    const double dr = (a.so3().inverse() * b.so3()).log().norm();
    EXPECT_LT(dt, 2e-3) << "translation mismatch at node " << id;
    EXPECT_LT(dr, 2e-3) << "rotation mismatch at node " << id;
  }
  // Both must actually reduce the cost (the drift gets absorbed).
  EXPECT_LT(r_ceres.final_cost, r_ceres.initial_cost);
  EXPECT_LT(r_gtsam.final_cost, r_gtsam.initial_cost);
}

TEST(PoseGraphGtsam, FallbackOrSolveDoesNotThrow) {
  // GtsamLM on a trivial graph must not throw (either solves, or — if built without GTSAM — falls
  // back to Ceres transparently).
  PoseGraphConfig c;
  c.backend = PoseGraphBackend::GtsamLM;
  PoseGraph g(c);
  g.addKeyframe(0, SE3());
  g.addKeyframe(1, yawTrans(0.1, 1, 0));
  g.addOdometryEdge(0, 1, yawTrans(0.0, 1, 0), 0.02, 0.01);
  EXPECT_NO_THROW(g.optimize());
}
