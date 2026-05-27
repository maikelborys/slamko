// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// NeverLostSupervisor + AnchorGate + SubMapArchive tests — fully synthetic, no
// ROS / no rosbag2 / no solver. Hand-built HealthSignal / RelocResult / SubMap
// drive the policy; exact SE3 assertions validate the seal→branch→weld math and
// the map→odom convention. This is the P2 acceptance gate.

#include <deque>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "slamko_core/features.hpp"
#include "slamko_core/relocalizer.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"

#include "slamko_loop/never_lost_supervisor.hpp"
#include "slamko_loop/submap_archive.hpp"

using slamko::EstimationFrame;
using slamko::Features;
using slamko::HealthSignal;
using slamko::HealthState;
using slamko::NeverLostSupervisor;
using slamko::RecoveryAction;
using slamko::RelocResult;
using slamko::SE3;
using slamko::SubMap;
using slamko::SubMapArchive;
using slamko::SupervisorConfig;
using slamko::SupervisorState;

namespace {

SE3 makeSE3(double tx, double ty, double tz, double angle, Eigen::Vector3d axis) {
  const Eigen::Matrix3d R =
      Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  return SE3(R, Eigen::Vector3d(tx, ty, tz));
}

void expectSE3Near(const SE3& a, const SE3& b, double tol) {
  const Eigen::Matrix4d A = a.matrix(), B = b.matrix();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) EXPECT_NEAR(A(i, j), B(i, j), tol) << "(" << i << "," << j << ")";
}

SupervisorConfig cfg() {
  SupervisorConfig c;  // defaults; explicit for clarity in tests
  c.recently_lost_gap_s = 0.25;
  c.lost_gap_s = 1.0;
  c.lost_dwell_frames = 3;
  c.recover_dwell_frames = 3;
  c.weld_cluster_radius_m = 0.10;
  c.weld_cluster_radius_rad = 0.05;
  c.weld_min_matches = 3;
  c.weld_min_inliers = 15;
  c.weld_min_confidence = 0.0;
  return c;
}

// Test double for slamko_core::Relocalizer: returns queued results, one per call.
class InjectableRelocalizer : public slamko::Relocalizer {
 public:
  std::string name() const override { return "injectable"; }
  void addSubMap(const SubMap&) override {}
  RelocResult relocalize(const Features&) const override {
    if (queue_.empty()) return RelocResult{};  // found = false
    RelocResult r = queue_.front();
    queue_.pop_front();
    return r;
  }
  void push(std::uint64_t sealed_id, const SE3& T, int inliers, double conf) {
    RelocResult r;
    r.found = true;
    r.submap_id = sealed_id;
    r.T_query_match = T;
    r.num_inliers = inliers;
    r.confidence = conf;
    queue_.push_back(r);
  }
  mutable std::deque<RelocResult> queue_;
};

RecoveryAction stepGap(NeverLostSupervisor& s, double gap, double& t) {
  HealthSignal h;
  h.odom_stale_gap_s = gap;
  EstimationFrame odom;  // identity T_WB
  t += 0.1;
  return s.step(h, odom, t);
}

RecoveryAction stepGapOdom(NeverLostSupervisor& s, double gap, const SE3& T_WB,
                           double& t) {
  HealthSignal h;
  h.odom_stale_gap_s = gap;
  EstimationFrame odom;
  odom.T_WB = T_WB;
  t += 0.1;
  return s.step(h, odom, t);
}

// Drive a full loss → seal+branch. Returns once state == Relocalizing.
void triggerLoss(NeverLostSupervisor& s, double& t) {
  for (int i = 0; i < cfg().lost_dwell_frames; ++i) stepGap(s, 1.5, t);
}

}  // namespace

// --- State machine -----------------------------------------------------------

TEST(Supervisor, StaleGapSealsAndBranches) {
  NeverLostSupervisor s(cfg(), nullptr);
  double t = 0;
  EXPECT_EQ(s.state(), SupervisorState::OK);
  EXPECT_FALSE(stepGap(s, 1.5, t).sealed);  // 1
  EXPECT_FALSE(stepGap(s, 1.5, t).sealed);  // 2
  const RecoveryAction a = stepGap(s, 1.5, t);  // 3 → seal+branch
  EXPECT_TRUE(a.sealed);
  EXPECT_TRUE(a.branched);
  EXPECT_EQ(a.sealed_id, 0u);
  EXPECT_EQ(a.branched_id, 1u);
  EXPECT_EQ(s.state(), SupervisorState::Relocalizing);
  EXPECT_EQ(s.healthState(), HealthState::Lost);
  EXPECT_EQ(s.archive().sealedCount(), 1u);
  EXPECT_EQ(s.archive().activeId(), 1u);
}

TEST(Supervisor, OneFrameBlipDoesNotSeal) {
  NeverLostSupervisor s(cfg(), nullptr);
  double t = 0;
  stepGap(s, 1.5, t);                         // one over-threshold blip
  const RecoveryAction a = stepGap(s, 0.0, t);  // recovers
  EXPECT_FALSE(a.sealed);
  EXPECT_EQ(s.state(), SupervisorState::OK);
  EXPECT_EQ(s.archive().sealedCount(), 0u);
}

TEST(Supervisor, RecentlyLostDoesNotSeal) {
  NeverLostSupervisor s(cfg(), nullptr);
  double t = 0;
  for (int i = 0; i < 5; ++i) EXPECT_FALSE(stepGap(s, 0.5, t).sealed);  // in (0.25, 1.0]
  EXPECT_EQ(s.state(), SupervisorState::RecentlyLost);
  EXPECT_EQ(s.healthState(), HealthState::Marginal);
  EXPECT_EQ(s.archive().sealedCount(), 0u);
}

// --- The weld + the map→odom convention --------------------------------------

TEST(Supervisor, WeldOnConsensusAndMapToOdom) {
  InjectableRelocalizer reloc;
  NeverLostSupervisor s(cfg(), &reloc);
  double t = 0;
  expectSE3Near(s.mapToOdom(), SE3(), 1e-12);  // identity at start
  triggerLoss(s, t);
  ASSERT_EQ(s.state(), SupervisorState::Relocalizing);
  ASSERT_EQ(s.archive().sealedCount(), 1u);
  expectSE3Near(s.mapToOdom(), SE3(), 1e-12);  // fresh branch → identity, pre-weld

  s.submitQueryFeatures(Features{});
  const SE3 T = makeSE3(1, 2, 3, 0.3, {0, 0, 1});
  for (int i = 0; i < 3; ++i) reloc.push(/*sealed_id=*/0, T, /*inliers=*/30, /*conf=*/1.0);

  bool welded = false;
  for (int i = 0; i < 3; ++i) {  // gap 0.5 keeps it in Relocalizing
    const RecoveryAction a = stepGap(s, 0.5, t);
    if (a.welded) {
      welded = true;
      EXPECT_EQ(a.welded_to_id, 0u);
      expectSE3Near(a.applied_T_active_sealed, T, 1e-9);
    }
  }
  EXPECT_TRUE(welded);
  // map→odom = active.anchor = S.anchor(I) * T == T.
  expectSE3Near(s.mapToOdom(), T, 1e-9);
  expectSE3Near(s.archive().active().anchor, T, 1e-9);
}

TEST(Supervisor, ScatteredMatchesRejected) {
  InjectableRelocalizer reloc;
  NeverLostSupervisor s(cfg(), &reloc);
  double t = 0;
  triggerLoss(s, t);
  s.submitQueryFeatures(Features{});
  // Four mutually-distant hits (perceptual aliasing) — no cluster reaches 3.
  reloc.push(0, makeSE3(0, 0, 0, 0, {0, 0, 1}), 30, 1.0);
  reloc.push(0, makeSE3(5, 0, 0, 0, {0, 0, 1}), 30, 1.0);
  reloc.push(0, makeSE3(0, 5, 0, 0, {0, 0, 1}), 30, 1.0);
  reloc.push(0, makeSE3(0, 0, 5, 0, {0, 0, 1}), 30, 1.0);
  bool welded = false;
  for (int i = 0; i < 4; ++i) welded |= stepGap(s, 0.5, t).welded;
  EXPECT_FALSE(welded);
  expectSE3Near(s.mapToOdom(), SE3(), 1e-12);  // never welded → still identity
}

TEST(Supervisor, MixedOutliersWeldOnAgreeingConsensus) {
  InjectableRelocalizer reloc;
  NeverLostSupervisor s(cfg(), &reloc);
  double t = 0;
  triggerLoss(s, t);
  s.submitQueryFeatures(Features{});
  const SE3 T = makeSE3(1, 2, 3, 0.3, {0, 0, 1});
  // 2 scattered outliers interleaved with 3 agreeing hits.
  reloc.push(0, makeSE3(9, 0, 0, 0, {0, 0, 1}), 30, 1.0);  // outlier
  reloc.push(0, T, 30, 1.0);                                // agree
  reloc.push(0, makeSE3(0, 9, 0, 0, {0, 0, 1}), 30, 1.0);  // outlier
  reloc.push(0, T, 30, 1.0);                                // agree
  reloc.push(0, T, 30, 1.0);                                // agree → cluster of 3
  bool welded = false;
  for (int i = 0; i < 5; ++i) welded |= stepGap(s, 0.5, t).welded;
  EXPECT_TRUE(welded);
  expectSE3Near(s.mapToOdom(), T, 1e-9);  // consensus = the agreeing cluster, outliers ignored
}

TEST(Supervisor, LowInlierMatchesRejected) {
  InjectableRelocalizer reloc;
  NeverLostSupervisor s(cfg(), &reloc);
  double t = 0;
  triggerLoss(s, t);
  s.submitQueryFeatures(Features{});
  const SE3 T = makeSE3(1, 2, 3, 0.3, {0, 0, 1});
  for (int i = 0; i < 5; ++i) reloc.push(0, T, /*inliers=*/5, 1.0);  // below floor (15)
  bool welded = false;
  for (int i = 0; i < 5; ++i) welded |= stepGap(s, 0.5, t).welded;
  EXPECT_FALSE(welded);
}

TEST(Supervisor, WeldComposesNonIdentitySealedAnchor) {
  InjectableRelocalizer reloc;
  NeverLostSupervisor s(cfg(), &reloc);
  s.submitQueryFeatures(Features{});
  double t = 0;
  const SE3 T1 = makeSE3(1, 0, 0, 0.1, {0, 0, 1});
  const SE3 T2 = makeSE3(0, 1, 0, 0.1, {1, 0, 0});

  // Cycle 1: loss → seal id0 (anchor I), branch id1; weld to id0 with T1
  //          → active(id1).anchor = I * T1 = T1.
  triggerLoss(s, t);
  for (int i = 0; i < 3; ++i) reloc.push(0, T1, 30, 1.0);
  for (int i = 0; i < 3; ++i) stepGap(s, 0.5, t);
  expectSE3Near(s.archive().active().anchor, T1, 1e-9);

  // Recover to OK (healthy odom for recover_dwell).
  for (int i = 0; i < 3; ++i) stepGap(s, 0.0, t);
  ASSERT_EQ(s.state(), SupervisorState::OK);

  // Cycle 2: loss → seal id1 (anchor T1!), branch id2; weld to id1 with T2
  //          → active(id2).anchor = T1 * T2 (non-identity sealed anchor).
  triggerLoss(s, t);
  ASSERT_EQ(s.archive().sealedCount(), 2u);
  for (int i = 0; i < 3; ++i) reloc.push(/*sealed_id=*/1, T2, 30, 1.0);
  bool welded = false;
  for (int i = 0; i < 3; ++i) {
    const RecoveryAction a = stepGap(s, 0.5, t);
    if (a.welded) { welded = true; EXPECT_EQ(a.welded_to_id, 1u); }
  }
  EXPECT_TRUE(welded);
  expectSE3Near(s.archive().active().anchor, T1 * T2, 1e-9);
}

// The relocalizer returns an ABSOLUTE query-body pose in sealed-local; the
// supervisor composes it with the live odom (T_active_sealed = T_query_match ·
// T_WB⁻¹) before gating. Verify that composition with a non-identity odom.
TEST(Supervisor, WeldComposesWithLiveOdom) {
  InjectableRelocalizer reloc;
  NeverLostSupervisor s(cfg(), &reloc);
  double t = 0;
  triggerLoss(s, t);  // seal id0 (anchor I), branch id1
  s.submitQueryFeatures(Features{});
  const SE3 Tqm   = makeSE3(2, 0, 1, 0.2, {0, 0, 1});  // absolute body-in-sealed
  const SE3 Todom = makeSE3(0, 1, 0, 0.1, {0, 1, 0});  // live odom T_WB
  for (int i = 0; i < 3; ++i) reloc.push(0, Tqm, 30, 1.0);
  bool welded = false;
  for (int i = 0; i < 3; ++i) welded |= stepGapOdom(s, 0.5, Todom, t).welded;
  EXPECT_TRUE(welded);
  // active.anchor = S.anchor(I) · (Tqm · Todom⁻¹).
  expectSE3Near(s.mapToOdom(), Tqm * Todom.inverse(), 1e-9);
}

// While Relocalizing, healthy odom ALONE must not recover to OK — the supervisor
// keeps attempting the weld (the re-acquired vision on revisit is what re-anchors
// the branch). It recovers only once WELDED (then a dwell), or after give-up.
TEST(Supervisor, RelocalizingStaysUntilWelded) {
  InjectableRelocalizer reloc;
  NeverLostSupervisor s(cfg(), &reloc);
  s.submitQueryFeatures(Features{});
  double t = 0;
  triggerLoss(s, t);
  ASSERT_EQ(s.state(), SupervisorState::Relocalizing);
  // Healthy odom but NO relocalization hit → stays Relocalizing (not OK).
  for (int i = 0; i < 10; ++i) stepGap(s, 0.0, t);
  EXPECT_EQ(s.state(), SupervisorState::Relocalizing);
  // A clustered weld arrives → re-anchor, then recover to OK after the dwell.
  const SE3 T = makeSE3(1, 0, 0, 0.1, {0, 0, 1});
  for (int i = 0; i < 3; ++i) reloc.push(0, T, 30, 1.0);
  bool welded = false;
  for (int i = 0; i < 6; ++i) welded |= stepGap(s, 0.0, t).welded;
  EXPECT_TRUE(welded);
  EXPECT_EQ(s.state(), SupervisorState::OK);
}

// --- SubMapArchive primitives -------------------------------------------------

TEST(SubMapArchive, SealBranchFindAnchor) {
  SubMapArchive a;
  EXPECT_EQ(a.activeId(), 0u);
  EXPECT_EQ(a.sealedCount(), 0u);

  // Content is ingested but id + anchor stay archive-owned.
  SubMap content;
  content.id = 999;                    // ignored
  content.anchor = makeSE3(7, 7, 7, 0, {0, 0, 1});  // ignored
  content.landmarks.resize(5);
  a.setActiveContent(content);
  EXPECT_EQ(a.activeId(), 0u);
  EXPECT_EQ(a.active().landmarks.size(), 5u);
  expectSE3Near(a.active().anchor, SE3(), 1e-12);

  const std::uint64_t sid = a.seal();
  EXPECT_EQ(sid, 0u);
  EXPECT_EQ(a.sealedCount(), 1u);
  const std::uint64_t bid = a.branch();
  EXPECT_EQ(bid, 1u);
  EXPECT_EQ(a.activeId(), 1u);

  // Anchor can be set on a sealed submap (the only legal post-seal mutation).
  const SE3 A = makeSE3(1, 2, 3, 0.2, {0, 1, 0});
  a.setAnchor(0, A);
  ASSERT_NE(a.find(0), nullptr);
  expectSE3Near(a.find(0)->anchor, A, 1e-12);
  EXPECT_EQ(a.find(42), nullptr);
}
