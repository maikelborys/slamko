// SPDX-License-Identifier: Apache-2.0
// LoopConsensusGate — the negative-case battery. Every test models a REAL
// failure mode observed (or designed against) on the casa/magistrale runs.

#include <gtest/gtest.h>

#include "slamko_core/loop_consensus.hpp"

namespace {

using slamko::LoopCandidate;
using slamko::LoopConsensusConfig;
using slamko::LoopConsensusGate;
using slamko::SE3;
using slamko::SO3;

SE3 trans(double x, double y, double z) {
  return SE3(SO3(), Eigen::Vector3d(x, y, z));
}

// A robot walking +x at 0.1 m per candidate; meas tracks it exactly.
LoopCandidate walking(std::uint64_t target, int k, double t0 = 100.0) {
  LoopCandidate c;
  c.target_id = target;
  c.q_id = 100 + k;
  c.t = t0 + 0.2 * k;
  c.T_OB = trans(0.1 * k, 0.0, 0.0);
  c.meas = trans(0.5 + 0.1 * k, 0.2, 0.0);  // same motion, offset target frame
  return c;
}

TEST(LoopConsensus, TrueStreakAcceptsOnKth) {
  LoopConsensusGate g;
  EXPECT_FALSE(g.feed(walking(7, 0)));
  EXPECT_FALSE(g.feed(walking(7, 1)));
  EXPECT_TRUE(g.feed(walking(7, 2)));  // 3rd consistent -> accept
  EXPECT_EQ(g.streak(7), 3);
}

// THE CASA1_Suave lesson: 6 m of absolute drift between graph and measurement
// must NOT matter — only pairwise consistency does.
TEST(LoopConsensus, DriftMagnitudeAgnostic) {
  LoopConsensusGate g;
  for (int k = 0; k < 3; ++k) {
    auto c = walking(7, k);
    c.meas = trans(6.0 + 0.1 * k, -2.0, 1.5);  // huge offset, consistent motion
    if (k < 2) EXPECT_FALSE(g.feed(c));
    else EXPECT_TRUE(g.feed(c));
  }
}

// Aliasing: PnP locks onto different self-similar structure each frame — the
// measurement jumps around instead of tracking the odometry.
TEST(LoopConsensus, AliasingJitterNeverAccepts) {
  LoopConsensusGate g;
  for (int k = 0; k < 20; ++k) {
    auto c = walking(7, k);
    c.meas = trans((k % 3) * 1.0, (k % 2) * 0.8, 0.0);  // jitter >> tol
    EXPECT_FALSE(g.feed(c)) << "aliased candidate " << k << " accepted";
  }
  EXPECT_EQ(g.streak(7), 1);  // never builds past the restart
}

// One bad candidate mid-streak restarts the count — 2 good + 1 bad + 2 good
// must NOT accept (the bad one becomes the new base).
TEST(LoopConsensus, InconsistencyResetsStreak) {
  LoopConsensusGate g;
  EXPECT_FALSE(g.feed(walking(7, 0)));
  EXPECT_FALSE(g.feed(walking(7, 1)));
  auto bad = walking(7, 2);
  bad.meas = trans(3.0, 3.0, 0.0);
  EXPECT_FALSE(g.feed(bad));
  EXPECT_EQ(g.streak(7), 1);
  // Two more consistent with the WALKING track: the first disagrees with the
  // bad base (restart to 1), the next two build 2,3 -> accept on the 3rd.
  EXPECT_FALSE(g.feed(walking(7, 3)));
  EXPECT_FALSE(g.feed(walking(7, 4)));
  EXPECT_TRUE(g.feed(walking(7, 5)));
}

// Rotation-only inconsistency must also break the streak (a yaw-flipped PnP
// solution can have a near-identical translation).
TEST(LoopConsensus, RotationInconsistencyRejected) {
  LoopConsensusGate g;
  EXPECT_FALSE(g.feed(walking(7, 0)));
  auto c = walking(7, 1);
  c.meas = SE3(SO3::exp(Eigen::Vector3d(0, 0, 0.5)), c.meas.translation());
  EXPECT_FALSE(g.feed(c));
  EXPECT_EQ(g.streak(7), 1);
}

// Cooldown: a second accept right after the first is suppressed until
// cooldown_s passes (prevents the 14-optimize()-calls-in-2-s storm).
TEST(LoopConsensus, CooldownSuppressesBurst) {
  LoopConsensusConfig cfg;
  cfg.cooldown_s = 2.0;
  LoopConsensusGate g(cfg);
  int accepts = 0;
  for (int k = 0; k < 14; ++k)
    if (g.feed(walking(7, k))) ++accepts;
  // 0.2 s spacing: first accept at k=2 (t=100.4); k=3..11 suppressed by the
  // cooldown despite a growing streak; second accept at k=12 (t=102.4).
  EXPECT_EQ(accepts, 2);
}

// Streaks are per-target: interleaved candidates against two submaps must not
// poison each other.
TEST(LoopConsensus, PerTargetStreaksIndependent) {
  LoopConsensusGate g;
  EXPECT_FALSE(g.feed(walking(1, 0)));
  EXPECT_FALSE(g.feed(walking(2, 0)));
  EXPECT_FALSE(g.feed(walking(1, 1)));
  EXPECT_FALSE(g.feed(walking(2, 1)));
  EXPECT_TRUE(g.feed(walking(1, 2)));
  EXPECT_EQ(g.streak(2), 2);  // target 2 unaffected by target 1's accept...
  EXPECT_EQ(g.streak(1), 3);
}

// Stationary robot (T_OB constant) with consistent meas is a valid streak —
// the kf_max_dt keyframe arm produces exactly this.
TEST(LoopConsensus, StationaryRobotAccepts) {
  LoopConsensusGate g;
  for (int k = 0; k < 3; ++k) {
    LoopCandidate c;
    c.target_id = 7;
    c.q_id = k;
    c.t = 100.0 + k;
    c.T_OB = trans(1.0, 1.0, 0.0);
    c.meas = trans(0.3, 0.0, 0.0);
    if (k < 2) EXPECT_FALSE(g.feed(c));
    else EXPECT_TRUE(g.feed(c));
  }
}

}  // namespace
