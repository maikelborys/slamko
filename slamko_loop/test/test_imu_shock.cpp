// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// ImuShockDetector = the kidnap/knock/drop trigger. Tests assert the load-bearing property:
// a CALM stream (gravity + small motion) flags NOTHING (no false positives that would seal a
// healthy map), while the three disturbance signatures — IMPACT (jerk spike), FREEFALL (|accel|
// ~0), YANK (gyro spike) — each fire. This is the channel that catches a clean lift/bump the
// pose-based quality-break misses (the provider coasts on IMU → no speed-jump).

#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "slamko_loop/imu_shock.hpp"

using slamko::ImuShockDetector;
using slamko::ImuShockConfig;
using slamko::ShockKind;

namespace {
const Eigen::Vector3d G(0, 0, 9.81);  // gravity in the accel reading at rest, optical-ish z
}

TEST(ImuShock, CalmStreamNoFalsePositive) {
  ImuShockDetector det;
  int flags = 0;
  double t = 0.0;
  for (int i = 0; i < 200; ++i, t += 0.005) {
    // gravity + gentle handheld wobble (small accel + small gyro)
    Eigen::Vector3d a = G + Eigen::Vector3d(0.3 * std::sin(i * 0.1), 0.2, 0.1);
    Eigen::Vector3d w(0.1 * std::cos(i * 0.1), 0.05, 0.02);
    if (det.feed(a, w, 0.005, t).detected()) ++flags;
  }
  EXPECT_EQ(flags, 0) << "a calm walk must not trigger a shock (no false seals)";
}

TEST(ImuShock, ImpactSpikeFires) {
  ImuShockDetector det;
  double t = 0.0;
  // calm, then a one-sample accel SPIKE (a bump): |accel| jumps high → big jerk.
  for (int i = 0; i < 10; ++i, t += 0.005) det.feed(G, Eigen::Vector3d::Zero(), 0.005, t);
  const auto ev = det.feed(G + Eigen::Vector3d(0, 0, 60.0), Eigen::Vector3d::Zero(), 0.005, t);
  EXPECT_EQ(ev.kind, ShockKind::Impact) << "a sharp accel jerk = IMPACT";
  EXPECT_GT(ev.jerk, 200.0);
}

TEST(ImuShock, FreefallFires) {
  ImuShockDetector det;
  double t = 0.0;
  for (int i = 0; i < 10; ++i, t += 0.005) det.feed(G, Eigen::Vector3d::Zero(), 0.005, t);
  // a drop: the accelerometer reads ~0 in free fall.
  const auto ev = det.feed(Eigen::Vector3d(0.1, 0.0, 0.2), Eigen::Vector3d::Zero(), 0.005, t);
  EXPECT_EQ(ev.kind, ShockKind::Freefall) << "|accel|~0 = FREEFALL (drop/flip/lift)";
}

TEST(ImuShock, YankFires) {
  ImuShockDetector det;
  double t = 0.0;
  for (int i = 0; i < 10; ++i, t += 0.005) det.feed(G, Eigen::Vector3d::Zero(), 0.005, t);
  // a fast snatch/flip: high angular rate.
  const auto ev = det.feed(G, Eigen::Vector3d(12.0, 1.0, 0.0), 0.005, t);
  EXPECT_EQ(ev.kind, ShockKind::Yank) << "a gyro-rate spike = YANK/flip";
}

TEST(ImuShock, RefractorySuppressesRepeat) {
  ImuShockDetector det;
  double t = 0.0;
  det.feed(G, Eigen::Vector3d::Zero(), 0.005, t);
  // two impacts within the refractory window → only the FIRST is reported.
  const auto e1 = det.feed(G + Eigen::Vector3d(0, 0, 60.0), Eigen::Vector3d::Zero(), 0.005, t += 0.01);
  const auto e2 = det.feed(G + Eigen::Vector3d(0, 0, 60.0), Eigen::Vector3d::Zero(), 0.005, t += 0.01);
  EXPECT_TRUE(e1.detected());
  EXPECT_FALSE(e2.detected()) << "refractory suppresses re-triggering the same event";
}
