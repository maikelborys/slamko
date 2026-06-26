// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// ScanContext = the VIEWPOINT-INVARIANT geometric loop channel. These tests assert the
// load-bearing property the appearance channel LACKS: a place rotated in yaw still
// matches (low distance), with the column shift recovering the rotation; and a DIFFERENT
// scene is far. This is what lets a different-heading revisit close a loop / self-correct
// a catastrophic provider jump where EigenPlaces (image overlap) fails.

#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "slamko_loop/scan_context.hpp"

using slamko::ScanContext;
using slamko::ScanContextConfig;

namespace {
// A deterministic "room": points on 4 walls of a 6×4 box, varied heights — in a
// gravity-aligned frame (z up), centered at the origin.
std::vector<Eigen::Vector3d> room() {
  std::vector<Eigen::Vector3d> pts;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> h(0.2, 2.4);
  for (double t = -3; t <= 3; t += 0.1) {
    pts.emplace_back(t, 2.0, h(rng));   // far wall
    pts.emplace_back(t, -2.0, h(rng));  // near wall
  }
  for (double t = -2; t <= 2; t += 0.1) {
    pts.emplace_back(3.0, t, h(rng));   // right wall
    pts.emplace_back(-3.0, t, h(rng));  // left wall
  }
  return pts;
}

std::vector<Eigen::Vector3d> rotateYaw(const std::vector<Eigen::Vector3d>& in, double deg) {
  const double a = deg * M_PI / 180.0, c = std::cos(a), s = std::sin(a);
  std::vector<Eigen::Vector3d> out;
  out.reserve(in.size());
  for (const auto& p : in) out.emplace_back(c * p.x() - s * p.y(), s * p.x() + c * p.y(), p.z());
  return out;
}
}  // namespace

TEST(ScanContext, ComputeNonEmpty) {
  auto d = ScanContext::compute(room());
  ASSERT_FALSE(d.empty());
  EXPECT_EQ(d.sc.rows(), ScanContextConfig{}.n_ring);
  EXPECT_EQ(d.sc.cols(), ScanContextConfig{}.n_sector);
  EXPECT_EQ(d.ring_key.size(), ScanContextConfig{}.n_ring);
}

TEST(ScanContext, EmptyOnNoPointsInRange) {
  std::vector<Eigen::Vector3d> far = {{100, 100, 1}};  // beyond max_range
  EXPECT_TRUE(ScanContext::compute(far).empty());
}

TEST(ScanContext, YawInvariantSamePlace) {
  ScanContextConfig cfg;  // 60 sectors → 6°/sector
  auto a = ScanContext::compute(room(), cfg);
  // rotate the SAME room by 90° in yaw — appearance would change totally, geometry not.
  auto b = ScanContext::compute(rotateYaw(room(), 90.0), cfg);
  auto [dist, shift] = ScanContext::distance(a.sc, b.sc);
  EXPECT_LT(dist, 0.15f) << "a yaw-rotated SAME place must match (geometric invariance)";
  // best shift should recover ~90° = 15 sectors (±2 sectors tolerance for binning)
  const int expected = 90 / (360 / cfg.n_sector);
  const int err = std::min(std::abs(shift - expected), cfg.n_sector - std::abs(shift - expected));
  EXPECT_LE(err, 2) << "column shift should recover the yaw (got " << shift << ", want " << expected << ")";
  // ring key is yaw-invariant -> identical place -> near-zero ring-key distance
  EXPECT_LT(ScanContext::ringKeyDistance(a.ring_key, b.ring_key), 0.05f);
}

TEST(ScanContext, DifferentSceneIsFar) {
  auto a = ScanContext::compute(room());
  std::vector<Eigen::Vector3d> corridor;  // a long narrow hallway, different shape
  for (double t = -7; t <= 7; t += 0.05) {
    corridor.emplace_back(t, 0.8, 1.0);
    corridor.emplace_back(t, -0.8, 1.0);
  }
  auto b = ScanContext::compute(corridor);
  auto [dist, shift] = ScanContext::distance(a.sc, b.sc);
  (void)shift;
  EXPECT_GT(dist, 0.25f) << "a geometrically different scene must be far";
}

TEST(ScanContext, IdenticalIsZero) {
  auto a = ScanContext::compute(room());
  auto [dist, shift] = ScanContext::distance(a.sc, a.sc);
  EXPECT_EQ(shift, 0);
  EXPECT_LT(dist, 1e-5f);
}
