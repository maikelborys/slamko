// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Unit-tests the BEND policy (VolumetricMapper) with a fake backend — no nvblox,
// no CUDA. The contract under test: reintegrate() resets the backend and fuses
// each VALID frame at its keyframe's corrected pose, skipping frames whose
// keyframe is absent from the map (a dangling island). This is the slamko-native
// guarantee that the dense map follows the corrected graph.

#include <gtest/gtest.h>

#include <memory>
#include <unordered_map>
#include <vector>

#include "slamko_core/volumetric_map.hpp"
#include "slamko_tsdf/volumetric_mapper.hpp"

using namespace slamko;

namespace {

// Records every integrate() call so the test can assert WHAT was fused and at
// WHICH pose — the only thing the bend policy is responsible for.
struct FakeBackend : public VolumetricBackend {
  int resets = 0;
  std::vector<std::pair<std::uint64_t, SE3>> integrated;  // (kf_id, T_map_body)
  std::vector<Aabb> cleared;                              // clearRegion() calls

  void integrate(const DepthFrame& f, const SE3& T_map_body) override {
    integrated.emplace_back(f.kf_id, T_map_body);
  }
  void reset() override {
    ++resets;
    integrated.clear();
  }
  void clearRegion(const Aabb& region) override { cleared.push_back(region); }
  CostmapSlice exportCostmap(const CostmapParams&) override {
    CostmapSlice s;
    s.width = 1;
    s.height = 1;
    s.data = {0.f};
    return s;
  }
  bool available() const override { return true; }
};

DepthFrame makeFrame(std::uint64_t kf_id, int w = 2, int h = 2) {
  DepthFrame f;
  f.kf_id = kf_id;
  f.width = w;
  f.height = h;
  f.depth.assign(static_cast<std::size_t>(w) * h, 1.0f);
  f.K = {100, 100, 1, 1, w, h};
  return f;
}

SE3 transAt(double x) {
  return SE3(SO3(), Eigen::Vector3d(x, 0, 0));
}

}  // namespace

TEST(VolumetricMapper, ReintegratesValidFramesAtCorrectedPoses) {
  auto fake = std::make_unique<FakeBackend>();
  FakeBackend* fb = fake.get();
  VolumetricMapper m(std::move(fake), VolumetricParams{});

  m.addFrame(makeFrame(1));
  m.addFrame(makeFrame(2));
  EXPECT_EQ(m.numFrames(), 2u);

  std::unordered_map<std::uint64_t, SE3> poses;
  poses[1] = transAt(1.0);
  poses[2] = transAt(2.0);

  const std::size_t n = m.reintegrate(poses);
  EXPECT_EQ(n, 2u);
  EXPECT_EQ(fb->resets, 1);  // reset exactly once, before fusing
  ASSERT_EQ(fb->integrated.size(), 2u);
  EXPECT_EQ(fb->integrated[0].first, 1u);
  EXPECT_DOUBLE_EQ(fb->integrated[0].second.translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(fb->integrated[1].second.translation().x(), 2.0);
}

TEST(VolumetricMapper, SkipsFramesWhoseKeyframeIsNotAnchored) {
  auto fake = std::make_unique<FakeBackend>();
  FakeBackend* fb = fake.get();
  VolumetricMapper m(std::move(fake), VolumetricParams{});

  m.addFrame(makeFrame(1));
  m.addFrame(makeFrame(99));  // dangling — not in the pose map

  std::unordered_map<std::uint64_t, SE3> poses;
  poses[1] = transAt(0.5);

  const std::size_t n = m.reintegrate(poses);
  EXPECT_EQ(n, 1u);
  ASSERT_EQ(fb->integrated.size(), 1u);
  EXPECT_EQ(fb->integrated[0].first, 1u);
}

TEST(VolumetricMapper, SkipsInvalidFrames) {
  auto fake = std::make_unique<FakeBackend>();
  FakeBackend* fb = fake.get();
  VolumetricMapper m(std::move(fake), VolumetricParams{});

  DepthFrame bad = makeFrame(1);
  bad.depth.clear();  // size mismatch ⇒ invalid
  m.addFrame(bad);

  std::unordered_map<std::uint64_t, SE3> poses;
  poses[1] = transAt(0.0);

  EXPECT_EQ(m.reintegrate(poses), 0u);
  EXPECT_TRUE(fb->integrated.empty());
}

// ---- LIVE path -------------------------------------------------------------

TEST(VolumetricMapper, IntegrateLiveFusesImmediatelyWithoutReset) {
  auto fake = std::make_unique<FakeBackend>();
  FakeBackend* fb = fake.get();
  VolumetricMapper m(std::move(fake), VolumetricParams{});

  EXPECT_TRUE(m.integrateLive(makeFrame(1), transAt(0.0)));
  EXPECT_TRUE(m.integrateLive(makeFrame(2), transAt(20.0)));

  EXPECT_EQ(fb->resets, 0);                 // forward path never resets
  ASSERT_EQ(fb->integrated.size(), 2u);     // both fused on arrival
  EXPECT_EQ(fb->integrated[1].first, 2u);
  EXPECT_DOUBLE_EQ(fb->integrated[1].second.translation().x(), 20.0);
  EXPECT_EQ(m.numLiveFrames(), 2u);

  DepthFrame bad = makeFrame(3);
  bad.depth.clear();
  EXPECT_FALSE(m.integrateLive(bad, transAt(1.0)));  // invalid → not stored/fused
  EXPECT_EQ(fb->integrated.size(), 2u);
}

TEST(VolumetricMapper, ReintegrateWindowClearsAndRefusesOnlyOverlap) {
  auto fake = std::make_unique<FakeBackend>();
  FakeBackend* fb = fake.get();
  VolumetricMapper m(std::move(fake), VolumetricParams{});

  m.integrateLive(makeFrame(1), transAt(0.0));    // footprint near x≈0
  m.integrateLive(makeFrame(2), transAt(20.0));   // far away, x≈20
  const std::size_t before = fb->integrated.size();

  // A loop nudges only kf1. Only frames overlapping kf1's touched region re-fuse.
  std::unordered_map<std::uint64_t, SE3> poses;
  poses[1] = transAt(0.05);
  poses[2] = transAt(20.0);
  const std::size_t n = m.reintegrateWindow({1}, poses);

  EXPECT_EQ(n, 1u);                          // only kf1 re-fused
  EXPECT_EQ(fb->resets, 0);                  // window path: no full reset
  ASSERT_EQ(fb->cleared.size(), 1u);         // exactly one region cleared
  ASSERT_EQ(fb->integrated.size(), before + 1u);
  EXPECT_EQ(fb->integrated.back().first, 1u);
  EXPECT_DOUBLE_EQ(fb->integrated.back().second.translation().x(), 0.05);
}

TEST(VolumetricMapper, ReintegrateWindowFallsBackToFullWhenMostMoved) {
  auto fake = std::make_unique<FakeBackend>();
  FakeBackend* fb = fake.get();
  VolumetricMapper m(std::move(fake), VolumetricParams{});

  m.integrateLive(makeFrame(1), transAt(0.0));
  m.integrateLive(makeFrame(2), transAt(20.0));

  std::unordered_map<std::uint64_t, SE3> poses;
  poses[1] = transAt(0.1);
  poses[2] = transAt(20.1);
  const std::size_t n = m.reintegrateWindow({1, 2}, poses);  // 100% moved

  EXPECT_EQ(n, 2u);
  EXPECT_EQ(fb->resets, 1);                  // fell back to a clean full rebuild
  EXPECT_TRUE(fb->cleared.empty());          // no region-clear on the full path
  ASSERT_EQ(fb->integrated.size(), 2u);      // reset cleared the live calls, re-fused 2
}

TEST(VolumetricMapper, SealFrameDropsPayloadAndIsSkippedByWindow) {
  auto fake = std::make_unique<FakeBackend>();
  FakeBackend* fb = fake.get();
  VolumetricMapper m(std::move(fake), VolumetricParams{});

  m.integrateLive(makeFrame(1), transAt(0.0));   // same region as kf2 — but sealed
  m.integrateLive(makeFrame(2), transAt(0.0));
  m.integrateLive(makeFrame(3), transAt(20.0));  // far away (keeps us off fallback)
  const std::size_t bytes_before = m.storeBytes();
  EXPECT_GT(bytes_before, 0u);

  EXPECT_TRUE(m.sealFrame(1));
  EXPECT_FALSE(m.sealFrame(1));               // already sealed
  EXPECT_LT(m.storeBytes(), bytes_before);    // payload released
  EXPECT_EQ(m.numLiveFrames(), 2u);           // kf2, kf3 still re-poseable

  // Window touches kf1+kf2's region. kf1 is sealed → frozen (NOT re-fused even
  // though it overlaps); only kf2 re-fuses. Proves the bound trades re-poseability.
  const std::size_t before = fb->integrated.size();
  std::unordered_map<std::uint64_t, SE3> poses;
  poses[2] = transAt(0.05);
  const std::size_t n = m.reintegrateWindow({2}, poses);
  EXPECT_EQ(n, 1u);                           // kf2 only (kf1 sealed, kf3 disjoint)
  ASSERT_EQ(fb->cleared.size(), 1u);          // window path, not fallback
  EXPECT_EQ(fb->integrated.size(), before + 1u);
  EXPECT_EQ(fb->integrated.back().first, 2u);
}

TEST(VolumetricMapper, ReintegrateWindowNoopOnEmptyOrUnknownMoved) {
  auto fake = std::make_unique<FakeBackend>();
  FakeBackend* fb = fake.get();
  VolumetricMapper m(std::move(fake), VolumetricParams{});
  m.integrateLive(makeFrame(1), transAt(0.0));

  EXPECT_EQ(m.reintegrateWindow({}, {}), 0u);           // nothing moved
  std::unordered_map<std::uint64_t, SE3> poses;
  poses[1] = transAt(0.0);
  EXPECT_EQ(m.reintegrateWindow({42}, poses), 0u);      // moved kf we don't hold
  EXPECT_TRUE(fb->cleared.empty());
}

TEST(VolumetricMapper, NoopBackendReportsUnavailable) {
  // The contract's available() flag lets ROS degrade gracefully when the real
  // backend wasn't built. A fake is "available"; the nvblox no-op stub is not
  // (covered where that TU is compiled). Here we just exercise the accessor.
  auto fake = std::make_unique<FakeBackend>();
  VolumetricMapper m(std::move(fake), VolumetricParams{});
  EXPECT_TRUE(m.backendAvailable());
}
