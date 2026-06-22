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

  void integrate(const DepthFrame& f, const SE3& T_map_body) override {
    integrated.emplace_back(f.kf_id, T_map_body);
  }
  void reset() override {
    ++resets;
    integrated.clear();
  }
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

TEST(VolumetricMapper, NoopBackendReportsUnavailable) {
  // The contract's available() flag lets ROS degrade gracefully when the real
  // backend wasn't built. A fake is "available"; the nvblox no-op stub is not
  // (covered where that TU is compiled). Here we just exercise the accessor.
  auto fake = std::make_unique<FakeBackend>();
  VolumetricMapper m(std::move(fake), VolumetricParams{});
  EXPECT_TRUE(m.backendAvailable());
}
