// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Unit-tests the LIVE policy (VolumetricLiveDriver) with a fake backend — no
// nvblox, no GPU. Under test: forward-fuse on addKeyframe; window ONLY the
// keyframes whose corrected pose moved past threshold on applyCorrection; bound
// the store by sealing the oldest frames outside the recent window.

#include <gtest/gtest.h>

#include <memory>
#include <unordered_map>

#include "slamko_core/volumetric_map.hpp"
#include "slamko_tsdf/live_driver.hpp"

using namespace slamko;

namespace {

struct FakeBackend : public VolumetricBackend {
  int resets = 0;
  std::vector<std::pair<std::uint64_t, SE3>> integrated;
  std::vector<Aabb> cleared;
  void integrate(const DepthFrame& f, const SE3& T) override {
    integrated.emplace_back(f.kf_id, T);
  }
  void reset() override { ++resets; integrated.clear(); }
  void clearRegion(const Aabb& r) override { cleared.push_back(r); }
  CostmapSlice exportCostmap(const CostmapParams&) override { return {}; }
};

DepthFrame makeFrame(std::uint64_t kf_id, int w = 2, int h = 2) {
  DepthFrame f;
  f.kf_id = kf_id;
  f.width = w; f.height = h;
  f.depth.assign(static_cast<std::size_t>(w) * h, 1.0f);
  f.K = {100, 100, 1, 1, w, h};
  return f;
}
SE3 transAt(double x) { return SE3(SO3(), Eigen::Vector3d(x, 0, 0)); }

std::unique_ptr<VolumetricLiveDriver> makeDriver(FakeBackend** out,
                                                 LiveParams lp = {}) {
  auto fake = std::make_unique<FakeBackend>();
  *out = fake.get();
  return std::make_unique<VolumetricLiveDriver>(std::move(fake),
                                                VolumetricParams{}, lp);
}

}  // namespace

TEST(LiveDriver, AddKeyframeForwardFusesAndTracks) {
  FakeBackend* fb;
  auto d = makeDriver(&fb);
  EXPECT_TRUE(d->addKeyframe(makeFrame(1), transAt(0.0)));
  EXPECT_TRUE(d->addKeyframe(makeFrame(2), transAt(20.0)));
  EXPECT_EQ(fb->resets, 0);
  EXPECT_EQ(fb->integrated.size(), 2u);
  EXPECT_EQ(d->numKeyframes(), 2u);
  EXPECT_EQ(d->numLiveFrames(), 2u);

  DepthFrame bad = makeFrame(3);
  bad.depth.clear();
  EXPECT_FALSE(d->addKeyframe(bad, transAt(1.0)));  // invalid → not tracked
  EXPECT_EQ(d->numKeyframes(), 2u);
}

TEST(LiveDriver, ApplyCorrectionWindowsOnlyMovedKeyframes) {
  FakeBackend* fb;
  auto d = makeDriver(&fb);
  d->addKeyframe(makeFrame(1), transAt(0.0));
  d->addKeyframe(makeFrame(2), transAt(20.0));
  const std::size_t before = fb->integrated.size();

  // Snapshot: kf1 moved 1 m (past 0.05 m threshold), kf2 unchanged.
  std::unordered_map<std::uint64_t, SE3> poses;
  poses[1] = transAt(1.0);
  poses[2] = transAt(20.0);
  const std::size_t n = d->applyCorrection(poses);

  EXPECT_EQ(n, 1u);                          // only kf1 re-fused
  EXPECT_EQ(fb->resets, 0);                  // window path, not full
  ASSERT_EQ(fb->cleared.size(), 1u);
  ASSERT_EQ(fb->integrated.size(), before + 1u);
  EXPECT_EQ(fb->integrated.back().first, 1u);
  EXPECT_DOUBLE_EQ(fb->integrated.back().second.translation().x(), 1.0);
}

TEST(LiveDriver, SubThresholdMoveDoesNotReintegrate) {
  FakeBackend* fb;
  auto d = makeDriver(&fb);
  d->addKeyframe(makeFrame(1), transAt(0.0));
  const std::size_t before = fb->integrated.size();

  std::unordered_map<std::uint64_t, SE3> poses;
  poses[1] = transAt(0.01);  // 1 cm < 5 cm threshold
  EXPECT_EQ(d->applyCorrection(poses), 0u);
  EXPECT_EQ(fb->integrated.size(), before);
  EXPECT_TRUE(fb->cleared.empty());
}

TEST(LiveDriver, SubThresholdDriftAccumulatesNotResets) {
  // A series of sub-threshold nudges that SUM past the threshold must eventually
  // fire (the baseline is the last INTEGRATION, not the last snapshot — so a slow
  // creep is caught, not perpetually reset).
  FakeBackend* fb;
  auto d = makeDriver(&fb);
  d->addKeyframe(makeFrame(1), transAt(0.0));

  std::size_t fired = 0;
  for (int i = 1; i <= 10; ++i) {
    std::unordered_map<std::uint64_t, SE3> poses;
    poses[1] = transAt(0.02 * i);  // 2 cm steps; crosses 5 cm at step 3
    fired += d->applyCorrection(poses);
  }
  EXPECT_GT(fired, 0u);  // the creep was caught at least once
}

TEST(LiveDriver, EnforceBudgetSealsOldestOutsideRecentWindow) {
  FakeBackend* fb;
  LiveParams lp;
  lp.store_budget_bytes = 48;  // each 2x2 frame = 16 B → keep ≤3 frames
  lp.keep_recent = 2;          // never seal the last 2
  auto d = makeDriver(&fb, lp);
  for (std::uint64_t i = 0; i < 5; ++i)
    d->addKeyframe(makeFrame(i), transAt(static_cast<double>(i)));
  EXPECT_EQ(d->storeBytes(), 80u);  // 5 × 16

  const std::size_t sealed = d->enforceBudget();
  EXPECT_EQ(sealed, 2u);                 // kf0, kf1 sealed → 48 B, under budget
  EXPECT_LE(d->storeBytes(), lp.store_budget_bytes);
  EXPECT_EQ(d->numLiveFrames(), 3u);     // kf2 (stopped), kf3, kf4 (recent) kept

  // A correction that moves a SEALED kf must not re-fuse it (frozen geometry).
  std::unordered_map<std::uint64_t, SE3> poses;
  for (std::uint64_t i = 0; i < 5; ++i) poses[i] = transAt(100.0 + i);
  const std::size_t before = fb->integrated.size();
  d->applyCorrection(poses);
  // Only the 3 unsealed frames are eligible to re-fuse; kf0/kf1 never reappear.
  for (std::size_t i = before; i < fb->integrated.size(); ++i)
    EXPECT_GE(fb->integrated[i].first, 2u);
}

TEST(LiveDriver, UnboundedBudgetSealsNothing) {
  FakeBackend* fb;
  auto d = makeDriver(&fb);  // store_budget_bytes default 0
  for (std::uint64_t i = 0; i < 5; ++i)
    d->addKeyframe(makeFrame(i), transAt(static_cast<double>(i)));
  EXPECT_EQ(d->enforceBudget(), 0u);
  EXPECT_EQ(d->numLiveFrames(), 5u);
}
