// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// test_lightglue — sanity gate for the LighterGlue matcher. Only built with
// -DSLAMKO_LOOP_WITH_TORCH; skips (not fails) if the model .pt isn't present so
// it never blocks a CI box without the weights. Validates: the module loads,
// forward runs, the decode + top-K index mapping return ORIGINAL-row indices.

#include <cstdlib>
#include <random>
#include <string>

#include <gtest/gtest.h>

#include "slamko_loop/lightglue_matcher.hpp"

namespace {

// Resolve the lighterglue.pt the dev box keeps (gitignored weights).
std::string findModel() {
  const char* home = std::getenv("HOME");
  const std::string h = home ? home : "";
  for (const std::string& p :
       {h + "/coding/slamko/slamko_vio/models/lighterglue.pt",
        h + "/coding/AirSLAM_XFEAT/output/lighterglue.pt",
        h + "/coding/orbslam3_xfeat/orbslam3_xfeat_core/weights/lighterglue.pt"}) {
    if (FILE* f = std::fopen(p.c_str(), "rb")) { std::fclose(f); return p; }
  }
  return {};
}

// N random L2-normalized 64-d descriptors + random in-image keypoints.
slamko::Features makeFeatures(int N, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> ux(0.f, 752.f), uy(0.f, 480.f);
  std::normal_distribution<float> ud(0.f, 1.f);
  slamko::Features f;
  f.keypoints.resize(N, 3);
  f.descriptors.resize(N, 64);
  for (int i = 0; i < N; ++i) {
    f.keypoints(i, 0) = ux(rng);
    f.keypoints(i, 1) = uy(rng);
    f.keypoints(i, 2) = 1.0f;
    float nrm = 0.f;
    for (int c = 0; c < 64; ++c) { float v = ud(rng); f.descriptors(i, c) = v; nrm += v * v; }
    nrm = std::sqrt(nrm);
    for (int c = 0; c < 64; ++c) f.descriptors(i, c) /= nrm;
  }
  return f;
}

TEST(LightGlue, IdenticalViewMatchesItself) {
  const std::string model = findModel();
  if (model.empty()) GTEST_SKIP() << "lighterglue.pt not found — skipping";

  slamko::LightGlueConfig cfg;
  cfg.model_path = model;
  slamko::LightGlueMatcher m(cfg);
  ASSERT_TRUE(m.build()) << "model failed to load: " << model;

  // Same keypoints + same descriptors on both sides → the matcher must find the
  // i↔i correspondence for the large majority of points (identity is the easiest
  // case; this catches a broken decode / transposed I/O / wrong device).
  const int N = 300;
  slamko::Features f = makeFeatures(N, 7);
  const auto matches = m.match(f, f);
  EXPECT_GT(static_cast<int>(matches.size()), N / 2)
      << "too few matches on an identical view";
  int correct = 0;
  for (const auto& mm : matches) {
    EXPECT_LT(mm.query_idx, static_cast<std::uint32_t>(N));
    EXPECT_LT(mm.train_idx, static_cast<std::uint32_t>(N));
    if (mm.query_idx == mm.train_idx) ++correct;
  }
  EXPECT_GT(correct, static_cast<int>(matches.size()) * 8 / 10)
      << "identity correspondence not recovered";
}

TEST(LightGlue, TopKAboveTraceNStaysValid) {
  const std::string model = findModel();
  if (model.empty()) GTEST_SKIP() << "lighterglue.pt not found — skipping";

  slamko::LightGlueConfig cfg;
  cfg.model_path = model;  // trace_n = 512 default
  slamko::LightGlueMatcher m(cfg);
  ASSERT_TRUE(m.build());

  // 700 > trace_n(512): the matcher must top-K internally and still return
  // indices into the ORIGINAL 700-row Features (never out of range).
  const int N = 700;
  slamko::Features f = makeFeatures(N, 11);
  const auto matches = m.match(f, f);
  EXPECT_GT(static_cast<int>(matches.size()), 50);
  for (const auto& mm : matches) {
    EXPECT_LT(mm.query_idx, static_cast<std::uint32_t>(N));
    EXPECT_LT(mm.train_idx, static_cast<std::uint32_t>(N));
  }
}

}  // namespace
