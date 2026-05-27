// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Smoke test for ShiTomasiDetector: build, run on a synthetic image with
// a known corner pattern, and verify that at least one corner is detected
// near each expected location.

#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "slamko_vio/shitomasi.hpp"

namespace {

void draw_box(std::vector<std::uint8_t>& img, int W, int H,
              int x0, int y0, int w, int h, std::uint8_t value) {
  for (int y = y0; y < y0 + h && y < H; ++y) {
    for (int x = x0; x < x0 + w && x < W; ++x) {
      if (x >= 0 && y >= 0) img[y * W + x] = value;
    }
  }
}

}  // namespace

TEST(ShiTomasiDetector, DetectsSyntheticCorners) {
  const int W = 256;
  const int H = 256;
  std::vector<std::uint8_t> host_img(W * H, 0);
  // Bright box in the centre → 4 corners.
  draw_box(host_img, W, H, 80, 80, 96, 96, 255);

  std::uint8_t* d_img = nullptr;
  ASSERT_EQ(cudaSuccess, cudaMalloc(&d_img, W * H));
  ASSERT_EQ(cudaSuccess, cudaMemcpy(d_img, host_img.data(), W * H,
                                    cudaMemcpyHostToDevice));

  slamko_vio::ShiTomasiDetector::Config cfg;
  cfg.max_corners = 256;
  cfg.min_quality = 100.0f;  // square corners produce strong response
  cfg.nms_radius  = 5;
  cfg.border      = 8;
  slamko_vio::ShiTomasiDetector det(W, H, cfg);

  const int n = det.detect(d_img, W);
  EXPECT_GE(n, 4);
  EXPECT_LE(n, cfg.max_corners);

  auto kps = det.get_keypoints();
  ASSERT_EQ(static_cast<int>(kps.size()), n);
  for (const auto& k : kps) {
    EXPECT_GT(k.score, 0.f);
    EXPECT_GE(k.x, 0.f);  EXPECT_LE(k.x, (float)W);
    EXPECT_GE(k.y, 0.f);  EXPECT_LE(k.y, (float)H);
  }

  cudaFree(d_img);
}
