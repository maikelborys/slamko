// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Smoke test for KltTracker: track a corner across two synthetic images
// that differ by a known sub-pixel translation, and verify the tracker
// recovers the shift.

#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "slamko_vio/klt_tracker.hpp"

namespace {

// Synthetic image with sinusoidal texture: rich gradient everywhere, no
// saturation, KLT-friendly. Translated views of this pattern produce a
// clean shift for the tracker to recover.
void render_sinusoidal(std::vector<std::uint8_t>& img, int W, int H,
                       float ox, float oy) {
  // Wavelengths chosen so that, after 3x dyadic downsampling (1/8 resolution),
  // the period is still well above Nyquist: 47/8 ≈ 6 px and 37/8 ≈ 4.6 px.
  const float kx = 2.f * 3.14159265f / 47.f;
  const float ky = 2.f * 3.14159265f / 37.f;
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const float u = (float)x - ox;
      const float v = (float)y - oy;
      // Two perpendicular sinusoids → unique 2D pattern, strong gradients
      // in both x and y, no saturation.
      const float s = 0.5f + 0.25f * std::sin(kx * u) + 0.25f * std::sin(ky * v);
      img[y * W + x] = (std::uint8_t)(s * 255.f);
    }
  }
}

}  // namespace

TEST(KltTracker, TracksTranslatedPattern_SingleLevel) {
  // Single-level, sub-pixel shift — isolates the inner KLT loop.
  const int W = 128;
  const int H = 128;
  const float feat_x = 64.f, feat_y = 64.f;
  const float shift_x = 0.7f, shift_y = -0.3f;

  std::vector<std::uint8_t> prev_img(W * H), curr_img(W * H);
  render_sinusoidal(prev_img, W, H, 0.f, 0.f);
  render_sinusoidal(curr_img, W, H, shift_x, shift_y);

  std::uint8_t *d_prev = nullptr, *d_curr = nullptr;
  ASSERT_EQ(cudaSuccess, cudaMalloc(&d_prev, W * H));
  ASSERT_EQ(cudaSuccess, cudaMalloc(&d_curr, W * H));
  cudaMemcpy(d_prev, prev_img.data(), W * H, cudaMemcpyHostToDevice);
  cudaMemcpy(d_curr, curr_img.data(), W * H, cudaMemcpyHostToDevice);

  slamko_vio::KltTracker::Config cfg;
  cfg.pyramid_levels = 1;
  cfg.patch_size     = 9;
  cfg.max_iterations = 40;
  cfg.epsilon        = 0.005f;
  slamko_vio::KltTracker tracker(W, H, cfg);

  tracker.set_image(d_prev, W);
  tracker.swap_pyramids();
  tracker.set_image(d_curr, W);

  float h_prev_xy[2] = {feat_x, feat_y};
  float h_curr_xy[2] = {0.f, 0.f};
  std::int8_t h_status[1] = {0};

  float *d_prev_xy, *d_curr_xy; std::int8_t* d_status;
  cudaMalloc(&d_prev_xy, sizeof(float) * 2);
  cudaMalloc(&d_curr_xy, sizeof(float) * 2);
  cudaMalloc(&d_status,  sizeof(std::int8_t));
  cudaMemcpy(d_prev_xy, h_prev_xy, sizeof(float) * 2, cudaMemcpyHostToDevice);

  tracker.track(d_prev_xy, d_curr_xy, d_status, 1);
  cudaMemcpy(h_curr_xy, d_curr_xy, sizeof(float) * 2, cudaMemcpyDeviceToHost);
  cudaMemcpy(h_status,  d_status,  sizeof(std::int8_t), cudaMemcpyDeviceToHost);

  EXPECT_EQ(h_status[0], 1);
  EXPECT_NEAR(h_curr_xy[0], feat_x + shift_x, 0.2f);
  EXPECT_NEAR(h_curr_xy[1], feat_y + shift_y, 0.2f);

  cudaFree(d_prev); cudaFree(d_curr);
  cudaFree(d_prev_xy); cudaFree(d_curr_xy); cudaFree(d_status);
}

TEST(KltTracker, TracksTranslatedPattern_Multilevel) {
  const int W = 256;
  const int H = 256;
  const float feat_x = 128.f, feat_y = 128.f;
  const float shift_x = 3.5f, shift_y = -1.25f;

  // prev image is the canonical view; curr image is the same texture
  // translated by (shift_x, shift_y). A feature at (feat_x, feat_y) in
  // prev should end up at (feat_x + shift_x, feat_y + shift_y) in curr.
  std::vector<std::uint8_t> prev_img(W * H), curr_img(W * H);
  render_sinusoidal(prev_img, W, H, 0.f, 0.f);
  render_sinusoidal(curr_img, W, H, shift_x, shift_y);

  std::uint8_t *d_prev = nullptr, *d_curr = nullptr;
  ASSERT_EQ(cudaSuccess, cudaMalloc(&d_prev, W * H));
  ASSERT_EQ(cudaSuccess, cudaMalloc(&d_curr, W * H));
  cudaMemcpy(d_prev, prev_img.data(), W * H, cudaMemcpyHostToDevice);
  cudaMemcpy(d_curr, curr_img.data(), W * H, cudaMemcpyHostToDevice);

  slamko_vio::KltTracker::Config cfg;
  cfg.pyramid_levels = 4;
  cfg.patch_size     = 9;
  cfg.max_iterations = 15;
  slamko_vio::KltTracker tracker(W, H, cfg);

  tracker.set_image(d_prev, W);
  tracker.swap_pyramids();
  tracker.set_image(d_curr, W);

  float h_prev_xy[2] = {feat_x, feat_y};
  float h_curr_xy[2] = {0.f, 0.f};
  std::int8_t h_status[1] = {0};

  float *d_prev_xy, *d_curr_xy; std::int8_t* d_status;
  cudaMalloc(&d_prev_xy, sizeof(float) * 2);
  cudaMalloc(&d_curr_xy, sizeof(float) * 2);
  cudaMalloc(&d_status,  sizeof(std::int8_t));
  cudaMemcpy(d_prev_xy, h_prev_xy, sizeof(float) * 2, cudaMemcpyHostToDevice);

  tracker.track(d_prev_xy, d_curr_xy, d_status, 1);
  cudaMemcpy(h_curr_xy, d_curr_xy, sizeof(float) * 2, cudaMemcpyDeviceToHost);
  cudaMemcpy(h_status,  d_status,  sizeof(std::int8_t), cudaMemcpyDeviceToHost);

  EXPECT_EQ(h_status[0], 1);
  EXPECT_NEAR(h_curr_xy[0], feat_x + shift_x, 0.5f);
  EXPECT_NEAR(h_curr_xy[1], feat_y + shift_y, 0.5f);

  cudaFree(d_prev); cudaFree(d_curr);
  cudaFree(d_prev_xy); cudaFree(d_curr_xy); cudaFree(d_status);
}
