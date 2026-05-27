// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Bias measurement for StereoMatcher.
//
// Synthetic stereo pair: a sum of two low-frequency sinusoids (wavelengths
// 47 and 37 px, matching test_klt_tracker.cpp). The left image is rendered
// at zero offset. The matcher's convention is xR = xL - d, i.e. the right
// pixel at xR shows the SAME scene content that the left pixel at xR + d
// shows. So rendering: render right at offset (-disparity_true, 0), which
// makes right pixel (x, y) sample the canonical pattern at (x + d, y) —
// then for any left keypoint xL the corresponding right pixel at xR=xL-d
// samples the canonical at xR + d = xL, identical to the left.
//
// We then record the recovered sub-pixel disparity and the signed error,
// per-disparity, to detect any systematic bias (e.g. consistently larger
// recovered disparities → depth scale loss in the VO).

#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>

#include "slamko_vio/stereo_matcher.hpp"

namespace {

// Render the canonical 2D-sinusoid texture, translated by (ox, oy):
// pixel (x, y) of the rendered image samples the canonical pattern at
// (x - ox, y - oy). Sub-pixel-accurate by construction (analytic).
void render_sinusoidal(std::vector<std::uint8_t>& img, int W, int H,
                       float ox, float oy) {
  const float kx = 2.f * 3.14159265f / 47.f;
  const float ky = 2.f * 3.14159265f / 37.f;
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const float u = static_cast<float>(x) - ox;
      const float v = static_cast<float>(y) - oy;
      const float s = 0.5f + 0.25f * std::sin(kx * u) + 0.25f * std::sin(ky * v);
      img[y * W + x] = static_cast<std::uint8_t>(s * 255.f);
    }
  }
}

// Add zero-mean uniform noise of given amplitude in [0, 127] to an image.
void add_noise(std::vector<std::uint8_t>& img, int amplitude, unsigned seed) {
  std::srand(seed);
  for (auto& p : img) {
    int n = (std::rand() % (2 * amplitude + 1)) - amplitude;
    int v = static_cast<int>(p) + n;
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    p = static_cast<std::uint8_t>(v);
  }
}

// One bias measurement: render pair with the given true disparity, run the
// matcher on a keypoint at (xL, yL), return signed (recovered - true).
// Returns NaN on rejection.
struct Result {
  float disparity_true;
  float disparity_recovered;
  float error;
  int   status;
};

Result measure_disparity(float disparity_true, float xL, float yL,
                         int W, int H, int noise_amp = 0) {
  std::vector<std::uint8_t> left_img(W * H), right_img(W * H);
  render_sinusoidal(left_img,  W, H, 0.f, 0.f);
  render_sinusoidal(right_img, W, H, -disparity_true, 0.f);
  if (noise_amp > 0) {
    // Only noise the right image — see if NCC then drifts.
    add_noise(right_img, noise_amp, 0xDEADBEEFu);
  }

  std::uint8_t *d_left = nullptr, *d_right = nullptr;
  cudaMalloc(&d_left,  W * H);
  cudaMalloc(&d_right, W * H);
  cudaMemcpy(d_left,  left_img.data(),  W * H, cudaMemcpyHostToDevice);
  cudaMemcpy(d_right, right_img.data(), W * H, cudaMemcpyHostToDevice);

  slamko_vio::StereoMatcher::Config cfg;
  cfg.patch_size    = 11;
  cfg.min_disparity = 1;
  cfg.max_disparity = 50;
  cfg.ncc_threshold = 0.5f;
  cfg.border        = 8;
  slamko_vio::StereoMatcher matcher(W, H, cfg);

  matcher.set_images(d_left, W, d_right, W);

  float h_left_xy[2]  = {xL, yL};
  float h_right_xy[2] = {0.f, 0.f};
  std::int8_t h_status[1] = {0};

  float *d_left_xy = nullptr, *d_right_xy = nullptr;
  std::int8_t* d_status = nullptr;
  cudaMalloc(&d_left_xy,  sizeof(float) * 2);
  cudaMalloc(&d_right_xy, sizeof(float) * 2);
  cudaMalloc(&d_status,   sizeof(std::int8_t));
  cudaMemcpy(d_left_xy, h_left_xy, sizeof(float) * 2, cudaMemcpyHostToDevice);

  matcher.match(d_left_xy, d_right_xy, d_status, 1);
  cudaDeviceSynchronize();

  cudaMemcpy(h_right_xy, d_right_xy, sizeof(float) * 2, cudaMemcpyDeviceToHost);
  cudaMemcpy(h_status,   d_status,   sizeof(std::int8_t), cudaMemcpyDeviceToHost);

  cudaFree(d_left); cudaFree(d_right);
  cudaFree(d_left_xy); cudaFree(d_right_xy); cudaFree(d_status);

  Result r;
  r.disparity_true = disparity_true;
  r.status         = h_status[0];
  if (h_status[0] == 1) {
    r.disparity_recovered = xL - h_right_xy[0];
    r.error               = r.disparity_recovered - disparity_true;
  } else {
    r.disparity_recovered = std::nanf("");
    r.error               = std::nanf("");
  }
  return r;
}

}  // namespace

TEST(StereoMatcher, SubpixelDisparityBiasSweep) {
  const int W = 256;
  const int H = 256;
  const float xL = 128.f, yL = 128.f;

  const std::vector<float> disparities = {5.0f, 5.7f, 9.3f, 12.0f,
                                           17.5f, 23.0f, 31.4f};

  std::vector<float> errors;
  errors.reserve(disparities.size());

  std::cout << "\n=== StereoMatcher sub-pixel disparity bias sweep ===\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "  d_true     d_recov     error       status\n";
  std::cout << "  --------------------------------------------\n";

  for (float d_true : disparities) {
    Result r = measure_disparity(d_true, xL, yL, W, H, 0);
    std::cout << "  " << std::setw(7) << r.disparity_true << "   "
              << std::setw(7) << r.disparity_recovered << "   "
              << std::setw(+8) << r.error << "    "
              << r.status << "\n";
    EXPECT_EQ(r.status, 1) << "matcher rejected disparity " << d_true;
    if (r.status == 1) {
      // Loose gate (~1 px) — we want to MEASURE, not gate.
      EXPECT_NEAR(r.disparity_recovered, r.disparity_true, 1.0f);
      errors.push_back(r.error);
      ::testing::Test::RecordProperty(
          ("err_d" + std::to_string(static_cast<int>(d_true * 10))).c_str(),
          std::to_string(r.error));
    }
  }

  if (!errors.empty()) {
    double sum = 0.0, sumsq = 0.0;
    for (float e : errors) { sum += e; sumsq += static_cast<double>(e) * e; }
    const double mean = sum / errors.size();
    const double var  = sumsq / errors.size() - mean * mean;
    const double stdv = std::sqrt(std::max(0.0, var));
    auto mm = std::minmax_element(errors.begin(), errors.end());

    std::cout << "  --------------------------------------------\n";
    std::cout << "  N            : " << errors.size() << "\n";
    std::cout << "  mean error   : " << mean   << " px"
              << (mean > 0 ? "  (recovered > true; depth scale LOSS)"
                           : "  (recovered < true; depth scale GAIN)") << "\n";
    std::cout << "  std  error   : " << stdv   << " px\n";
    std::cout << "  min  error   : " << *mm.first  << " px\n";
    std::cout << "  max  error   : " << *mm.second << " px\n";
    std::cout << "  range        : " << (*mm.second - *mm.first) << " px\n";

    ::testing::Test::RecordProperty("mean_error_px", std::to_string(mean));
    ::testing::Test::RecordProperty("std_error_px",  std::to_string(stdv));
    ::testing::Test::RecordProperty("min_error_px",  std::to_string(*mm.first));
    ::testing::Test::RecordProperty("max_error_px",  std::to_string(*mm.second));
  }
}

TEST(StereoMatcher, SubpixelDisparityBiasUnderNoise) {
  const int W = 256;
  const int H = 256;
  const float xL = 128.f, yL = 128.f;
  const float d_true = 9.3f;

  std::cout << "\n=== StereoMatcher noise sweep (d_true = 9.3 px) ===\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "  noise_amp   d_recov     error       status\n";
  std::cout << "  --------------------------------------------\n";

  // Clean baseline + escalating noise; the LAST row is "VERY high noise".
  const std::vector<int> noises = {0, 10, 30, 60, 100};
  for (int n : noises) {
    Result r = measure_disparity(d_true, xL, yL, W, H, n);
    std::cout << "  " << std::setw(5) << n << "      "
              << std::setw(7) << r.disparity_recovered << "   "
              << std::setw(+8) << r.error << "    "
              << r.status << "\n";
    // No EXPECT_NEAR gate here — heavy noise CAN flip NCC to a sidelobe at
    // the far end of the search range, and that's exactly what we want to
    // see in the printout, not silence with a failure.
    if (r.status == 1) {
      ::testing::Test::RecordProperty(
          ("err_noise" + std::to_string(n)).c_str(),
          std::to_string(r.error));
    }
  }
}
