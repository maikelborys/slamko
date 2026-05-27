// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// ImageView — a non-owning, OpenCV-free view of a grayscale image. Keeps the
// feature contracts (FeatureSource/FeatureTracker) in slamko_core dependency-
// free (Eigen only): a vio implementation wraps its cv::Mat / cv::cuda::GpuMat
// into an ImageView at the call boundary. No pixel data is copied or owned.

#pragma once

#include <cstdint>

namespace slamko {

struct ImageView {
  const std::uint8_t* data = nullptr;  // row-major, 1 byte/px (8-bit gray)
  int width = 0;
  int height = 0;
  int step = 0;                        // row stride in bytes (>= width)

  ImageView() = default;
  ImageView(const std::uint8_t* d, int w, int h, int s)
      : data(d), width(w), height(h), step(s) {}
  ImageView(const std::uint8_t* d, int w, int h) : ImageView(d, w, h, w) {}

  bool empty() const { return data == nullptr || width <= 0 || height <= 0; }
  const std::uint8_t* row(int y) const { return data + static_cast<long>(y) * step; }
};

}  // namespace slamko
