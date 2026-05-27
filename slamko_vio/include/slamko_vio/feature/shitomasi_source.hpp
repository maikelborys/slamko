// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// ShiTomasiSource — the baseline FeatureSource (descriptor-less). Wraps the CUDA
// ShiTomasiDetector behind slamko_core's contract so the detector is swappable
// (XFeat / LiftFeat-m1 implement the same interface — B2). Owns its own device
// image and uploads the host ImageView internally, mirroring how XFeatSource's
// TRT path takes a host image; the ~360 KB H2D copy per frame is negligible
// against the ~5 ms frame budget and keeps the contract OpenCV/CUDA-free in core.

#pragma once

#include <cstdint>
#include <string>

#include <cuda_runtime.h>

#include "slamko_core/feature_source.hpp"
#include "slamko_vio/shitomasi.hpp"

namespace slamko_vio {

class ShiTomasiSource : public slamko::FeatureSource {
 public:
  ShiTomasiSource(int width, int height, const ShiTomasiDetector::Config& cfg)
      : width_(width), height_(height), det_(width, height, cfg) {
    cudaMalloc(&d_img_, static_cast<std::size_t>(width) * height);
  }
  ~ShiTomasiSource() override {
    if (d_img_) cudaFree(d_img_);
  }

  std::string name() const override { return "shitomasi"; }
  int descriptorDim() const override { return 0; }

  slamko::Features detect(const slamko::ImageView& img) override {
    cudaMemcpy2D(d_img_, width_, img.data, img.step, width_, height_,
                 cudaMemcpyHostToDevice);
    det_.detect(d_img_, width_);
    const auto kps = det_.get_keypoints();
    slamko::Features f;
    f.keypoints.resize(static_cast<int>(kps.size()), 3);
    for (int i = 0; i < static_cast<int>(kps.size()); ++i) {
      f.keypoints(i, 0) = kps[i].x;
      f.keypoints(i, 1) = kps[i].y;
      f.keypoints(i, 2) = kps[i].score;
    }
    return f;
  }

 private:
  int width_, height_;
  ShiTomasiDetector det_;
  std::uint8_t* d_img_ = nullptr;
};

}  // namespace slamko_vio
