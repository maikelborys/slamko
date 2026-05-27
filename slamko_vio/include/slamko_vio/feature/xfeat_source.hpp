// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// XFeatSource — the learned-feature FeatureSource (XFeat via TensorRT 10). Wraps
// the ported XFeat extractor behind slamko_core's contract, so the VIO front-end
// swaps Shi-Tomasi ↔ XFeat with no other change (feature_source:=xfeat). Output
// carries 64-d L2-normalized descriptors → the reloc map is built for free (B3).
//
// The TRT engine is static-shape (752×480 EuRoC-native): build() deserializes a
// cached engine or builds it from the ONNX on first run (~30 s) and saves it.

#pragma once

#include <cstdio>
#include <string>

#include <opencv2/opencv.hpp>

#include "slamko_core/feature_source.hpp"
#include "slamko_vio/feature/xfeat.h"

namespace slamko_vio {

class XFeatSource : public slamko::FeatureSource {
 public:
  explicit XFeatSource(const XFeatConfig& cfg) : xfeat_(cfg) {
    ok_ = xfeat_.build();
    if (!ok_) {
      std::fprintf(stderr, "[slamko_vio] XFeatSource: engine build FAILED "
                   "(onnx=%s engine=%s)\n",
                   cfg.onnx_file.c_str(), cfg.engine_file.c_str());
    }
  }

  std::string name() const override { return "xfeat"; }
  int descriptorDim() const override { return 64; }

  slamko::Features detect(const slamko::ImageView& img) override {
    slamko::Features f;
    if (!ok_ || img.empty()) return f;
    // Non-owning mono8 view → cv::Mat (XFeat resizes to the engine shape).
    const cv::Mat gray(img.height, img.width, CV_8UC1,
                       const_cast<std::uint8_t*>(img.data), img.step);
    Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic> feats;
    if (!xfeat_.infer(gray, feats)) return f;

    const int n = static_cast<int>(feats.cols());
    f.keypoints.resize(n, 3);
    f.descriptors.resize(n, 64);
    for (int i = 0; i < n; ++i) {
      f.keypoints(i, 0) = feats(1, i);   // x
      f.keypoints(i, 1) = feats(2, i);   // y
      f.keypoints(i, 2) = feats(0, i);   // score
      for (int d = 0; d < 64; ++d) f.descriptors(i, d) = feats(3 + d, i);
    }
    return f;
  }

  bool ok() const { return ok_; }

 private:
  XFeat xfeat_;
  bool ok_ = false;
};

}  // namespace slamko_vio
