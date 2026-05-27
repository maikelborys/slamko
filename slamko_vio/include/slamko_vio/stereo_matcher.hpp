// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// CUDA stereo matcher for rectified pairs.
//
// For each input keypoint in the left image, performs a one-dimensional
// (horizontal) NCC search along the epipolar line of the right image,
// across a disparity range. Sub-pixel disparity is recovered by a parabolic
// fit over the three NCC samples nearest the integer peak.
//
// Inputs are mono8 device images with arbitrary row pitch; internally the
// matcher converts to float for the NCC computation.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "slamko_vio/types.hpp"

namespace slamko_vio {

class StereoMatcher {
 public:
  struct Config {
    int   patch_size    = 11;     // odd, in [7, 21]
    int   min_disparity = 0;
    int   max_disparity = 100;
    float ncc_threshold = 0.6f;   // accept match if NCC >= threshold
    int   border        = 8;      // require keypoint and search window inside this border
  };

  StereoMatcher(int width, int height, const Config& cfg);
  ~StereoMatcher();

  StereoMatcher(const StereoMatcher&) = delete;
  StereoMatcher& operator=(const StereoMatcher&) = delete;

  // Push the current stereo pair. Both images must be mono8, same size,
  // arbitrary row pitch (bytes).
  void set_images(const std::uint8_t* d_left_mono8, std::size_t left_pitch_bytes,
                  const std::uint8_t* d_right_mono8, std::size_t right_pitch_bytes);

  // For each left keypoint at left_xy[i] = (xL_i, yL_i), find the corresponding
  // right keypoint along the same scanline. On success write the sub-pixel
  // right_xy[i] and status[i] = 1. On rejection (NCC < threshold or out of
  // range) write status[i] = 0 and leave right_xy[i] = left_xy[i].
  void match(const float* d_left_xy, float* d_right_xy,
             std::int8_t* d_status, int n_features);

  int width()  const { return w_; }
  int height() const { return h_; }

 private:
  int w_;
  int h_;
  Config cfg_;
  float* d_left_f_  = nullptr;
  float* d_right_f_ = nullptr;
};

}  // namespace slamko_vio
