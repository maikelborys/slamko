// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// CUDA Shi-Tomasi corner detector for mono8 images.
//
// Pipeline (single kernel chain on default stream):
//   Sobel 3x3   -> Ix, Iy                                (float)
//   3x3 box     -> Ixx, Iyy, Ixy                         (float)
//   min eig     -> score = 0.5*(Ixx+Iyy - sqrt(...))     (float)
//   3x3 NMS     -> non-max suppression
//   threshold   -> emit candidates atomically into d_kps_
//   topK select -> simple radix / partial sort host-side (sprint 1 ok)
//   sub-pixel   -> quadratic fit of score around peak    (added in same kernel)
//
// All buffers are owned by the detector. The caller passes a device pointer
// to the input mono8 image with a row pitch (bytes). Output keypoints live
// on device; copy_keypoints() pulls them to host.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "slamko_vio/types.hpp"

namespace slamko_vio {

class ShiTomasiDetector {
 public:
  struct Config {
    int   max_corners  = 1500;     // upper bound on output
    float min_quality  = 1.0e-4f;  // min Shi-Tomasi score (relative to local image scale)
    int   nms_radius   = 3;        // pixel radius for non-max suppression
    int   border       = 8;        // ignore corners closer than this to image edge
    // Grid-based detection — image is divided into grid_cols × grid_rows
    // non-overlapping cells; each cell admits up to k_per_cell corners. This
    // is what the cuVSLAM paper uses to enforce spatial coverage. Set
    // grid_cols == 0 to disable and fall back to a single global pool.
    int   grid_cols    = 8;
    int   grid_rows    = 6;
    int   k_per_cell   = 32;
  };

  ShiTomasiDetector(int width, int height, const Config& cfg);
  ~ShiTomasiDetector();

  ShiTomasiDetector(const ShiTomasiDetector&) = delete;
  ShiTomasiDetector& operator=(const ShiTomasiDetector&) = delete;

  // Detect corners on a mono8 device image. Returns the number of corners
  // emitted (clamped to max_corners). Output lives on device; call
  // copy_keypoints() or device_keypoints() to access.
  int detect(const std::uint8_t* d_image_mono8, std::size_t pitch_bytes);

  // Copy detected corners to host (allocates / resizes out).
  std::vector<Keypoint> get_keypoints() const;

  // Direct access for in-pipeline use (e.g. feed straight into KLT). Layout:
  //   d_keypoints():  float2 array [x, y, x, y, ...]
  //   d_scores():     float  array [score, score, ...]
  //   count():        most recently detected number
  const float*  device_keypoints() const;  // 2N floats: x0,y0,x1,y1,...
  const float*  device_scores()    const;  // N floats
  int           count()            const;

  int width()  const { return w_; }
  int height() const { return h_; }

 private:
  int w_;
  int h_;
  Config cfg_;
  int last_count_ = 0;

  // device scratch
  float*  d_ix_     = nullptr;  // dI/dx
  float*  d_iy_     = nullptr;  // dI/dy
  float*  d_score_  = nullptr;  // Shi-Tomasi min eigenvalue per pixel
  float*  d_kps_    = nullptr;  // 2 * max_corners floats
  float*  d_kp_sc_  = nullptr;  // max_corners floats
  int*    d_count_  = nullptr;  // 1 int
  int*    d_cell_count_ = nullptr;  // grid_cols * grid_rows ints (grid mode)
};

}  // namespace slamko_vio
