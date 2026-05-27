// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Pyramidal Lucas-Kanade tracker (Bouguet 1999), CUDA implementation.
//
// Inverse-compositional formulation: at each pyramid level, the patch
// gradients and Hessian are computed once from the *previous* frame; the
// per-iteration cost is then just a warp + subtract + reduction.
//
// Each tracker instance owns two pitched 32F image pyramids (current and
// previous). swap_pyramids() rotates them at frame boundary.
//
// Patch size & pyramid levels are compile-time tunable via Config; defaults
// (9x9, 4 levels) are chosen to match cuVSLAM's apparent setup.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "slamko_vio/types.hpp"

namespace slamko_vio {

class KltTracker {
 public:
  struct Config {
    int   pyramid_levels = 4;
    int   patch_size     = 9;      // must be odd, in [5, 15]
    int   max_iterations = 10;
    float epsilon        = 0.03f;  // convergence tol on per-iter displacement
    float min_eig        = 1.0e-4f;  // reject patches with degenerate Hessian
    // Reject patches whose final RMSE (per-pixel) exceeds this. <= 0 disables.
    float max_residual   = 0.0f;
    // After per-level convergence, require zero-mean NCC between template
    // and warped current patch >= this threshold. <= 0 disables. 0.5 keeps
    // moderately-textured tracks while still rejecting catastrophic drift;
    // matches the per-level filter cuVSLAM describes (their actual threshold
    // is not published).
    float ncc_threshold  = 0.5f;
  };

  KltTracker(int width, int height, const Config& cfg);
  ~KltTracker();

  KltTracker(const KltTracker&) = delete;
  KltTracker& operator=(const KltTracker&) = delete;

  // Push a new mono8 image. Builds the pyramid into the "current" slot.
  // Pitch is in bytes.
  void set_image(const std::uint8_t* d_image_mono8, std::size_t pitch_bytes);

  // Track n features from previous frame to current frame.
  //   prev_xy:  device, 2n floats (x0,y0,x1,y1,...) — input positions in the
  //             previous frame
  //   curr_xy:  device, 2n floats — output positions in the current frame
  //             (initialised in-place by the kernel from prev_xy)
  //   status:   device, n int8 — TrackStatus per feature
  //
  // After tracking, swap_pyramids() promotes current → previous so next
  // set_image() builds into the freed slot.
  void track(const float* prev_xy, float* curr_xy, std::int8_t* status, int n);

  // Rotate roles. Call at the end of each frame, after track().
  void swap_pyramids();

  int width()  const { return w_; }
  int height() const { return h_; }
  int levels() const { return cfg_.pyramid_levels; }

 private:
  int w_;
  int h_;
  Config cfg_;

  // [level] entries; 0 = full resolution
  std::vector<float*>      d_prev_pyr_;
  std::vector<float*>      d_curr_pyr_;
  std::vector<std::size_t> pitch_;
  std::vector<int>         lvl_w_;
  std::vector<int>         lvl_h_;

  bool prev_valid_ = false;
};

}  // namespace slamko_vio
