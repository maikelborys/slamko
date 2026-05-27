// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// CUDA P3P + RANSAC: GPU replacement for cv::solvePnPRansac (Phase A of the
// slamko_vio CUDA acceleration roadmap).
//
// Architecture:
//   - Host generates n_hyp deterministic 3-point samples (random seed).
//   - One GPU kernel launches with `(n_hyp × max_candidates_per_sample)`
//     blocks. Each block holds one (R, t) candidate; threads cooperatively
//     reproject all N landmarks, count inliers, and reduce within the block.
//   - Host argmax over per-block inlier counts → best hypothesis.
//   - Second small kernel writes the per-point inlier mask for the winner.
//   - Ceres motion-only BA refine (CPU) still post-processes the inlier set.
//
// P3P math: slamko_vio::p3p::solve (Grunert algorithm via Sylvester-resultant
// quartic, validated by tests/test_p3p.cpp against synthetic ground truth).

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "slamko_vio/types.hpp"

namespace slamko_vio {

class PnPCuda {
 public:
  struct Config {
    int   max_ransac_iters          = 200;
    float reprojection_threshold_px = 0.8f;
    int   min_inliers               = 12;
  };

  PnPCuda(int max_points, const Config& cfg);
  ~PnPCuda();
  PnPCuda(const PnPCuda&) = delete;
  PnPCuda& operator=(const PnPCuda&) = delete;

  // Returns true on success; writes T_prev_to_cur and inlier_indices.
  // random_seed makes the host-side 3-point sampling deterministic.
  // Phase A only: GPU P3P+RANSAC. Caller runs Ceres LM refine on top.
  bool solve(const std::vector<Eigen::Vector3f>& landmarks_3d,
             const std::vector<Eigen::Vector2f>& observations_2d_left,
             const StereoIntrinsics& K,
             Eigen::Matrix4f& T_prev_to_cur,
             std::vector<int>& inlier_indices,
             std::uint64_t random_seed);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  Config cfg_;
};

}  // namespace slamko_vio
