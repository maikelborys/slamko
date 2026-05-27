// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Frame-to-frame pose estimation: PnP RANSAC for outlier rejection on the
// left camera, then a motion-only BA refine over both stereo cameras.
//
// Given 3D landmarks expressed in the previous-frame left-camera coordinates
// and their 2D observations in the current frame's left and right cameras,
// solve the 6-DoF rigid transform
//
//   p_cur = T_{prev->cur} * p_prev
//
// using OpenCV's EPnP + RANSAC (left-camera only — fast geometric outlier
// rejection), then refine the inlier set with a Ceres LM that emits both
// left and right reprojection residuals per landmark (cuVSLAM Eq. 4).
//
// For rectified stereo, the right-camera reprojection of a point (X, Y, Z) in
// the left camera frame is
//   u_r = fx (X - b) / Z + cx
//   v_r = fy  Y      / Z + cy
// where b = baseline_m. Observations whose right-pixel x is NaN are treated as
// "no stereo match this frame" and contribute only a left-camera residual.

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slamko_vio/types.hpp"

namespace slamko_vio {

class PnPCuda;

class PoseEstimator {
 public:
  struct Config {
    float  reprojection_threshold_px = 1.5f;
    int    max_ransac_iters          = 200;
    double ransac_confidence         = 0.999;
    int    min_inliers               = 12;
    bool   refine_lm                 = true;
    // R4: two-pass LM. After the first refine, re-prune outliers above
    // refine_pixel_threshold and refine again. 0 disables the second pass.
    float  refine_pixel_threshold    = 0.0f;
    // Phase A CUDA: swap cv::solvePnPRansac for slamko_vio::PnPCuda. Ceres
    // motion-only BA refine still runs on CPU. Default false until validated.
    bool   use_cuda_ransac           = false;
    int    cuda_max_points           = 4096;
    // Speed-mode controls. The CPU Ceres LM refine is the largest remaining
    // single cost (~2 ms with 10 iters). Reducing this gives near-linear fps
    // win at the cost of ATE precision.
    //   lm_max_iters       — max iterations for the (first) refine pass.
    //                        Default 5 is the empirical sweet spot: same
    //                        ATE as 10 on MH_01, *better* ATE on V1_03 plus
    //                        +73% fps. iters≥5 just wastes Ceres cycles
    //                        on convergence checks for an already-converged
    //                        solution.
    //   refine_second_pass — whether to run the R4 "re-prune + refine" pass.
    //                        Critical for ATE — turning this off destroys
    //                        MH_01 ATE from 0.07 to 0.35.
    //   lm_function_tol    — convergence tolerance (Ceres function_tolerance)
    int    lm_max_iters              = 5;
    bool   refine_second_pass        = true;
    double lm_function_tol           = 1.0e-6;
  };

  PoseEstimator();
  explicit PoseEstimator(const Config& cfg);
  ~PoseEstimator();

  // Solve T_{prev->cur} from corresponding (3D, 2D, 2D_right) triples.
  // observations_2d_right may be empty (falls back to left-only refine), or
  // may contain NaN-x entries for individual landmarks that lacked a
  // current-frame stereo match.
  bool solve(const std::vector<Eigen::Vector3f>& landmarks_3d,
             const std::vector<Eigen::Vector2f>& observations_2d_left,
             const std::vector<Eigen::Vector2f>& observations_2d_right,
             const StereoIntrinsics& K,
             Eigen::Matrix4f& T_prev_to_cur,
             std::vector<int>& inlier_indices,
             bool use_guess = false) const;

  // Backwards-compatible single-camera form (left observations only).
  bool solve(const std::vector<Eigen::Vector3f>& landmarks_3d,
             const std::vector<Eigen::Vector2f>& observations_2d_left,
             const StereoIntrinsics& K,
             Eigen::Matrix4f& T_prev_to_cur,
             std::vector<int>& inlier_indices,
             bool use_guess = false) const {
    return solve(landmarks_3d, observations_2d_left, {},
                 K, T_prev_to_cur, inlier_indices, use_guess);
  }

 private:
  Config cfg_;
  std::unique_ptr<PnPCuda> pnp_cuda_;
};

}  // namespace slamko_vio
