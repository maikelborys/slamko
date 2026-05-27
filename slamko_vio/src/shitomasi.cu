// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// CUDA Shi-Tomasi corner detector.

#include "slamko_vio/shitomasi.hpp"

#include <cuda_runtime.h>
#include <stdexcept>
#include <string>
#include <cstdio>
#include <algorithm>
#include <cmath>

namespace slamko_vio {

namespace {

#define KLT_VO_CUDA_CHECK(stmt)                                                  \
  do {                                                                            \
    cudaError_t _e = (stmt);                                                     \
    if (_e != cudaSuccess) {                                                     \
      throw std::runtime_error(std::string("CUDA error: ") +                     \
                               cudaGetErrorString(_e) + " at " __FILE__ ":" +    \
                               std::to_string(__LINE__));                        \
    }                                                                             \
  } while (0)

// ---------------------------------------------------------------------------
// Shi-Tomasi response kernel.
//
// For each pixel (x,y) inside the active border:
//   - Compute Sobel Ix, Iy for the 3x3 neighbourhood around (x,y).
//   - Accumulate sum_xx = Σ Ix*Ix, sum_yy = Σ Iy*Iy, sum_xy = Σ Ix*Iy
//     over that 3x3 patch.
//   - Output the minimum eigenvalue of the structure tensor.
//
// Pixels in the border get score 0 so they cannot win NMS.
// ---------------------------------------------------------------------------
__device__ __forceinline__ float sobel_ix(const std::uint8_t* img,
                                          int pitch, int x, int y) {
  // Sobel x kernel:  [-1 0 1; -2 0 2; -1 0 1] / 8
  const std::uint8_t* row0 = img + (y - 1) * pitch;
  const std::uint8_t* row1 = img + (y    ) * pitch;
  const std::uint8_t* row2 = img + (y + 1) * pitch;
  float s = (float)row0[x + 1] - (float)row0[x - 1]
          + 2.f * ((float)row1[x + 1] - (float)row1[x - 1])
          + (float)row2[x + 1] - (float)row2[x - 1];
  return s * (1.f / 8.f);
}

__device__ __forceinline__ float sobel_iy(const std::uint8_t* img,
                                          int pitch, int x, int y) {
  // Sobel y kernel:  [-1 -2 -1; 0 0 0; 1 2 1] / 8
  const std::uint8_t* row0 = img + (y - 1) * pitch;
  const std::uint8_t* row2 = img + (y + 1) * pitch;
  float s = (float)row2[x - 1] - (float)row0[x - 1]
          + 2.f * ((float)row2[x] - (float)row0[x])
          + (float)row2[x + 1] - (float)row0[x + 1];
  return s * (1.f / 8.f);
}

__global__ void shi_tomasi_score_kernel(
    const std::uint8_t* __restrict__ img,
    int   pitch_img,
    float* __restrict__ score,
    int   w, int h, int border) {
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;

  if (x < border || y < border || x >= w - border || y >= h - border) {
    score[y * w + x] = 0.f;
    return;
  }

  float sum_xx = 0.f, sum_yy = 0.f, sum_xy = 0.f;
  #pragma unroll
  for (int dy = -1; dy <= 1; ++dy) {
    #pragma unroll
    for (int dx = -1; dx <= 1; ++dx) {
      const float ix = sobel_ix(img, pitch_img, x + dx, y + dy);
      const float iy = sobel_iy(img, pitch_img, x + dx, y + dy);
      sum_xx += ix * ix;
      sum_yy += iy * iy;
      sum_xy += ix * iy;
    }
  }

  // Min eigenvalue of [[sum_xx, sum_xy], [sum_xy, sum_yy]]:
  //   lambda_min = ((sum_xx + sum_yy) - sqrt((sum_xx - sum_yy)^2 + 4*sum_xy^2)) / 2
  const float tr_half = 0.5f * (sum_xx + sum_yy);
  const float disc    = 0.5f * sqrtf((sum_xx - sum_yy) * (sum_xx - sum_yy)
                                     + 4.f * sum_xy * sum_xy);
  const float min_eig = tr_half - disc;
  score[y * w + x] = fmaxf(min_eig, 0.f);
}

// ---------------------------------------------------------------------------
// NMS + threshold + emit kernel. 3x3 strict local maximum.
//   - Reject if score < min_quality.
//   - Reject if any of the 8 neighbours has strictly greater score.
//   - On a tie, the lower-(y, x) cell wins (deterministic order).
// Emit (x_sub, y_sub, score) into d_kps_ + d_scores_ via atomicAdd on d_count_.
//
// Sub-pixel refinement via quadratic fit:
//   along x: dx = (S(x-1, y) - S(x+1, y)) / (2 * (S(x-1, y) + S(x+1, y) - 2*S(x, y)))
//   along y: same with y axis.
// ---------------------------------------------------------------------------
__global__ void nms_emit_kernel(
    const float* __restrict__ score,
    int   w, int h, int border, int nms_r,
    float min_quality, int max_corners,
    int grid_cols, int grid_rows, int k_per_cell,
    int* __restrict__ d_cell_count,   // grid_cols * grid_rows (may be null)
    float* __restrict__ d_kps,        // 2 * max_corners
    float* __restrict__ d_kp_sc,      // max_corners
    int*   __restrict__ d_count) {
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x < border + nms_r || y < border + nms_r ||
      x >= w - border - nms_r || y >= h - border - nms_r) return;

  const float s = score[y * w + x];
  if (s < min_quality) return;

  for (int dy = -nms_r; dy <= nms_r; ++dy) {
    for (int dx = -nms_r; dx <= nms_r; ++dx) {
      if (dx == 0 && dy == 0) continue;
      const float n = score[(y + dy) * w + (x + dx)];
      if (n > s) return;
      if (n == s && (dy < 0 || (dy == 0 && dx < 0))) return;
    }
  }

  // Grid quota — first k_per_cell candidates per cell (atomic-ordered, so
  // not strictly top-K-by-score, but uniformly spread spatially).
  if (grid_cols > 0 && d_cell_count != nullptr) {
    const float cw = (float)w / (float)grid_cols;
    const float ch = (float)h / (float)grid_rows;
    int cx = (int)((float)x / cw);
    int cy = (int)((float)y / ch);
    if (cx >= grid_cols) cx = grid_cols - 1;
    if (cy >= grid_rows) cy = grid_rows - 1;
    const int cell_idx = cy * grid_cols + cx;
    const int rank = atomicAdd(&d_cell_count[cell_idx], 1);
    if (rank >= k_per_cell) return;
  }

  // Sub-pixel quadratic refinement (Taylor 2nd-order along x then y).
  const float sxm = score[y * w + (x - 1)];
  const float sxp = score[y * w + (x + 1)];
  const float sym = score[(y - 1) * w + x];
  const float syp = score[(y + 1) * w + x];
  const float denx = (sxm + sxp - 2.f * s);
  const float deny = (sym + syp - 2.f * s);
  float fx = 0.f, fy = 0.f;
  if (fabsf(denx) > 1e-12f) fx = 0.5f * (sxm - sxp) / denx;
  if (fabsf(deny) > 1e-12f) fy = 0.5f * (sym - syp) / deny;
  fx = fmaxf(-0.5f, fminf(0.5f, fx));
  fy = fmaxf(-0.5f, fminf(0.5f, fy));

  const int idx = atomicAdd(d_count, 1);
  if (idx >= max_corners) return;

  d_kps[2 * idx + 0] = (float)x + fx;
  d_kps[2 * idx + 1] = (float)y + fy;
  d_kp_sc[idx]      = s;
}

}  // namespace

// ============================================================================
// Detector lifetime
// ============================================================================

ShiTomasiDetector::ShiTomasiDetector(int width, int height, const Config& cfg)
    : w_(width), h_(height), cfg_(cfg) {
  if (cfg_.max_corners <= 0)
    throw std::invalid_argument("ShiTomasiDetector: max_corners must be > 0");
  KLT_VO_CUDA_CHECK(cudaMalloc(&d_score_, sizeof(float) * w_ * h_));
  KLT_VO_CUDA_CHECK(cudaMalloc(&d_kps_,   sizeof(float) * 2 * cfg_.max_corners));
  KLT_VO_CUDA_CHECK(cudaMalloc(&d_kp_sc_, sizeof(float) * cfg_.max_corners));
  KLT_VO_CUDA_CHECK(cudaMalloc(&d_count_, sizeof(int)));
  if (cfg_.grid_cols > 0 && cfg_.grid_rows > 0) {
    KLT_VO_CUDA_CHECK(cudaMalloc(&d_cell_count_,
                                 sizeof(int) * cfg_.grid_cols * cfg_.grid_rows));
  }
}

ShiTomasiDetector::~ShiTomasiDetector() {
  if (d_ix_)         cudaFree(d_ix_);
  if (d_iy_)         cudaFree(d_iy_);
  if (d_score_)      cudaFree(d_score_);
  if (d_kps_)        cudaFree(d_kps_);
  if (d_kp_sc_)      cudaFree(d_kp_sc_);
  if (d_count_)      cudaFree(d_count_);
  if (d_cell_count_) cudaFree(d_cell_count_);
}

int ShiTomasiDetector::detect(const std::uint8_t* d_image_mono8,
                              std::size_t pitch_bytes) {
  KLT_VO_CUDA_CHECK(cudaMemsetAsync(d_count_, 0, sizeof(int)));
  if (d_cell_count_) {
    KLT_VO_CUDA_CHECK(cudaMemsetAsync(d_cell_count_, 0,
                          sizeof(int) * cfg_.grid_cols * cfg_.grid_rows));
  }

  const dim3 block(16, 16);
  const dim3 grid((w_ + block.x - 1) / block.x,
                  (h_ + block.y - 1) / block.y);

  shi_tomasi_score_kernel<<<grid, block>>>(
      d_image_mono8, (int)pitch_bytes, d_score_, w_, h_, cfg_.border);

  nms_emit_kernel<<<grid, block>>>(
      d_score_, w_, h_, cfg_.border, cfg_.nms_radius,
      cfg_.min_quality, cfg_.max_corners,
      cfg_.grid_cols, cfg_.grid_rows, cfg_.k_per_cell,
      d_cell_count_, d_kps_, d_kp_sc_, d_count_);

  int n = 0;
  KLT_VO_CUDA_CHECK(cudaMemcpy(&n, d_count_, sizeof(int), cudaMemcpyDeviceToHost));
  last_count_ = std::min(n, cfg_.max_corners);
  return last_count_;
}

std::vector<Keypoint> ShiTomasiDetector::get_keypoints() const {
  std::vector<Keypoint> out(last_count_);
  if (last_count_ == 0) return out;

  std::vector<float> xy(2 * last_count_);
  std::vector<float> sc(last_count_);
  KLT_VO_CUDA_CHECK(cudaMemcpy(xy.data(), d_kps_,
                               sizeof(float) * 2 * last_count_,
                               cudaMemcpyDeviceToHost));
  KLT_VO_CUDA_CHECK(cudaMemcpy(sc.data(), d_kp_sc_,
                               sizeof(float) * last_count_,
                               cudaMemcpyDeviceToHost));
  for (int i = 0; i < last_count_; ++i) {
    out[i] = Keypoint{xy[2 * i + 0], xy[2 * i + 1], sc[i]};
  }
  return out;
}

const float* ShiTomasiDetector::device_keypoints() const { return d_kps_; }
const float* ShiTomasiDetector::device_scores()    const { return d_kp_sc_; }
int          ShiTomasiDetector::count()            const { return last_count_; }

}  // namespace slamko_vio
