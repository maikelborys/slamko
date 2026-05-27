// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys

#include "slamko_vio/stereo_matcher.hpp"

#include <cuda_runtime.h>
#include <stdexcept>
#include <string>

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

__global__ void u8_to_f32_kernel(
    const std::uint8_t* __restrict__ src, int src_pitch_bytes,
    float* __restrict__ dst, int W, int H) {
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= W || y >= H) return;
  dst[y * W + x] = (float)src[y * src_pitch_bytes + x];
}

__device__ __forceinline__ float bilinear_sample(
    const float* img, int W, int H, float x, float y) {
  int ix = (int)floorf(x);
  int iy = (int)floorf(y);
  const float fx = x - (float)ix;
  const float fy = y - (float)iy;
  if (ix < 0) ix = 0;
  if (iy < 0) iy = 0;
  if (ix > W - 2) ix = W - 2;
  if (iy > H - 2) iy = H - 2;
  const float v00 = img[iy * W + ix];
  const float v01 = img[iy * W + ix + 1];
  const float v10 = img[(iy + 1) * W + ix];
  const float v11 = img[(iy + 1) * W + ix + 1];
  return (1.f - fy) * ((1.f - fx) * v00 + fx * v01)
       + fy        * ((1.f - fx) * v10 + fx * v11);
}

// Compute zero-mean NCC of an 11×11 (or 2*patch_half+1) patch centred at
// (xL, yL) in the left image vs. the same-sized patch at (xR, yR) in the
// right image. Called with one warp; returns the result on every lane via
// warp shuffle.
__device__ __forceinline__ float warp_ncc(
    const float* left, const float* right, int W, int H,
    float xL, float yL, float xR, float yR,
    int patch_half, int lane) {
  const int patch = 2 * patch_half + 1;
  const int patch_sz = patch * patch;
  float sl=0.f, sr=0.f, sll=0.f, srr=0.f, slr=0.f;
  for (int p = lane; p < patch_sz; p += 32) {
    const int dx = (p % patch) - patch_half;
    const int dy = (p / patch) - patch_half;
    const float lv = bilinear_sample(left,  W, H, xL + (float)dx, yL + (float)dy);
    const float rv = bilinear_sample(right, W, H, xR + (float)dx, yR + (float)dy);
    sl += lv;  sr += rv;
    sll += lv * lv;  srr += rv * rv;  slr += lv * rv;
  }
  const unsigned m = 0xFFFFFFFFu;
  #pragma unroll
  for (int s = 16; s > 0; s >>= 1) {
    sl  += __shfl_xor_sync(m, sl,  s);
    sr  += __shfl_xor_sync(m, sr,  s);
    sll += __shfl_xor_sync(m, sll, s);
    srr += __shfl_xor_sync(m, srr, s);
    slr += __shfl_xor_sync(m, slr, s);
  }
  const float n = (float)patch_sz;
  const float ml = sl / n;
  const float mr = sr / n;
  const float varl = sll - n * ml * ml;
  const float varr = srr - n * mr * mr;
  const float cov  = slr - n * ml * mr;
  const float denom = sqrtf(fmaxf(varl * varr, 1.0e-12f));
  return cov / denom;
}

__global__ void stereo_match_kernel(
    const float* __restrict__ left, const float* __restrict__ right,
    int W, int H,
    int patch_half, int min_disp, int max_disp, float ncc_thr, int border,
    const float* __restrict__ left_xy,
    float*       __restrict__ right_xy,
    std::int8_t* __restrict__ status,
    int n_features) {
  const int feat = blockIdx.x;
  if (feat >= n_features) return;
  const int lane = threadIdx.x;

  const float xL = left_xy[2 * feat + 0];
  const float yL = left_xy[2 * feat + 1];

  // Bounds: need the patch entirely inside the right image at the worst-case
  // disparity, and inside the left image at the keypoint.
  const float fborder = (float)border + (float)patch_half;
  if (yL < fborder || yL >= (float)H - fborder ||
      xL < fborder + (float)max_disp || xL >= (float)W - fborder) {
    if (lane == 0) {
      status[feat] = 0;
      right_xy[2 * feat + 0] = xL;
      right_xy[2 * feat + 1] = yL;
    }
    return;
  }

  // Scan disparities, keep the best integer d.
  float best_ncc = -2.f;
  int   best_d   = -1;
  for (int d = min_disp; d <= max_disp; ++d) {
    const float ncc = warp_ncc(left, right, W, H,
                               xL, yL, xL - (float)d, yL,
                               patch_half, lane);
    if (ncc > best_ncc) { best_ncc = ncc; best_d = d; }
  }

  if (best_ncc < ncc_thr || best_d < min_disp || best_d > max_disp) {
    if (lane == 0) {
      status[feat] = 0;
      right_xy[2 * feat + 0] = xL;
      right_xy[2 * feat + 1] = yL;
    }
    return;
  }

  // Sub-pixel refinement: parabolic fit through (d-1, d, d+1).
  float frac = 0.f;
  if (best_d > min_disp && best_d < max_disp) {
    const float ncc_m1 = warp_ncc(left, right, W, H,
                                  xL, yL, xL - (float)(best_d - 1), yL,
                                  patch_half, lane);
    const float ncc_p1 = warp_ncc(left, right, W, H,
                                  xL, yL, xL - (float)(best_d + 1), yL,
                                  patch_half, lane);
    const float den = (ncc_m1 + ncc_p1 - 2.f * best_ncc);
    if (fabsf(den) > 1.0e-9f) frac = 0.5f * (ncc_m1 - ncc_p1) / den;
    if (frac < -0.5f) frac = -0.5f;
    if (frac >  0.5f) frac =  0.5f;
  }
  const float d_refined = (float)best_d + frac;

  if (lane == 0) {
    status[feat] = 1;
    right_xy[2 * feat + 0] = xL - d_refined;
    right_xy[2 * feat + 1] = yL;
  }
}

}  // namespace

// ============================================================================
// StereoMatcher lifetime
// ============================================================================

StereoMatcher::StereoMatcher(int width, int height, const Config& cfg)
    : w_(width), h_(height), cfg_(cfg) {
  if ((cfg_.patch_size & 1) == 0 || cfg_.patch_size < 7 || cfg_.patch_size > 21)
    throw std::invalid_argument("StereoMatcher: patch_size must be odd in [7,21]");
  if (cfg_.min_disparity < 0 || cfg_.max_disparity <= cfg_.min_disparity)
    throw std::invalid_argument("StereoMatcher: invalid disparity range");
  KLT_VO_CUDA_CHECK(cudaMalloc(&d_left_f_,  sizeof(float) * w_ * h_));
  KLT_VO_CUDA_CHECK(cudaMalloc(&d_right_f_, sizeof(float) * w_ * h_));
}

StereoMatcher::~StereoMatcher() {
  if (d_left_f_)  cudaFree(d_left_f_);
  if (d_right_f_) cudaFree(d_right_f_);
}

void StereoMatcher::set_images(
    const std::uint8_t* d_left, std::size_t left_pitch,
    const std::uint8_t* d_right, std::size_t right_pitch) {
  const dim3 block(16, 16);
  const dim3 grid((w_ + block.x - 1) / block.x,
                  (h_ + block.y - 1) / block.y);
  u8_to_f32_kernel<<<grid, block>>>(d_left,  (int)left_pitch,  d_left_f_,  w_, h_);
  u8_to_f32_kernel<<<grid, block>>>(d_right, (int)right_pitch, d_right_f_, w_, h_);
}

void StereoMatcher::match(const float* d_left_xy, float* d_right_xy,
                          std::int8_t* d_status, int n) {
  if (n <= 0) return;
  const int patch_half = cfg_.patch_size / 2;
  const dim3 block(32, 1);
  const dim3 grid(n, 1);
  stereo_match_kernel<<<grid, block>>>(
      d_left_f_, d_right_f_, w_, h_,
      patch_half, cfg_.min_disparity, cfg_.max_disparity,
      cfg_.ncc_threshold, cfg_.border,
      d_left_xy, d_right_xy, d_status, n);
}

}  // namespace slamko_vio
