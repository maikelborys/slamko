// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Pyramidal Lucas-Kanade tracker (Bouguet 1999), CUDA implementation.

#include "slamko_vio/klt_tracker.hpp"

#include <cuda_runtime.h>
#include <stdexcept>
#include <string>
#include <cstdio>

namespace slamko_vio {

namespace {

constexpr int kMaxLevels = 6;

#define KLT_VO_CUDA_CHECK(stmt)                                                  \
  do {                                                                            \
    cudaError_t _e = (stmt);                                                     \
    if (_e != cudaSuccess) {                                                     \
      throw std::runtime_error(std::string("CUDA error: ") +                     \
                               cudaGetErrorString(_e) + " at " __FILE__ ":" +    \
                               std::to_string(__LINE__));                        \
    }                                                                             \
  } while (0)

struct LevelPtrs {
  const float* p[kMaxLevels];
  int w[kMaxLevels];
  int h[kMaxLevels];
};

// ---------------------------------------------------------------------------
// Image conversion + pyramid build kernels
// ---------------------------------------------------------------------------
__global__ void u8_to_f32_kernel(
    const std::uint8_t* __restrict__ src, int src_pitch_bytes,
    float* __restrict__ dst, int W, int H) {
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= W || y >= H) return;
  dst[y * W + x] = (float)src[y * src_pitch_bytes + x];
}

// Anti-aliased 2x decimation: 5x5 binomial filter [1 4 6 4 1] / 16 (separable)
// then drop every other sample. Equivalent to a small Gaussian low-pass before
// decimation, which is what Bouguet's KLT paper recommends.
__global__ void downsample_2x_kernel(
    const float* __restrict__ src, int sW, int sH,
    float* __restrict__ dst, int dW, int dH) {
  const int dx = blockIdx.x * blockDim.x + threadIdx.x;
  const int dy = blockIdx.y * blockDim.y + threadIdx.y;
  if (dx >= dW || dy >= dH) return;
  const int sx = dx * 2;
  const int sy = dy * 2;

  auto S = [src, sW, sH] __device__ (int x, int y) -> float {
    if (x < 0) x = 0; else if (x > sW - 1) x = sW - 1;
    if (y < 0) y = 0; else if (y > sH - 1) y = sH - 1;
    return src[y * sW + x];
  };

  const float r0 = S(sx-2, sy-2) + 4.f*S(sx-1, sy-2) + 6.f*S(sx, sy-2)
                 + 4.f*S(sx+1, sy-2) + S(sx+2, sy-2);
  const float r1 = S(sx-2, sy-1) + 4.f*S(sx-1, sy-1) + 6.f*S(sx, sy-1)
                 + 4.f*S(sx+1, sy-1) + S(sx+2, sy-1);
  const float r2 = S(sx-2, sy  ) + 4.f*S(sx-1, sy  ) + 6.f*S(sx, sy  )
                 + 4.f*S(sx+1, sy  ) + S(sx+2, sy  );
  const float r3 = S(sx-2, sy+1) + 4.f*S(sx-1, sy+1) + 6.f*S(sx, sy+1)
                 + 4.f*S(sx+1, sy+1) + S(sx+2, sy+1);
  const float r4 = S(sx-2, sy+2) + 4.f*S(sx-1, sy+2) + 6.f*S(sx, sy+2)
                 + 4.f*S(sx+1, sy+2) + S(sx+2, sy+2);
  dst[dy * dW + dx] = (r0 + 4.f*r1 + 6.f*r2 + 4.f*r3 + r4) * (1.f / 256.f);
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

// ---------------------------------------------------------------------------
// KLT track kernel: one warp per feature, all pyramid levels in one launch.
// Inverse-compositional Lucas-Kanade (Baker-Matthews 2004) — gradients and
// Hessian are computed from the TEMPLATE patch (previous image) and reused
// across iterations. For pure translation the update is W <- W o (-Δp), so
// curr_x_L -= du.
//
// At the end of each level (after the iterations converge), we run an
// NCC (zero-mean normalised cross-correlation) check between template and
// warped current patch. Tracks below ncc_thr are dropped. This matches
// the per-level NCC filter the cuVSLAM paper describes.
// ---------------------------------------------------------------------------
__global__ void klt_track_kernel(
    LevelPtrs prev, LevelPtrs curr, int n_levels,
    int patch_half, int max_iter, float eps, float min_eig, float max_residual,
    float ncc_thr,
    const float* __restrict__ prev_xy,
    float* __restrict__ curr_xy,
    std::int8_t* __restrict__ status,
    int n_features) {
  const int feat = blockIdx.x;
  if (feat >= n_features) return;
  const int lane = threadIdx.x;          // 0 .. 31
  const unsigned mask = 0xFFFFFFFFu;
  const int patch = 2 * patch_half + 1;
  const int patch_sz = patch * patch;

  const float px_full = prev_xy[2 * feat + 0];
  const float py_full = prev_xy[2 * feat + 1];

  bool ok = true;
  float u = 0.f, v = 0.f;                // displacement in level-L coords
  float final_x = px_full, final_y = py_full;

  for (int L = n_levels - 1; L >= 0; --L) {
    const float scale = 1.f / (float)(1 << L);
    const float prev_x_L = px_full * scale;
    const float prev_y_L = py_full * scale;
    float curr_x_L = prev_x_L + u;
    float curr_y_L = prev_y_L + v;

    const int   W       = prev.w[L];
    const int   H       = prev.h[L];
    const float* pr_img = prev.p[L];
    const float* cu_img = curr.p[L];

    const float border = (float)patch_half + 1.f;
    if (prev_x_L < border || prev_y_L < border ||
        prev_x_L >= (float)W - border || prev_y_L >= (float)H - border) {
      ok = false; break;
    }

    // -------- pre-compute template Hessian H_T and gradients (constant) --
    float A11 = 0.f, A22 = 0.f, A12 = 0.f;
    for (int p = lane; p < patch_sz; p += 32) {
      const int dx = (p % patch) - patch_half;
      const int dy = (p / patch) - patch_half;
      const float spx = prev_x_L + (float)dx;
      const float spy = prev_y_L + (float)dy;
      const float ix_t = 0.5f *
          (bilinear_sample(pr_img, W, H, spx + 1.f, spy)
         - bilinear_sample(pr_img, W, H, spx - 1.f, spy));
      const float iy_t = 0.5f *
          (bilinear_sample(pr_img, W, H, spx, spy + 1.f)
         - bilinear_sample(pr_img, W, H, spx, spy - 1.f));
      A11 += ix_t * ix_t;
      A22 += iy_t * iy_t;
      A12 += ix_t * iy_t;
    }
    #pragma unroll
    for (int s = 16; s > 0; s >>= 1) {
      A11 += __shfl_xor_sync(mask, A11, s);
      A22 += __shfl_xor_sync(mask, A22, s);
      A12 += __shfl_xor_sync(mask, A12, s);
    }
    const float det        = A11 * A22 - A12 * A12;
    const float trace_half = 0.5f * (A11 + A22);
    const float disc       = sqrtf(fmaxf(trace_half * trace_half - det, 0.f));
    const float lmin       = trace_half - disc;
    if (det < 1e-12f || lmin < min_eig) { ok = false; break; }

    // -------- inner LK iterations (H is fixed; only b changes) -----------
    for (int iter = 0; iter < max_iter; ++iter) {
      if (curr_x_L < border || curr_y_L < border ||
          curr_x_L >= (float)W - border || curr_y_L >= (float)H - border) {
        ok = false; break;
      }
      float b1 = 0.f, b2 = 0.f;
      for (int p = lane; p < patch_sz; p += 32) {
        const int dx = (p % patch) - patch_half;
        const int dy = (p / patch) - patch_half;
        const float spx = prev_x_L + (float)dx;
        const float spy = prev_y_L + (float)dy;
        const float scx = curr_x_L + (float)dx;
        const float scy = curr_y_L + (float)dy;
        const float pv = bilinear_sample(pr_img, W, H, spx, spy);
        const float cv = bilinear_sample(cu_img, W, H, scx, scy);
        // Template gradient at the template pixel (constant across iters but
        // recomputed here to keep register pressure low; cheap from L2).
        const float ix_t = 0.5f *
            (bilinear_sample(pr_img, W, H, spx + 1.f, spy)
           - bilinear_sample(pr_img, W, H, spx - 1.f, spy));
        const float iy_t = 0.5f *
            (bilinear_sample(pr_img, W, H, spx, spy + 1.f)
           - bilinear_sample(pr_img, W, H, spx, spy - 1.f));
        const float r = cv - pv;            // IC: I - T
        b1 += ix_t * r;
        b2 += iy_t * r;
      }
      #pragma unroll
      for (int s = 16; s > 0; s >>= 1) {
        b1 += __shfl_xor_sync(mask, b1, s);
        b2 += __shfl_xor_sync(mask, b2, s);
      }
      const float du = ( A22 * b1 - A12 * b2) / det;
      const float dv = (-A12 * b1 + A11 * b2) / det;
      // Inverse compositional update for pure translation: p <- p - Δp.
      curr_x_L -= du;
      curr_y_L -= dv;
      if (du * du + dv * dv < eps * eps) break;
    }
    if (!ok) break;

    // -------- NCC validation (zero-mean normalised cross-correlation) ----
    if (ncc_thr > 0.f) {
      if (curr_x_L < border || curr_y_L < border ||
          curr_x_L >= (float)W - border || curr_y_L >= (float)H - border) {
        ok = false; break;
      }
      float sp = 0.f, sc = 0.f;       // sums for means
      float spp = 0.f, scc = 0.f, spc = 0.f;
      for (int p = lane; p < patch_sz; p += 32) {
        const int dx = (p % patch) - patch_half;
        const int dy = (p / patch) - patch_half;
        const float pv = bilinear_sample(pr_img, W, H,
                                         prev_x_L + (float)dx,
                                         prev_y_L + (float)dy);
        const float cv = bilinear_sample(cu_img, W, H,
                                         curr_x_L + (float)dx,
                                         curr_y_L + (float)dy);
        sp += pv;  sc += cv;
        spp += pv * pv;  scc += cv * cv;  spc += pv * cv;
      }
      #pragma unroll
      for (int s = 16; s > 0; s >>= 1) {
        sp  += __shfl_xor_sync(mask, sp,  s);
        sc  += __shfl_xor_sync(mask, sc,  s);
        spp += __shfl_xor_sync(mask, spp, s);
        scc += __shfl_xor_sync(mask, scc, s);
        spc += __shfl_xor_sync(mask, spc, s);
      }
      const float n = (float)patch_sz;
      const float mp = sp / n;
      const float mc = sc / n;
      const float varp = spp - n * mp * mp;
      const float varc = scc - n * mc * mc;
      const float cov  = spc - n * mp * mc;
      const float denom = sqrtf(fmaxf(varp * varc, 1.0e-12f));
      const float ncc = cov / denom;
      if (ncc < ncc_thr) { ok = false; break; }
    }
    (void)max_residual;

    u = curr_x_L - prev_x_L;
    v = curr_y_L - prev_y_L;
    if (L > 0) { u *= 2.f; v *= 2.f; }
    else       { final_x = px_full + u; final_y = py_full + v; }
  }

  if (lane == 0) {
    if (ok) {
      curr_xy[2 * feat + 0] = final_x;
      curr_xy[2 * feat + 1] = final_y;
      status[feat] = (std::int8_t)1;
    } else {
      curr_xy[2 * feat + 0] = px_full;
      curr_xy[2 * feat + 1] = py_full;
      status[feat] = (std::int8_t)0;
    }
  }
}

}  // namespace

// ============================================================================
// Tracker lifetime
// ============================================================================

KltTracker::KltTracker(int width, int height, const Config& cfg)
    : w_(width), h_(height), cfg_(cfg) {
  if (cfg_.pyramid_levels < 1 || cfg_.pyramid_levels > kMaxLevels)
    throw std::invalid_argument("KltTracker: pyramid_levels out of range");
  if ((cfg_.patch_size & 1) == 0 || cfg_.patch_size < 5 || cfg_.patch_size > 15)
    throw std::invalid_argument("KltTracker: patch_size must be odd in [5,15]");

  d_prev_pyr_.resize(cfg_.pyramid_levels, nullptr);
  d_curr_pyr_.resize(cfg_.pyramid_levels, nullptr);
  pitch_.resize(cfg_.pyramid_levels, 0);
  lvl_w_.resize(cfg_.pyramid_levels);
  lvl_h_.resize(cfg_.pyramid_levels);

  int cw = w_, ch = h_;
  for (int L = 0; L < cfg_.pyramid_levels; ++L) {
    lvl_w_[L] = cw;
    lvl_h_[L] = ch;
    KLT_VO_CUDA_CHECK(cudaMalloc(&d_prev_pyr_[L], sizeof(float) * cw * ch));
    KLT_VO_CUDA_CHECK(cudaMalloc(&d_curr_pyr_[L], sizeof(float) * cw * ch));
    pitch_[L] = sizeof(float) * cw;
    cw = (cw + 1) / 2;
    ch = (ch + 1) / 2;
  }
}

KltTracker::~KltTracker() {
  for (auto* p : d_prev_pyr_) if (p) cudaFree(p);
  for (auto* p : d_curr_pyr_) if (p) cudaFree(p);
}

void KltTracker::set_image(const std::uint8_t* d_image_mono8,
                           std::size_t pitch_bytes) {
  // L0: u8 → f32 in-place.
  {
    const dim3 block(16, 16);
    const dim3 grid((w_ + block.x - 1) / block.x,
                    (h_ + block.y - 1) / block.y);
    u8_to_f32_kernel<<<grid, block>>>(d_image_mono8, (int)pitch_bytes,
                                      d_curr_pyr_[0], w_, h_);
  }
  // L1..N-1: downsample-by-2 with a simple 2x2 average.
  for (int L = 1; L < cfg_.pyramid_levels; ++L) {
    const dim3 block(16, 16);
    const dim3 grid((lvl_w_[L] + block.x - 1) / block.x,
                    (lvl_h_[L] + block.y - 1) / block.y);
    downsample_2x_kernel<<<grid, block>>>(d_curr_pyr_[L - 1],
                                          lvl_w_[L - 1], lvl_h_[L - 1],
                                          d_curr_pyr_[L], lvl_w_[L], lvl_h_[L]);
  }
}

void KltTracker::track(const float* prev_xy, float* curr_xy,
                       std::int8_t* status, int n) {
  if (!prev_valid_) {
    // First frame: no previous pyramid yet. Mark all as lost.
    KLT_VO_CUDA_CHECK(cudaMemsetAsync(status, 0, sizeof(std::int8_t) * n));
    KLT_VO_CUDA_CHECK(cudaMemcpyAsync(curr_xy, prev_xy, sizeof(float) * 2 * n,
                                      cudaMemcpyDeviceToDevice));
    return;
  }

  LevelPtrs prev{}, curr{};
  for (int L = 0; L < cfg_.pyramid_levels; ++L) {
    prev.p[L] = d_prev_pyr_[L];
    curr.p[L] = d_curr_pyr_[L];
    prev.w[L] = lvl_w_[L];
    prev.h[L] = lvl_h_[L];
    curr.w[L] = lvl_w_[L];
    curr.h[L] = lvl_h_[L];
  }

  const int patch_half = cfg_.patch_size / 2;
  const dim3 block(32, 1);
  const dim3 grid(n, 1);
  klt_track_kernel<<<grid, block>>>(
      prev, curr, cfg_.pyramid_levels, patch_half,
      cfg_.max_iterations, cfg_.epsilon, cfg_.min_eig, cfg_.max_residual,
      cfg_.ncc_threshold,
      prev_xy, curr_xy, status, n);
}

void KltTracker::swap_pyramids() {
  d_prev_pyr_.swap(d_curr_pyr_);
  prev_valid_ = true;
}

}  // namespace slamko_vio
