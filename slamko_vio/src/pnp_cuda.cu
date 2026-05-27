// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// GPU implementation of slamko_vio::PnPCuda. See header for the architecture
// summary. P3P math lives in slamko_vio/p3p_solver.hpp (validated by test_p3p).

#include "slamko_vio/pnp_cuda.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include <cuda_runtime.h>

#include "slamko_vio/p3p_solver.hpp"

namespace slamko_vio {

namespace {

// Per-block warp/block int reduction. bsz must be a multiple of 32 and ≤ 1024.
__device__ inline int block_reduce_add(int v, int tid, int bsz) {
  for (int off = 16; off > 0; off >>= 1) v += __shfl_down_sync(0xffffffff, v, off);
  __shared__ int warp_sums[32];
  const int lane = tid & 31;
  const int warp = tid >> 5;
  if (lane == 0) warp_sums[warp] = v;
  __syncthreads();
  if (warp == 0) {
    int total = (lane < (bsz + 31) / 32) ? warp_sums[lane] : 0;
    for (int off = 16; off > 0; off >>= 1) total += __shfl_down_sync(0xffffffff, total, off);
    if (lane == 0) warp_sums[0] = total;
  }
  __syncthreads();
  return warp_sums[0];
}

// One block per RANSAC hypothesis. Block solves P3P on the 3-sample, scores
// each of up to 4 candidate poses, and writes the best (R, t, inlier_count)
// to per-block global slots.
__global__ void solve_and_score_kernel(
    const int* __restrict__ d_samples,  // 3 * n_hyp
    const float* __restrict__ d_X,       // 3 * N
    const float* __restrict__ d_uv,      // 2 * N
    int N,
    float fx, float fy, float cx, float cy,
    float thr_px,
    int* __restrict__ out_score,         // n_hyp
    float* __restrict__ out_R,           // 9 * n_hyp
    float* __restrict__ out_t) {         // 3 * n_hyp
  const int hyp = blockIdx.x;
  const int tid = threadIdx.x;
  const int bsz = blockDim.x;

  __shared__ int idx[3];
  __shared__ double Pw[3][3];
  __shared__ double fb[3][3];
  __shared__ int    n_sol;
  __shared__ double R_cand[4][9];
  __shared__ double t_cand[4][3];
  __shared__ int    best_score;
  __shared__ int    best_cand;
  __shared__ float  R_f[9];
  __shared__ float  t_f[3];

  if (tid < 3) idx[tid] = d_samples[hyp*3 + tid];
  __syncthreads();
  // Build Pw and bearings on threads 0..2.
  if (tid < 3) {
    const int pid = idx[tid];
    Pw[tid][0] = (double)d_X[3*pid + 0];
    Pw[tid][1] = (double)d_X[3*pid + 1];
    Pw[tid][2] = (double)d_X[3*pid + 2];
    const double xn = (d_uv[2*pid + 0] - cx) / fx;
    const double yn = (d_uv[2*pid + 1] - cy) / fy;
    const double nr = sqrt(xn*xn + yn*yn + 1.0);
    fb[tid][0] = xn / nr;
    fb[tid][1] = yn / nr;
    fb[tid][2] = 1.0 / nr;
  }
  if (tid == 0) {
    best_score = -1;
    best_cand  = -1;
  }
  __syncthreads();

  // Solve P3P on thread 0.
  if (tid == 0) {
    n_sol = slamko_vio::p3p::solve(Pw, fb, R_cand, t_cand);
  }
  __syncthreads();

  if (n_sol <= 0) {
    if (tid == 0) {
      out_score[hyp] = 0;
      for (int j = 0; j < 9; ++j) out_R[hyp*9 + j] = 0.f;
      for (int j = 0; j < 3; ++j) out_t[hyp*3 + j] = 0.f;
    }
    return;
  }

  const float thr2 = thr_px * thr_px;
  for (int s = 0; s < n_sol; ++s) {
    if (tid < 9) R_f[tid] = (float)R_cand[s][tid];
    if (tid < 3) t_f[tid] = (float)t_cand[s][tid];
    __syncthreads();

    int local = 0;
    for (int i = tid; i < N; i += bsz) {
      const float Xx = d_X[3*i + 0];
      const float Xy = d_X[3*i + 1];
      const float Xz = d_X[3*i + 2];
      const float px = R_f[0]*Xx + R_f[1]*Xy + R_f[2]*Xz + t_f[0];
      const float py = R_f[3]*Xx + R_f[4]*Xy + R_f[5]*Xz + t_f[1];
      const float pz = R_f[6]*Xx + R_f[7]*Xy + R_f[8]*Xz + t_f[2];
      if (pz < 1e-3f) continue;
      const float inv_z = 1.f / pz;
      const float u_p = fx * px * inv_z + cx;
      const float v_p = fy * py * inv_z + cy;
      const float du = u_p - d_uv[2*i + 0];
      const float dv = v_p - d_uv[2*i + 1];
      if (du*du + dv*dv <= thr2) ++local;
    }
    const int total = block_reduce_add(local, tid, bsz);
    if (tid == 0 && total > best_score) {
      best_score = total;
      best_cand  = s;
    }
    __syncthreads();
  }

  if (tid == 0) {
    out_score[hyp] = best_score;
    if (best_cand >= 0) {
      for (int j = 0; j < 9; ++j) out_R[hyp*9 + j] = (float)R_cand[best_cand][j];
      for (int j = 0; j < 3; ++j) out_t[hyp*3 + j] = (float)t_cand[best_cand][j];
    }
  }
}

// Final pass: write 1/0 mask per point under the winning (R, t).
__global__ void inlier_mask_kernel(
    const float* __restrict__ R, const float* __restrict__ t,
    const float* __restrict__ d_X, const float* __restrict__ d_uv,
    int N, float fx, float fy, float cx, float cy, float thr_px,
    int* __restrict__ mask) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= N) return;
  const float Xx = d_X[3*i + 0], Xy = d_X[3*i + 1], Xz = d_X[3*i + 2];
  const float px = R[0]*Xx + R[1]*Xy + R[2]*Xz + t[0];
  const float py = R[3]*Xx + R[4]*Xy + R[5]*Xz + t[1];
  const float pz = R[6]*Xx + R[7]*Xy + R[8]*Xz + t[2];
  if (pz < 1e-3f) { mask[i] = 0; return; }
  const float inv_z = 1.f / pz;
  const float u = fx * px * inv_z + cx;
  const float v = fy * py * inv_z + cy;
  const float du = u - d_uv[2*i + 0];
  const float dv = v - d_uv[2*i + 1];
  mask[i] = (du*du + dv*dv <= thr_px*thr_px) ? 1 : 0;
}

}  // namespace

// -----------------------------------------------------------------------------
// Host orchestration.
// -----------------------------------------------------------------------------
struct PnPCuda::Impl {
  int  max_points;
  int  max_hyp;
  float* d_X       = nullptr;
  float* d_uv      = nullptr;
  int*   d_samples = nullptr;
  int*   d_scores  = nullptr;
  float* d_R       = nullptr;
  float* d_t       = nullptr;
  int*   d_mask    = nullptr;
  float* d_best_R  = nullptr;
  float* d_best_t  = nullptr;
  std::vector<int>   h_samples;
  std::vector<int>   h_scores;
  std::vector<float> h_R;
  std::vector<float> h_t;
  std::vector<int>   h_mask;

  Impl(int n, int max_hyp_)
      : max_points(n), max_hyp(max_hyp_) {
    cudaMalloc(&d_X,         sizeof(float) * 3 * n);
    cudaMalloc(&d_uv,        sizeof(float) * 2 * n);
    cudaMalloc(&d_samples,   sizeof(int)   * 3 * max_hyp);
    cudaMalloc(&d_scores,    sizeof(int)   * max_hyp);
    cudaMalloc(&d_R,         sizeof(float) * 9 * max_hyp);
    cudaMalloc(&d_t,         sizeof(float) * 3 * max_hyp);
    cudaMalloc(&d_mask,      sizeof(int)   * n);
    cudaMalloc(&d_best_R,    sizeof(float) * 9);
    cudaMalloc(&d_best_t,    sizeof(float) * 3);
    h_samples.resize(3 * max_hyp);
    h_scores.resize(max_hyp);
    h_R.resize(9 * max_hyp);
    h_t.resize(3 * max_hyp);
    h_mask.resize(n);
  }
  ~Impl() {
    if (d_X) cudaFree(d_X);
    if (d_uv) cudaFree(d_uv);
    if (d_samples) cudaFree(d_samples);
    if (d_scores) cudaFree(d_scores);
    if (d_R) cudaFree(d_R);
    if (d_t) cudaFree(d_t);
    if (d_mask) cudaFree(d_mask);
    if (d_best_R) cudaFree(d_best_R);
    if (d_best_t) cudaFree(d_best_t);
  }
};

PnPCuda::PnPCuda(int max_points, const Config& cfg)
    : impl_(std::make_unique<Impl>(max_points, std::max(cfg.max_ransac_iters, 1))),
      cfg_(cfg) {}

PnPCuda::~PnPCuda() = default;

bool PnPCuda::solve(
    const std::vector<Eigen::Vector3f>& landmarks_3d,
    const std::vector<Eigen::Vector2f>& observations_2d_left,
    const StereoIntrinsics& K,
    Eigen::Matrix4f& T_prev_to_cur,
    std::vector<int>& inlier_indices,
    std::uint64_t random_seed) {
  const int N = (int)landmarks_3d.size();
  if (N != (int)observations_2d_left.size() || N < cfg_.min_inliers) return false;
  if (N > impl_->max_points) return false;

  const int n_hyp = std::min(cfg_.max_ransac_iters, impl_->max_hyp);

  // 1. Sample 3 distinct indices per hypothesis (deterministic from seed).
  std::mt19937_64 rng(random_seed);
  std::uniform_int_distribution<int> dist(0, N - 1);
  for (int h = 0; h < n_hyp; ++h) {
    int i0 = dist(rng), i1, i2;
    do { i1 = dist(rng); } while (i1 == i0);
    do { i2 = dist(rng); } while (i2 == i0 || i2 == i1);
    impl_->h_samples[3*h + 0] = i0;
    impl_->h_samples[3*h + 1] = i1;
    impl_->h_samples[3*h + 2] = i2;
  }

  // 2. Build / upload point cloud.
  std::vector<float> h_X(3 * N);
  std::vector<float> h_uv(2 * N);
  for (int i = 0; i < N; ++i) {
    h_X[3*i + 0] = landmarks_3d[i].x();
    h_X[3*i + 1] = landmarks_3d[i].y();
    h_X[3*i + 2] = landmarks_3d[i].z();
    h_uv[2*i + 0] = observations_2d_left[i].x();
    h_uv[2*i + 1] = observations_2d_left[i].y();
  }
  cudaMemcpy(impl_->d_X,       h_X.data(),                sizeof(float)*3*N,   cudaMemcpyHostToDevice);
  cudaMemcpy(impl_->d_uv,      h_uv.data(),               sizeof(float)*2*N,   cudaMemcpyHostToDevice);
  cudaMemcpy(impl_->d_samples, impl_->h_samples.data(),   sizeof(int)*3*n_hyp, cudaMemcpyHostToDevice);

  // 3. Launch P3P + score kernel.
  const int block = 128;
  solve_and_score_kernel<<<n_hyp, block>>>(
      impl_->d_samples, impl_->d_X, impl_->d_uv, N,
      K.fx, K.fy, K.cx, K.cy,
      cfg_.reprojection_threshold_px,
      impl_->d_scores, impl_->d_R, impl_->d_t);

  cudaMemcpy(impl_->h_scores.data(), impl_->d_scores, sizeof(int) * n_hyp, cudaMemcpyDeviceToHost);
  cudaMemcpy(impl_->h_R.data(),      impl_->d_R,      sizeof(float)*9*n_hyp, cudaMemcpyDeviceToHost);
  cudaMemcpy(impl_->h_t.data(),      impl_->d_t,      sizeof(float)*3*n_hyp, cudaMemcpyDeviceToHost);

  int best_idx = -1, best_count = -1;
  for (int h = 0; h < n_hyp; ++h) {
    if (impl_->h_scores[h] > best_count) {
      best_count = impl_->h_scores[h];
      best_idx = h;
    }
  }
  if (best_idx < 0 || best_count < cfg_.min_inliers) return false;

  // 4. Inlier mask kernel for the winner.
  cudaMemcpy(impl_->d_best_R, &impl_->h_R[best_idx * 9], sizeof(float)*9, cudaMemcpyHostToDevice);
  cudaMemcpy(impl_->d_best_t, &impl_->h_t[best_idx * 3], sizeof(float)*3, cudaMemcpyHostToDevice);
  const int blk = 256;
  inlier_mask_kernel<<<(N + blk - 1) / blk, blk>>>(
      impl_->d_best_R, impl_->d_best_t,
      impl_->d_X, impl_->d_uv, N,
      K.fx, K.fy, K.cx, K.cy, cfg_.reprojection_threshold_px,
      impl_->d_mask);
  cudaMemcpy(impl_->h_mask.data(), impl_->d_mask, sizeof(int)*N, cudaMemcpyDeviceToHost);

  inlier_indices.clear();
  inlier_indices.reserve(best_count);
  for (int i = 0; i < N; ++i) {
    if (impl_->h_mask[i]) inlier_indices.push_back(i);
  }

  T_prev_to_cur.setIdentity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T_prev_to_cur(r, c) = impl_->h_R[best_idx*9 + r*3 + c];
    }
    T_prev_to_cur(r, 3) = impl_->h_t[best_idx*3 + r];
  }
  return (int)inlier_indices.size() >= cfg_.min_inliers;
}

}  // namespace slamko_vio
