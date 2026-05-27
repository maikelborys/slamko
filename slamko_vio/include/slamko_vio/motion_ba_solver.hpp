// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Motion-only bundle adjustment for stereo VO. Header-only, __host__ __device__
// callable so the same code is unit-tested on CPU and deployed to GPU.
//
// Problem: given N inlier landmarks (Xw_i in world / previous-frame coords)
// with observations (u_l, v_l, u_r, v_r) and a current pose estimate (R, t)
// such that Xc_i = R Xw_i + t, refine (R, t) to minimise the sum of stereo
// reprojection errors with a Cauchy robust loss.
//
//   E(R, t) = sum_i ρ_c( ||r_l_i||² + has_right_i · ||r_r_i||² )
//
//   r_l_i = π_l(Xc_i) - u_l_i      (left  reprojection residual, 2-d)
//   r_r_i = π_r(Xc_i - [b,0,0]) - u_r_i   (right, 2-d, only if has_right_i)
//
// Optimisation: Levenberg-Marquardt with left perturbation on the camera pose:
//   R_new = exp(δaa) · R
//   t_new = exp(δaa) · t + δt              (full SE(3) update)
// Compact small-delta approximation:
//   Xc' ≈ Xc + δt - [Xc]_× δaa
//   ∂Xc/∂δaa = -[Xc]_× ,   ∂Xc/∂δt = I
// Jacobian of one residual: J = ∂π/∂Xc · ∂Xc/∂δ   (2×6 per camera).
//
// Cauchy weight (single-weight robust LS approximation):
//   ρ_c(s) = c² log(1 + s/c²),  weight w = ρ'(s) = 1 / (1 + s/c²).
//   Apply w to (J, r) before accumulating H = ΣJ^T W J,  g = ΣJ^T W r.
//
// We solve H δ = -g via 6×6 Cholesky (or LDL with a small ridge) and update
// the pose; if cost rose, undo and increase μ; otherwise decrease μ. Typical
// converged in 5-10 iterations.

#ifndef KLT_VO_MOTION_BA_SOLVER_HPP_
#define KLT_VO_MOTION_BA_SOLVER_HPP_

#ifdef __CUDACC__
#define KLT_VO_BA_HD __host__ __device__
#else
#define KLT_VO_BA_HD
#endif

#include <cmath>
#include <cstdint>

namespace slamko_vio {
namespace motion_ba {

// --- Tiny linalg primitives. R is row-major 3×3. ---
KLT_VO_BA_HD inline void mat3_vec3(const float R[9], const float v[3], float out[3]) {
  out[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
  out[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
  out[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}
KLT_VO_BA_HD inline void mat3_mul(const float A[9], const float B[9], float C[9]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      C[i*3 + j] = A[i*3+0]*B[0*3+j] + A[i*3+1]*B[1*3+j] + A[i*3+2]*B[2*3+j];
    }
  }
}

// Rodrigues exp map: aa (3-vec) → R (3×3 row-major).
KLT_VO_BA_HD inline void rodrigues_exp(const float aa[3], float R[9]) {
  const float theta2 = aa[0]*aa[0] + aa[1]*aa[1] + aa[2]*aa[2];
  if (theta2 < 1e-12f) {
    // R ≈ I + [aa]_×  for very small angles.
    R[0] = 1.f;  R[1] = -aa[2]; R[2] =  aa[1];
    R[3] = aa[2]; R[4] = 1.f;   R[5] = -aa[0];
    R[6] = -aa[1]; R[7] = aa[0]; R[8] = 1.f;
    return;
  }
  const float theta  = sqrtf(theta2);
  const float kx = aa[0]/theta, ky = aa[1]/theta, kz = aa[2]/theta;
  const float c = cosf(theta), s = sinf(theta), C = 1.0f - c;
  R[0] = c + kx*kx*C;     R[1] = kx*ky*C - kz*s;  R[2] = kx*kz*C + ky*s;
  R[3] = ky*kx*C + kz*s;  R[4] = c + ky*ky*C;     R[5] = ky*kz*C - kx*s;
  R[6] = kz*kx*C - ky*s;  R[7] = kz*ky*C + kx*s;  R[8] = c + kz*kz*C;
}

// Compose pose with left-multiplied perturbation (SO(3) on rotation, simple
// additive on translation — standard SLAM convention for small δ).
//   R_new = exp(δaa) · R
//   t_new = exp(δaa) · t + δt
KLT_VO_BA_HD inline void compose_pose_left(float R[9], float t[3], const float delta[6]) {
  float dR[9];
  const float daa[3] = {delta[0], delta[1], delta[2]};
  rodrigues_exp(daa, dR);
  float Rnew[9];
  mat3_mul(dR, R, Rnew);
  float dRt[3];
  mat3_vec3(dR, t, dRt);
  for (int i = 0; i < 9; ++i) R[i] = Rnew[i];
  t[0] = dRt[0] + delta[3];
  t[1] = dRt[1] + delta[4];
  t[2] = dRt[2] + delta[5];
}

// Compute reprojection residuals (4-d, right zeroed if has_right=false) and
// Jacobian (4×6 row-major, right zeroed if has_right=false).
// Returns squared residual norm (for Cauchy weight). Returns -1 if behind camera.
KLT_VO_BA_HD inline float compute_residual_jacobian(
    const float Xw[3], const float R[9], const float t[3],
    float fx, float fy, float cx, float cy, float baseline,
    float u_l, float v_l, float u_r, float v_r, bool has_right,
    float r[4], float J[24]) {  // J is 4×6 row-major
  float Xc[3];
  mat3_vec3(R, Xw, Xc);
  Xc[0] += t[0]; Xc[1] += t[1]; Xc[2] += t[2];
  if (Xc[2] < 1e-3f) return -1.f;
  const float inv_z  = 1.f / Xc[2];
  const float inv_z2 = inv_z * inv_z;

  // Left reprojection.
  const float pu_l = fx * Xc[0] * inv_z + cx;
  const float pv_l = fy * Xc[1] * inv_z + cy;
  r[0] = pu_l - u_l;
  r[1] = pv_l - v_l;
  // ∂π_l/∂Xc (2×3):
  //   [fx/z,  0,   -fx X/z²]
  //   [0,   fy/z,  -fy Y/z²]
  const float Jl[6] = {
    fx*inv_z, 0.f, -fx*Xc[0]*inv_z2,
    0.f, fy*inv_z, -fy*Xc[1]*inv_z2
  };
  // ∂Xc/∂δaa = -[Xc]_×  (row-major 3×3):
  //   [ 0,    Xc.z, -Xc.y ]
  //   [-Xc.z, 0,     Xc.x ]
  //   [ Xc.y, -Xc.x, 0    ]
  const float Mr[9] = {
    0.f, Xc[2], -Xc[1],
    -Xc[2], 0.f, Xc[0],
    Xc[1], -Xc[0], 0.f
  };
  // J_left = ∂π_l/∂Xc · [Mr, I_3]   (2×6).
  // First 3 cols (∂δaa):
  J[0*6 + 0] = Jl[0]*Mr[0] + Jl[1]*Mr[3] + Jl[2]*Mr[6];
  J[0*6 + 1] = Jl[0]*Mr[1] + Jl[1]*Mr[4] + Jl[2]*Mr[7];
  J[0*6 + 2] = Jl[0]*Mr[2] + Jl[1]*Mr[5] + Jl[2]*Mr[8];
  J[1*6 + 0] = Jl[3]*Mr[0] + Jl[4]*Mr[3] + Jl[5]*Mr[6];
  J[1*6 + 1] = Jl[3]*Mr[1] + Jl[4]*Mr[4] + Jl[5]*Mr[7];
  J[1*6 + 2] = Jl[3]*Mr[2] + Jl[4]*Mr[5] + Jl[5]*Mr[8];
  // Last 3 cols (∂δt = ∂π/∂Xc):
  J[0*6 + 3] = Jl[0]; J[0*6 + 4] = Jl[1]; J[0*6 + 5] = Jl[2];
  J[1*6 + 3] = Jl[3]; J[1*6 + 4] = Jl[4]; J[1*6 + 5] = Jl[5];

  float s = r[0]*r[0] + r[1]*r[1];

  if (has_right) {
    // Right camera: Xc_r = Xc - [b, 0, 0]. ∂Xc_r/∂Xc = I, so Jacobian shape is
    // the same as left but evaluated at Xc_r.
    const float Xr0 = Xc[0] - baseline;
    const float pu_r = fx * Xr0    * inv_z + cx;
    const float pv_r = fy * Xc[1]  * inv_z + cy;
    r[2] = pu_r - u_r;
    r[3] = pv_r - v_r;
    const float Jr[6] = {
      fx*inv_z, 0.f, -fx*Xr0*inv_z2,
      0.f, fy*inv_z, -fy*Xc[1]*inv_z2
    };
    J[2*6 + 0] = Jr[0]*Mr[0] + Jr[1]*Mr[3] + Jr[2]*Mr[6];
    J[2*6 + 1] = Jr[0]*Mr[1] + Jr[1]*Mr[4] + Jr[2]*Mr[7];
    J[2*6 + 2] = Jr[0]*Mr[2] + Jr[1]*Mr[5] + Jr[2]*Mr[8];
    J[3*6 + 0] = Jr[3]*Mr[0] + Jr[4]*Mr[3] + Jr[5]*Mr[6];
    J[3*6 + 1] = Jr[3]*Mr[1] + Jr[4]*Mr[4] + Jr[5]*Mr[7];
    J[3*6 + 2] = Jr[3]*Mr[2] + Jr[4]*Mr[5] + Jr[5]*Mr[8];
    J[2*6 + 3] = Jr[0]; J[2*6 + 4] = Jr[1]; J[2*6 + 5] = Jr[2];
    J[3*6 + 3] = Jr[3]; J[3*6 + 4] = Jr[4]; J[3*6 + 5] = Jr[5];
    s += r[2]*r[2] + r[3]*r[3];
  } else {
    r[2] = r[3] = 0.f;
    for (int j = 12; j < 24; ++j) J[j] = 0.f;
  }
  return s;
}

KLT_VO_BA_HD inline float cauchy_weight(float s, float c) {
  return 1.f / (1.f + s / (c * c));
}

// In-place 6×6 Cholesky solve: H is symmetric positive-definite (row-major).
// On entry H holds the matrix; on exit it holds L (lower triangular).
// Solves H x = b; b on entry is g, on exit is the solution x.
// Returns true on success, false if non-PD.
KLT_VO_BA_HD inline bool cholesky_solve_6(float H[36], float b[6]) {
  // Cholesky factorization (Cholesky-Banachiewicz, in-place lower triangle).
  for (int i = 0; i < 6; ++i) {
    float sum = H[i*6 + i];
    for (int k = 0; k < i; ++k) sum -= H[i*6 + k] * H[i*6 + k];
    if (sum <= 1e-12f) return false;
    H[i*6 + i] = sqrtf(sum);
    const float inv_lii = 1.f / H[i*6 + i];
    for (int j = i + 1; j < 6; ++j) {
      float s = H[j*6 + i];
      for (int k = 0; k < i; ++k) s -= H[j*6 + k] * H[i*6 + k];
      H[j*6 + i] = s * inv_lii;
    }
  }
  // Forward solve L y = b → b becomes y.
  for (int i = 0; i < 6; ++i) {
    float s = b[i];
    for (int k = 0; k < i; ++k) s -= H[i*6 + k] * b[k];
    b[i] = s / H[i*6 + i];
  }
  // Back solve L^T x = y → b becomes x.
  for (int i = 5; i >= 0; --i) {
    float s = b[i];
    for (int k = i + 1; k < 6; ++k) s -= H[k*6 + i] * b[k];
    b[i] = s / H[i*6 + i];
  }
  return true;
}

// Single inlier observation (used by the CPU host solver).
struct Obs {
  float Xw[3];
  float u_l, v_l;
  float u_r, v_r;
  bool  has_right;
};

// CPU LM solver. Returns true on success; updates R and t in place. Used by
// the unit test, and as the reference implementation that the GPU kernel
// matches numerically.
inline bool solve_motion_only_cpu(
    const Obs* obs, int N,
    float fx, float fy, float cx, float cy, float baseline,
    float cauchy_c, int max_iters,
    float R[9], float t[3]) {
  if (N < 3) return false;

  auto eval_cost = [&]() -> float {
    float total = 0.f;
    for (int i = 0; i < N; ++i) {
      const Obs& o = obs[i];
      float r[4], J[24];
      float s = compute_residual_jacobian(
          o.Xw, R, t, fx, fy, cx, cy, baseline,
          o.u_l, o.v_l, o.u_r, o.v_r, o.has_right, r, J);
      if (s < 0.f) { total += 1e9f; continue; }
      total += cauchy_c * cauchy_c * logf(1.f + s / (cauchy_c * cauchy_c));
    }
    return total;
  };

  float mu = 1e-3f;
  float prev_cost = eval_cost();

  for (int iter = 0; iter < max_iters; ++iter) {
    float H[36] = {0.f};
    float g[6]  = {0.f};
    int n_used = 0;
    for (int i = 0; i < N; ++i) {
      const Obs& o = obs[i];
      float r[4], J[24];
      float s = compute_residual_jacobian(
          o.Xw, R, t, fx, fy, cx, cy, baseline,
          o.u_l, o.v_l, o.u_r, o.v_r, o.has_right, r, J);
      if (s < 0.f) continue;
      const float w = cauchy_weight(s, cauchy_c);
      const int n_res = o.has_right ? 4 : 2;
      for (int row = 0; row < 6; ++row) {
        float gi = 0.f;
        for (int k = 0; k < n_res; ++k) gi += J[k*6 + row] * r[k];
        g[row] += w * gi;
        for (int col = 0; col <= row; ++col) {
          float hij = 0.f;
          for (int k = 0; k < n_res; ++k) hij += J[k*6 + row] * J[k*6 + col];
          H[row*6 + col] += w * hij;
          if (col != row) H[col*6 + row] = H[row*6 + col];
        }
      }
      ++n_used;
    }
    if (n_used < 3) return false;

    // Apply LM damping (multiplicative on diagonal).
    float H_damp[36];
    for (int i = 0; i < 36; ++i) H_damp[i] = H[i];
    for (int i = 0; i < 6; ++i) H_damp[i*6 + i] *= (1.f + mu);
    float delta[6] = {-g[0], -g[1], -g[2], -g[3], -g[4], -g[5]};
    if (!cholesky_solve_6(H_damp, delta)) return false;

    // Try update.
    float R_save[9], t_save[3];
    for (int i = 0; i < 9; ++i) R_save[i] = R[i];
    for (int i = 0; i < 3; ++i) t_save[i] = t[i];
    compose_pose_left(R, t, delta);

    const float new_cost = eval_cost();
    if (new_cost < prev_cost) {
      prev_cost = new_cost;
      mu *= 0.5f;
      if (mu < 1e-7f) mu = 1e-7f;
    } else {
      // Reject — restore.
      for (int i = 0; i < 9; ++i) R[i] = R_save[i];
      for (int i = 0; i < 3; ++i) t[i] = t_save[i];
      mu *= 4.f;
      if (mu > 1e3f) break;
    }
  }
  return true;
}

}  // namespace motion_ba
}  // namespace slamko_vio

#endif  // KLT_VO_MOTION_BA_SOLVER_HPP_
