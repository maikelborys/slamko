// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Ceres cost function for the Forster 2017 preintegrated IMU residual.
//
// Parameter blocks (per consecutive keyframes i, j):
//   - aa_i: 3 doubles. World-to-cam rotation in angle-axis (matches LocalBA).
//   - t_i:  3 doubles. World-to-cam translation.
//   - vel_i:  3 doubles. Body velocity in world frame, m/s.
//   - bias_i: 6 doubles. (bias_g[3], bias_a[3]).
//   - aa_j, t_j, vel_j, bias_j: same shape, frame j.
//
// Residual (15-dim):
//   r[0..2]   = Log(ΔR_corr^T · R_i^T · R_j)
//   r[3..5]   = R_i^T (v_j - v_i - g Δt) - ΔV_corr
//   r[6..8]   = R_i^T (p_j - p_i - v_i Δt - 0.5 g Δt²) - ΔP_corr
//   r[9..11]  = bias_g_j - bias_g_i               (random walk)
//   r[12..14] = bias_a_j - bias_a_i               (random walk)
//
// The first 9 residuals are whitened by ImuPreintegration::sqrt_information().
// The bias-RW residuals are whitened by 1/√(σ_rw² · Δt).
//
// Pose convention: the optimisation parameter is the (world-to-cam) pose used
// by LocalBA. Internally we convert to (world-to-body) using the fixed
// extrinsic T_BS (sensor-in-body, i.e. p_body = T_BS * p_cam). Specifically
// for the IMU residual we need R_world_body and p_world_body:
//   p_body = T_BS * p_cam
//          = T_BS * (R_wc * p_world + t_wc)
//          = (T_BS_R * R_wc) p_world + (T_BS_R * t_wc + T_BS_t)
//   so R_w_b = T_BS_R * R_wc,  t_w_b = T_BS_R * t_wc + T_BS_t.
//   Forster's R_i is "body-to-world" = R_w_b^T.
//   Forster's p_i is "body-in-world" = -R_w_b^T * t_w_b.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "slamko_vio/imu_preintegration.hpp"
#include "slamko_vio/imu_types.hpp"

namespace slamko_vio {

// Templated SO(3) helpers usable with ceres::Jet types via auto-diff.
namespace so3_t {

template <typename T>
Eigen::Matrix<T, 3, 3> hat(const Eigen::Matrix<T, 3, 1>& v) {
  Eigen::Matrix<T, 3, 3> S;
  S <<  T(0.0), -v.z(),  v.y(),
        v.z(),  T(0.0), -v.x(),
       -v.y(),  v.x(),  T(0.0);
  return S;
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& phi) {
  const T theta2 = phi.squaredNorm();
  if (theta2 < T(1.0e-12)) {
    // 2nd-order Taylor.
    const Eigen::Matrix<T, 3, 3> S = hat<T>(phi);
    return Eigen::Matrix<T, 3, 3>::Identity() + S + T(0.5) * (S * S);
  }
  const T theta = ceres::sqrt(theta2);
  const Eigen::Matrix<T, 3, 1> a = phi / theta;
  const Eigen::Matrix<T, 3, 3> K = hat<T>(a);
  return Eigen::Matrix<T, 3, 3>::Identity()
       + ceres::sin(theta) * K
       + (T(1.0) - ceres::cos(theta)) * (K * K);
}

template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& R) {
  const T tr = R(0, 0) + R(1, 1) + R(2, 2);
  // Clamp 0.5*(tr-1) into [-1, 1] before acos.
  T arg = T(0.5) * (tr - T(1.0));
  if (arg > T( 1.0)) arg = T( 1.0);
  if (arg < T(-1.0)) arg = T(-1.0);
  const T theta = ceres::acos(arg);
  // Vector along skew-symmetric part of R - R^T.
  Eigen::Matrix<T, 3, 1> w;
  w[0] = R(2, 1) - R(1, 2);
  w[1] = R(0, 2) - R(2, 0);
  w[2] = R(1, 0) - R(0, 1);
  // For very small angles use the linearised form.
  if (theta < T(1.0e-6)) return T(0.5) * w;
  const T s = theta / (T(2.0) * ceres::sin(theta));
  return s * w;
}

}  // namespace so3_t

// Forster IMU factor, AutoDiff with 15 residuals over (pose_i, vel_i, bias_i,
// pose_j, vel_j, bias_j). Holds the preintegration result, the IMU↔cam
// extrinsic and gravity by value.
class ImuFactor {
 public:
  ImuFactor(const ImuPreintegration& pi,
            const Eigen::Matrix4d&    T_BS,        // sensor → body
            const Eigen::Vector3d&    gravity_w,   // m/s², e.g. (0,0,-9.81)
            double bias_rw_gyro,                   // σ_bg per √s (rad/s²/√Hz)
            double bias_rw_accel)                  // σ_ba per √s (m/s³/√Hz)
      : pi_(pi), T_BS_(T_BS), g_(gravity_w),
        bias_rw_gyro_(bias_rw_gyro), bias_rw_accel_(bias_rw_accel) {
    // 9x9 sqrt-information for the ΔR/V/P part.
    sqrt_info_imu_ = pi.sqrt_information();
    // 1/σ for bias random walks. σ² = (σ_d)² · Δt for white noise integrated
    // over the interval.
    const double sig_bg = bias_rw_gyro_  * std::sqrt(pi.dt());
    const double sig_ba = bias_rw_accel_ * std::sqrt(pi.dt());
    inv_sig_bg_ = (sig_bg > 0.0) ? (1.0 / sig_bg) : 1.0;
    inv_sig_ba_ = (sig_ba > 0.0) ? (1.0 / sig_ba) : 1.0;
  }

  template <typename T>
  bool operator()(const T* const aa_i, const T* const t_i_p,
                  const T* const vel_i, const T* const bias_i,
                  const T* const aa_j, const T* const t_j_p,
                  const T* const vel_j, const T* const bias_j,
                  T* residuals) const {
    using Vec3   = Eigen::Matrix<T, 3, 1>;
    using Mat3   = Eigen::Matrix<T, 3, 3>;

    // ---- Extract world-to-cam SE(3) for both frames -------------------------
    T R_wc_i_arr[9];  ceres::AngleAxisToRotationMatrix(aa_i, R_wc_i_arr);
    T R_wc_j_arr[9];  ceres::AngleAxisToRotationMatrix(aa_j, R_wc_j_arr);
    // Ceres uses column-major output.
    Mat3 R_wc_i, R_wc_j;
    R_wc_i << R_wc_i_arr[0], R_wc_i_arr[3], R_wc_i_arr[6],
              R_wc_i_arr[1], R_wc_i_arr[4], R_wc_i_arr[7],
              R_wc_i_arr[2], R_wc_i_arr[5], R_wc_i_arr[8];
    R_wc_j << R_wc_j_arr[0], R_wc_j_arr[3], R_wc_j_arr[6],
              R_wc_j_arr[1], R_wc_j_arr[4], R_wc_j_arr[7],
              R_wc_j_arr[2], R_wc_j_arr[5], R_wc_j_arr[8];
    Vec3 t_wc_i(t_i_p[0], t_i_p[1], t_i_p[2]);
    Vec3 t_wc_j(t_j_p[0], t_j_p[1], t_j_p[2]);

    // T_BS (constant): convert cam-frame pose to body-frame pose.
    Mat3 R_BS;
    R_BS << T(T_BS_(0,0)), T(T_BS_(0,1)), T(T_BS_(0,2)),
            T(T_BS_(1,0)), T(T_BS_(1,1)), T(T_BS_(1,2)),
            T(T_BS_(2,0)), T(T_BS_(2,1)), T(T_BS_(2,2));
    Vec3 t_BS(T(T_BS_(0,3)), T(T_BS_(1,3)), T(T_BS_(2,3)));

    // R_wb = R_BS · R_wc, t_wb = R_BS · t_wc + t_BS  (see header comment).
    Mat3 R_wb_i = R_BS * R_wc_i;
    Vec3 t_wb_i = R_BS * t_wc_i + t_BS;
    Mat3 R_wb_j = R_BS * R_wc_j;
    Vec3 t_wb_j = R_BS * t_wc_j + t_BS;

    // Forster's R_i = body-to-world = R_wb^T; p_i = -R_bw · t_wb.
    Mat3 R_i = R_wb_i.transpose();
    Mat3 R_j = R_wb_j.transpose();
    Vec3 p_i = -R_i * t_wb_i;
    Vec3 p_j = -R_j * t_wb_j;

    Vec3 v_i(vel_i[0], vel_i[1], vel_i[2]);
    Vec3 v_j(vel_j[0], vel_j[1], vel_j[2]);

    // ---- Bias deltas from preintegration linearisation point ----------------
    Vec3 dbg(bias_i[0] - T(pi_.bias_lin().bg.x()),
             bias_i[1] - T(pi_.bias_lin().bg.y()),
             bias_i[2] - T(pi_.bias_lin().bg.z()));
    Vec3 dba(bias_i[3] - T(pi_.bias_lin().ba.x()),
             bias_i[4] - T(pi_.bias_lin().ba.y()),
             bias_i[5] - T(pi_.bias_lin().ba.z()));

    // ---- Bias-corrected preintegrated quantities ----------------------------
    Mat3 dR_lin;
    dR_lin << T(pi_.dR()(0,0)), T(pi_.dR()(0,1)), T(pi_.dR()(0,2)),
              T(pi_.dR()(1,0)), T(pi_.dR()(1,1)), T(pi_.dR()(1,2)),
              T(pi_.dR()(2,0)), T(pi_.dR()(2,1)), T(pi_.dR()(2,2));
    Mat3 J_dR_bg;
    J_dR_bg << T(pi_.J_dR_bg()(0,0)), T(pi_.J_dR_bg()(0,1)), T(pi_.J_dR_bg()(0,2)),
               T(pi_.J_dR_bg()(1,0)), T(pi_.J_dR_bg()(1,1)), T(pi_.J_dR_bg()(1,2)),
               T(pi_.J_dR_bg()(2,0)), T(pi_.J_dR_bg()(2,1)), T(pi_.J_dR_bg()(2,2));
    Vec3 dV_lin(T(pi_.dV().x()), T(pi_.dV().y()), T(pi_.dV().z()));
    Vec3 dP_lin(T(pi_.dP().x()), T(pi_.dP().y()), T(pi_.dP().z()));
    auto mat_to_T = [](const Eigen::Matrix3d& M) {
      Mat3 R;
      R << T(M(0,0)), T(M(0,1)), T(M(0,2)),
           T(M(1,0)), T(M(1,1)), T(M(1,2)),
           T(M(2,0)), T(M(2,1)), T(M(2,2));
      return R;
    };
    const Mat3 J_dV_bg = mat_to_T(pi_.J_dV_bg());
    const Mat3 J_dV_ba = mat_to_T(pi_.J_dV_ba());
    const Mat3 J_dP_bg = mat_to_T(pi_.J_dP_bg());
    const Mat3 J_dP_ba = mat_to_T(pi_.J_dP_ba());

    const Mat3 dR_corr = dR_lin * so3_t::Exp<T>(Vec3(J_dR_bg * dbg));
    const Vec3 dV_corr = dV_lin + J_dV_bg * dbg + J_dV_ba * dba;
    const Vec3 dP_corr = dP_lin + J_dP_bg * dbg + J_dP_ba * dba;

    // ---- Forster residuals --------------------------------------------------
    Vec3 g(T(g_.x()), T(g_.y()), T(g_.z()));
    const T dt   = T(pi_.dt());
    const T dt2h = T(0.5 * pi_.dt() * pi_.dt());

    Vec3 r_R = so3_t::Log<T>(Mat3(dR_corr.transpose() * R_i.transpose() * R_j));
    Vec3 r_V = R_i.transpose() * (v_j - v_i - g * dt) - dV_corr;
    Vec3 r_P = R_i.transpose() * (p_j - p_i - v_i * dt - g * dt2h) - dP_corr;

    // Whiten by sqrt(information) — 9x9 upper-triangular L^T s.t. info = L^T L.
    Eigen::Matrix<T, 9, 1> r_imu;
    r_imu << r_R, r_V, r_P;
    Eigen::Matrix<T, 9, 9> S;
    for (int a = 0; a < 9; ++a)
      for (int b = 0; b < 9; ++b)
        S(a, b) = T(sqrt_info_imu_(a, b));
    Eigen::Matrix<T, 9, 1> r_imu_w = S * r_imu;
    for (int k = 0; k < 9; ++k) residuals[k] = r_imu_w[k];

    // Bias-RW residuals: (b_j - b_i) / σ_rw.
    residuals[9]  = (bias_j[0] - bias_i[0]) * T(inv_sig_bg_);
    residuals[10] = (bias_j[1] - bias_i[1]) * T(inv_sig_bg_);
    residuals[11] = (bias_j[2] - bias_i[2]) * T(inv_sig_bg_);
    residuals[12] = (bias_j[3] - bias_i[3]) * T(inv_sig_ba_);
    residuals[13] = (bias_j[4] - bias_i[4]) * T(inv_sig_ba_);
    residuals[14] = (bias_j[5] - bias_i[5]) * T(inv_sig_ba_);

    return true;
  }

  static ceres::CostFunction* Create(const ImuPreintegration& pi,
                                     const Eigen::Matrix4d&    T_BS,
                                     const Eigen::Vector3d&    g,
                                     double bias_rw_gyro,
                                     double bias_rw_accel) {
    // Block sizes: 15 residuals; aa 3, t 3, vel 3, bias 6 (×2 frames).
    return new ceres::AutoDiffCostFunction<
        ImuFactor, 15,
        3, 3, 3, 6,
        3, 3, 3, 6>(
        new ImuFactor(pi, T_BS, g, bias_rw_gyro, bias_rw_accel));
  }

 private:
  ImuPreintegration            pi_;
  Eigen::Matrix4d              T_BS_;
  Eigen::Vector3d              g_;
  double                       bias_rw_gyro_;
  double                       bias_rw_accel_;
  Eigen::Matrix<double, 9, 9>  sqrt_info_imu_;
  double                       inv_sig_bg_;
  double                       inv_sig_ba_;
};

}  // namespace slamko_vio
