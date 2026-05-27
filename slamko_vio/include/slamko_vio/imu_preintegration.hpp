// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// IMU preintegration following Forster et al. 2017 "On-Manifold Preintegration
// for Real-Time Visual-Inertial Odometry", IEEE T-RO 33(1).
//
// Given a sequence of IMU samples between two image frames (i, j), this class
// accumulates the preintegrated rotation ΔR_ij, velocity ΔV_ij, and position
// ΔP_ij, along with the 9×9 covariance Σ_ij of these quantities and the
// first-order Jacobians w.r.t. the linearisation bias.
//
// The IMU factor (see imu_factor.hpp) then enforces:
//   ΔR_ij_corrected ≈ R_i^T R_j
//   ΔV_ij_corrected ≈ R_i^T (v_j - v_i - g Δt_ij)
//   ΔP_ij_corrected ≈ R_i^T (p_j - p_i - v_i Δt_ij - 0.5 g Δt_ij²)
// where "_corrected" applies the first-order bias-correction Jacobians when
// the optimisation moves the bias away from the linearisation point.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slamko_vio/imu_types.hpp"

namespace slamko_vio {

class ImuPreintegration {
 public:
  ImuPreintegration(const ImuBias& bias_lin,
                    const ImuNoise& noise = ImuNoise{});

  // Integrate one IMU step. dt is the elapsed time since the previous sample
  // (or since the i-frame for the first call). `w` and `a` are the gyro and
  // accel readings; biases are subtracted internally.
  void integrate(double dt,
                 const Eigen::Vector3d& w_meas,
                 const Eigen::Vector3d& a_meas);

  // Convenience: integrate a contiguous span. Each consecutive pair must
  // have monotonically increasing timestamps. dt is computed from samples.
  void integrate_span(const ImuSample* samples, std::size_t n);

  // Apply the first-order bias correction at the current linearisation
  // point. If the optimiser changes the bias by `db = b_new - b_lin`, the
  // bias-corrected preintegrated measurements are:
  //   ΔR_corr = ΔR̄ * Exp(J_R_bg * db.bg)
  //   ΔV_corr = ΔV̄ + J_V_bg * db.bg + J_V_ba * db.ba
  //   ΔP_corr = ΔP̄ + J_P_bg * db.bg + J_P_ba * db.ba
  void corrected(const ImuBias& bias_new,
                 Eigen::Matrix3d& dR_out,
                 Eigen::Vector3d& dV_out,
                 Eigen::Vector3d& dP_out) const;

  // Raw preintegrated measurements at the linearisation bias.
  const Eigen::Matrix3d& dR()   const { return dR_; }
  const Eigen::Vector3d& dV()   const { return dV_; }
  const Eigen::Vector3d& dP()   const { return dP_; }
  double                 dt()   const { return dt_; }
  const ImuBias&         bias_lin() const { return bias_lin_; }

  // 9×9 covariance of [dR, dV, dP] in the order (rotation 3, velocity 3,
  // position 3). Already includes IMU noise propagation; does not include
  // bias random walk (that's handled by a separate factor between i and j).
  const Eigen::Matrix<double, 9, 9>& covariance() const { return cov_; }

  // Bias Jacobians: 3×3 each.
  const Eigen::Matrix3d& J_dR_bg() const { return J_dR_bg_; }
  const Eigen::Matrix3d& J_dV_bg() const { return J_dV_bg_; }
  const Eigen::Matrix3d& J_dV_ba() const { return J_dV_ba_; }
  const Eigen::Matrix3d& J_dP_bg() const { return J_dP_bg_; }
  const Eigen::Matrix3d& J_dP_ba() const { return J_dP_ba_; }

  // 9×9 information matrix (cov^-1) — what the Ceres factor scales the
  // residual by. Cached on first call.
  Eigen::Matrix<double, 9, 9> sqrt_information() const;

 private:
  ImuBias  bias_lin_;
  ImuNoise noise_;

  double         dt_  = 0.0;          // total elapsed time
  Eigen::Matrix3d dR_  = Eigen::Matrix3d::Identity();
  Eigen::Vector3d dV_  = Eigen::Vector3d::Zero();
  Eigen::Vector3d dP_  = Eigen::Vector3d::Zero();

  Eigen::Matrix<double, 9, 9> cov_  = Eigen::Matrix<double, 9, 9>::Zero();

  // First-order bias Jacobians (∂(dR,dV,dP)/∂(bg,ba)).
  Eigen::Matrix3d J_dR_bg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_dV_bg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_dV_ba_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_dP_bg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_dP_ba_ = Eigen::Matrix3d::Zero();
};

// SO(3) helpers used by both preintegration and the IMU factor.
namespace so3 {

// Skew-symmetric matrix from 3-vector.
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S <<     0.0, -v.z(),  v.y(),
        v.z(),     0.0, -v.x(),
       -v.y(),  v.x(),     0.0;
  return S;
}

// Rodrigues / Exp(SO(3)) for a 3-vector.
Eigen::Matrix3d Exp(const Eigen::Vector3d& phi);

// Log(SO(3)) → 3-vector.
Eigen::Vector3d Log(const Eigen::Matrix3d& R);

// Right Jacobian of Exp at phi (Forster eq. 7).
Eigen::Matrix3d J_r(const Eigen::Vector3d& phi);

// Inverse of right Jacobian.
Eigen::Matrix3d J_r_inv(const Eigen::Vector3d& phi);

}  // namespace so3

}  // namespace slamko_vio
