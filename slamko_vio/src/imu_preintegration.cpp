// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// On-manifold IMU preintegration following Forster et al. 2017
// "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
// (T-RO 33(1), arXiv:1512.02363).
//
// Discrete-time formulation. Notation matches the paper:
//   - ΔR_ij, ΔV_ij, ΔP_ij at linearisation bias b_lin
//   - 9x9 covariance Σ in order [rot, vel, pos]
//   - First-order bias Jacobians J_*_b{a,g}
//
// Noise model: continuous-time white noise with spectral density σ_c
// (units / √Hz). Discrete per-step variance: σ_d² = σ_c² / Δt.

#include "slamko_vio/imu_preintegration.hpp"

#include <cmath>

namespace slamko_vio {

// =============================================================================
// SO(3) helpers
// =============================================================================

namespace so3 {

Eigen::Matrix3d Exp(const Eigen::Vector3d& phi) {
  const double theta = phi.norm();
  if (theta < 1.0e-9) {
    // 2nd-order Taylor: Exp(phi) ≈ I + hat(phi) + 0.5 hat(phi)²
    const Eigen::Matrix3d S = hat(phi);
    return Eigen::Matrix3d::Identity() + S + 0.5 * S * S;
  }
  const Eigen::Vector3d a = phi / theta;
  const Eigen::Matrix3d K = hat(a);
  return Eigen::Matrix3d::Identity()
       + std::sin(theta) * K
       + (1.0 - std::cos(theta)) * (K * K);
}

Eigen::Vector3d Log(const Eigen::Matrix3d& R) {
  // arccos((tr-1)/2), clamped for numerical safety.
  const double tr = R.trace();
  const double cos_theta = std::min(std::max(0.5 * (tr - 1.0), -1.0), 1.0);
  const double theta = std::acos(cos_theta);
  if (theta < 1.0e-9) {
    // For small angles, Log(R) ≈ skew⁻¹(R - I)
    return Eigen::Vector3d(R(2, 1) - R(1, 2),
                           R(0, 2) - R(2, 0),
                           R(1, 0) - R(0, 1)) * 0.5;
  }
  const double s = theta / (2.0 * std::sin(theta));
  return s * Eigen::Vector3d(R(2, 1) - R(1, 2),
                             R(0, 2) - R(2, 0),
                             R(1, 0) - R(0, 1));
}

Eigen::Matrix3d J_r(const Eigen::Vector3d& phi) {
  const double theta = phi.norm();
  if (theta < 1.0e-6) {
    return Eigen::Matrix3d::Identity() - 0.5 * hat(phi);
  }
  const Eigen::Matrix3d S = hat(phi);
  const double t2 = theta * theta;
  const double t3 = t2 * theta;
  return Eigen::Matrix3d::Identity()
       - ((1.0 - std::cos(theta)) / t2) * S
       + ((theta - std::sin(theta)) / t3) * (S * S);
}

Eigen::Matrix3d J_r_inv(const Eigen::Vector3d& phi) {
  const double theta = phi.norm();
  if (theta < 1.0e-6) {
    return Eigen::Matrix3d::Identity() + 0.5 * hat(phi);
  }
  const Eigen::Matrix3d S = hat(phi);
  const double t2 = theta * theta;
  const double cot_half = std::cos(theta * 0.5) / std::sin(theta * 0.5);
  return Eigen::Matrix3d::Identity()
       + 0.5 * S
       + (1.0 / t2 - 0.5 * cot_half / theta) * (S * S);
}

}  // namespace so3

// =============================================================================
// Preintegration
// =============================================================================

ImuPreintegration::ImuPreintegration(const ImuBias& bias_lin,
                                     const ImuNoise& noise)
    : bias_lin_(bias_lin), noise_(noise) {}

void ImuPreintegration::integrate(double dt,
                                  const Eigen::Vector3d& w_meas,
                                  const Eigen::Vector3d& a_meas) {
  if (dt <= 0.0) return;
  const Eigen::Vector3d w = w_meas - bias_lin_.bg;
  const Eigen::Vector3d a = a_meas - bias_lin_.ba;
  const Eigen::Vector3d wdt = w * dt;
  const Eigen::Matrix3d dR_step = so3::Exp(wdt);
  const Eigen::Matrix3d Jr = so3::J_r(wdt);

  // Cache previous ΔR, ΔV for the position/velocity updates.
  const Eigen::Matrix3d R_prev = dR_;
  const Eigen::Matrix3d a_hat  = so3::hat(a);
  const Eigen::Matrix3d R_a    = R_prev * a_hat;

  // -- Covariance / bias Jacobians (use previous state, then advance) ----
  // A (9x9), B (9x6): see Forster eqs. (62), (63).
  Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
  Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();

  // ∂Δθ_{k+1}/∂Δθ_k = dR_step^T  (right-perturbation propagation)
  A.block<3, 3>(0, 0) = dR_step.transpose();
  // ∂ΔV_{k+1}/∂Δθ_k = -R_prev * hat(a) * dt
  A.block<3, 3>(3, 0) = -R_a * dt;
  // ∂ΔV_{k+1}/∂ΔV_k = I  (already)
  // ∂ΔP_{k+1}/∂Δθ_k = -0.5 * R_prev * hat(a) * dt^2
  A.block<3, 3>(6, 0) = -0.5 * R_a * dt * dt;
  // ∂ΔP_{k+1}/∂ΔV_k = I * dt
  A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;

  // B: noise injection. B*η maps gyro/accel noise into Δ-state increments.
  // η order: [η_g (3), η_a (3)]; per-step cov diag(σ_g²/dt, σ_a²/dt).
  B.block<3, 3>(0, 0) = Jr * dt;
  B.block<3, 3>(3, 3) = R_prev * dt;
  B.block<3, 3>(6, 3) = 0.5 * R_prev * dt * dt;

  Eigen::Matrix<double, 6, 6> Sigma_eta = Eigen::Matrix<double, 6, 6>::Zero();
  const double sg2 = noise_.gyro_noise_density  * noise_.gyro_noise_density;
  const double sa2 = noise_.accel_noise_density * noise_.accel_noise_density;
  Sigma_eta.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (sg2 / dt);
  Sigma_eta.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (sa2 / dt);

  cov_ = A * cov_ * A.transpose() + B * Sigma_eta * B.transpose();

  // Bias Jacobian recurrences (Forster, appendix A).
  const Eigen::Matrix3d J_dR_bg_prev = J_dR_bg_;
  const Eigen::Matrix3d J_dV_bg_prev = J_dV_bg_;
  const Eigen::Matrix3d J_dV_ba_prev = J_dV_ba_;
  const Eigen::Matrix3d J_dP_bg_prev = J_dP_bg_;
  const Eigen::Matrix3d J_dP_ba_prev = J_dP_ba_;
  const Eigen::Matrix3d R_a_J_dR_bg  = R_a * J_dR_bg_prev;

  J_dR_bg_ = dR_step.transpose() * J_dR_bg_prev - Jr * dt;
  J_dV_ba_ = J_dV_ba_prev - R_prev * dt;
  J_dV_bg_ = J_dV_bg_prev - R_a_J_dR_bg * dt;
  J_dP_ba_ = J_dP_ba_prev + J_dV_ba_prev * dt - 0.5 * R_prev * dt * dt;
  J_dP_bg_ = J_dP_bg_prev + J_dV_bg_prev * dt - 0.5 * R_a_J_dR_bg * dt * dt;

  // -- State propagation (note the order matters: P uses old V, V uses old R) --
  dP_ += dV_ * dt + 0.5 * R_prev * a * dt * dt;
  dV_ += R_prev * a * dt;
  dR_  = R_prev * dR_step;
  dt_ += dt;
}

void ImuPreintegration::integrate_span(const ImuSample* samples,
                                       std::size_t n) {
  if (n < 2) return;
  for (std::size_t i = 1; i < n; ++i) {
    const double dt = samples[i].t - samples[i - 1].t;
    integrate(dt, samples[i - 1].w, samples[i - 1].a);
  }
}

void ImuPreintegration::corrected(const ImuBias& bias_new,
                                  Eigen::Matrix3d& dR_out,
                                  Eigen::Vector3d& dV_out,
                                  Eigen::Vector3d& dP_out) const {
  const Eigen::Vector3d dbg = bias_new.bg - bias_lin_.bg;
  const Eigen::Vector3d dba = bias_new.ba - bias_lin_.ba;
  dR_out = dR_ * so3::Exp(J_dR_bg_ * dbg);
  dV_out = dV_ + J_dV_bg_ * dbg + J_dV_ba_ * dba;
  dP_out = dP_ + J_dP_bg_ * dbg + J_dP_ba_ * dba;
}

Eigen::Matrix<double, 9, 9> ImuPreintegration::sqrt_information() const {
  // Σ might be borderline-rank-deficient on the first step; add a tiny ridge
  // so the LLT decomposition stays stable.
  Eigen::Matrix<double, 9, 9> S = cov_;
  S.diagonal().array() += 1.0e-12;
  Eigen::LLT<Eigen::Matrix<double, 9, 9>> llt(S.inverse());
  return llt.matrixL().transpose();   // upper triangular L^T s.t. info = L^T L
}

}  // namespace slamko_vio
