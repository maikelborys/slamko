// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Minimal SE(3)/SO(3) manifold on Eigen — the only math slamko_core ships.
// Header-only, double precision, Eigen-only (no Sophus/GTSAM dep), so every
// package can use it without pulling a backend.
//
// Conventions (locked, MASTER_PLAN §8.3): the pose node is T_WB (body-in-world).
// SE(3) twist ordering is [rho; omega] (translation-first, Sophus-compatible):
//   exp([rho; omega]) = ( SO3::exp(omega), V(omega) * rho ).
// Group action: SE3 * point transforms the point; SE3 * SE3 composes.

#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace slamko {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// ---------------------------------------------------------------- SO(3)
class SO3 {
 public:
  SO3() : q_(Eigen::Quaterniond::Identity()) {}
  explicit SO3(const Eigen::Quaterniond& q) : q_(q.normalized()) {}
  explicit SO3(const Eigen::Matrix3d& R) : q_(Eigen::Quaterniond(R)) { q_.normalize(); }

  // hat: R^3 -> so(3) skew-symmetric; vee: inverse.
  static Eigen::Matrix3d hat(const Eigen::Vector3d& w) {
    Eigen::Matrix3d S;
    S <<   0.0, -w.z(),  w.y(),
         w.z(),    0.0, -w.x(),
        -w.y(),  w.x(),    0.0;
    return S;
  }
  static Eigen::Vector3d vee(const Eigen::Matrix3d& W) {
    return Eigen::Vector3d(W(2, 1), W(0, 2), W(1, 0));
  }

  // Exponential map: axis-angle (rad) -> rotation (Rodrigues, Taylor near 0).
  static SO3 exp(const Eigen::Vector3d& w) {
    const double theta2 = w.squaredNorm();
    if (theta2 < kEps2) {
      const Eigen::Matrix3d S = hat(w);
      Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + S + 0.5 * S * S;
      return SO3(R);
    }
    const double theta = std::sqrt(theta2);
    const Eigen::Vector3d axis = w / theta;
    return SO3(Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis)));
  }

  // Logarithm: rotation -> axis-angle (rad).
  Eigen::Vector3d log() const {
    Eigen::AngleAxisd aa(q_);
    return aa.angle() * aa.axis();
  }

  SO3 inverse() const { return SO3(q_.conjugate()); }
  Eigen::Matrix3d matrix() const { return q_.toRotationMatrix(); }
  const Eigen::Quaterniond& unit_quaternion() const { return q_; }
  // Adjoint of SO(3) is just the rotation matrix.
  Eigen::Matrix3d adjoint() const { return matrix(); }

  SO3 operator*(const SO3& o) const { return SO3(q_ * o.q_); }
  Eigen::Vector3d operator*(const Eigen::Vector3d& p) const { return q_ * p; }

  static constexpr double kEps2 = 1e-20;  // (1e-10 rad)^2 — Taylor cutoff
 private:
  Eigen::Quaterniond q_;
};

// ---------------------------------------------------------------- SE(3)
class SE3 {
 public:
  SE3() : R_(), t_(Eigen::Vector3d::Zero()) {}
  SE3(const SO3& R, const Eigen::Vector3d& t) : R_(R), t_(t) {}
  SE3(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) : R_(R), t_(t) {}
  explicit SE3(const Eigen::Matrix4d& T)
      : R_(Eigen::Matrix3d(T.topLeftCorner<3, 3>())),
        t_(T.topRightCorner<3, 1>()) {}

  // Left Jacobian of SO(3) and its inverse (used by SE(3) exp/log).
  static Eigen::Matrix3d leftJacobian(const Eigen::Vector3d& w) {
    const double t2 = w.squaredNorm();
    const Eigen::Matrix3d W = SO3::hat(w);
    if (t2 < SO3::kEps2) {
      return Eigen::Matrix3d::Identity() + 0.5 * W + (1.0 / 6.0) * W * W;
    }
    const double t = std::sqrt(t2);
    const double B = (1.0 - std::cos(t)) / t2;
    const double C = (t - std::sin(t)) / (t2 * t);
    return Eigen::Matrix3d::Identity() + B * W + C * W * W;
  }
  static Eigen::Matrix3d leftJacobianInverse(const Eigen::Vector3d& w) {
    const double t2 = w.squaredNorm();
    const Eigen::Matrix3d W = SO3::hat(w);
    if (t2 < SO3::kEps2) {
      return Eigen::Matrix3d::Identity() - 0.5 * W + (1.0 / 12.0) * W * W;
    }
    const double t = std::sqrt(t2);
    const double half = 0.5 * t;
    // coefficient of W^2: (1/t^2) (1 - (t/2) cot(t/2))
    const double c = (1.0 / t2) * (1.0 - half * std::cos(half) / std::sin(half));
    return Eigen::Matrix3d::Identity() - 0.5 * W + c * W * W;
  }

  // Exponential: xi = [rho; omega] -> SE(3).
  static SE3 exp(const Vector6d& xi) {
    const Eigen::Vector3d rho = xi.head<3>();
    const Eigen::Vector3d omega = xi.tail<3>();
    const SO3 R = SO3::exp(omega);
    const Eigen::Vector3d t = leftJacobian(omega) * rho;
    return SE3(R, t);
  }

  // Logarithm: SE(3) -> [rho; omega].
  Vector6d log() const {
    const Eigen::Vector3d omega = R_.log();
    const Eigen::Vector3d rho = leftJacobianInverse(omega) * t_;
    Vector6d xi;
    xi.head<3>() = rho;
    xi.tail<3>() = omega;
    return xi;
  }

  SE3 inverse() const {
    const SO3 Rinv = R_.inverse();
    return SE3(Rinv, -(Rinv * t_));
  }

  Eigen::Matrix4d matrix() const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = R_.matrix();
    T.topRightCorner<3, 1>() = t_;
    return T;
  }

  // Adjoint (6x6) for the [rho; omega] ordering: [[R, [t]x R], [0, R]].
  Matrix6d adjoint() const {
    const Eigen::Matrix3d R = R_.matrix();
    Matrix6d A = Matrix6d::Zero();
    A.topLeftCorner<3, 3>() = R;
    A.topRightCorner<3, 3>() = SO3::hat(t_) * R;
    A.bottomRightCorner<3, 3>() = R;
    return A;
  }

  const SO3& so3() const { return R_; }
  SO3& so3() { return R_; }
  const Eigen::Vector3d& translation() const { return t_; }
  Eigen::Vector3d& translation() { return t_; }
  Eigen::Matrix3d rotationMatrix() const { return R_.matrix(); }

  SE3 operator*(const SE3& o) const { return SE3(R_ * o.R_, t_ + R_ * o.t_); }
  Eigen::Vector3d operator*(const Eigen::Vector3d& p) const { return R_ * p + t_; }

 private:
  SO3 R_;
  Eigen::Vector3d t_;
};

}  // namespace slamko
