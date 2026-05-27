// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys

#include "slamko_vio/local_ba.hpp"
#include "slamko_vio/imu_factor.hpp"

#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace slamko_vio {

// =============================================================================
// KeyFrame helpers
// =============================================================================

namespace {

inline void T_to_aa_t(const Eigen::Matrix4d& T, double aa[3], double t[3]) {
  const Eigen::AngleAxisd a(Eigen::Matrix3d(T.block<3, 3>(0, 0)));
  const Eigen::Vector3d v = a.axis() * a.angle();
  aa[0] = v.x();  aa[1] = v.y();  aa[2] = v.z();
  t[0]  = T(0, 3);  t[1] = T(1, 3);  t[2] = T(2, 3);
}

inline Eigen::Matrix4d aa_t_to_T(const double aa[3], const double t[3]) {
  const Eigen::Vector3d v(aa[0], aa[1], aa[2]);
  const double angle = v.norm();
  Eigen::Matrix3d R;
  if (angle > 1.0e-12) {
    R = Eigen::AngleAxisd(angle, v / angle).toRotationMatrix();
  } else {
    R = Eigen::Matrix3d::Identity();
  }
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T(0, 3) = t[0];  T(1, 3) = t[1];  T(2, 3) = t[2];
  return T;
}

}  // namespace

Eigen::Matrix4d KeyFrame::T_w_c() const {
  return aa_t_to_T(angle_axis, translation);
}

void KeyFrame::set_T_w_c(const Eigen::Matrix4d& T) {
  T_to_aa_t(T, angle_axis, translation);
}

// =============================================================================
// Reprojection cost (auto-diff)
// =============================================================================

namespace {

// Anchored inverse-depth stereo reprojection cost (OV2SLAM-style).
// Parameter blocks: aa_host[3], t_host[3], aa_curr[3], t_curr[3], inv_depth[1].
// Compile-time consts: u_host, v_host (landmark's pixel in host KF), the
// current KF's observation (u_l, v_l, u_r, v_r), intrinsics, baseline.
//
// Math: back-project from host into host-cam at depth 1/ρ, transform to
// world via T_w_host^{-1}, then re-project into current cam via T_w_curr.
// The host's own LEFT observation gives a structurally-zero residual; the
// host's right observation and all non-host observations are informative.
struct InvDepthStereoReprojCost {
  InvDepthStereoReprojCost(double u_host, double v_host,
                            double u_l, double v_l, double u_r, double v_r,
                            double fx, double fy, double cx, double cy,
                            double baseline_m, bool has_right)
      : u_host_(u_host), v_host_(v_host),
        u_l_(u_l), v_l_(v_l), u_r_(u_r), v_r_(v_r),
        fx_(fx), fy_(fy), cx_(cx), cy_(cy), b_(baseline_m),
        has_right_(has_right) {}

  template <typename T>
  bool operator()(const T* const aa_h, const T* const t_h,
                  const T* const aa_c, const T* const t_c,
                  const T* const rho,
                  T* residuals) const {
    // 1. Back-project (u_host, v_host) at depth 1/ρ into host-cam.
    const T inv_rho = T(1.0) / rho[0];
    T X_h_cam[3] = {
        inv_rho * (T((u_host_ - cx_) / fx_)),
        inv_rho * (T((v_host_ - cy_) / fy_)),
        inv_rho
    };
    // 2. p_h = R_wh * p_w + t_wh, so p_w = R_wh^T * (p_h - t_wh).
    T tmp[3] = { X_h_cam[0] - t_h[0],
                 X_h_cam[1] - t_h[1],
                 X_h_cam[2] - t_h[2] };
    T neg_aa_h[3] = { -aa_h[0], -aa_h[1], -aa_h[2] };
    T X_w[3];
    ceres::AngleAxisRotatePoint(neg_aa_h, tmp, X_w);
    // 3. p_c = R_wc * X_w + t_wc.
    T p_c[3];
    ceres::AngleAxisRotatePoint(aa_c, X_w, p_c);
    p_c[0] += t_c[0]; p_c[1] += t_c[1]; p_c[2] += t_c[2];
    // 4. Project + residual.
    if (p_c[2] < T(1.0e-3)) {
      residuals[0] = residuals[1] = residuals[2] = residuals[3] = T(0.0);
      return false;
    }
    const T inv_z = T(1.0) / p_c[2];
    residuals[0] = T(fx_) * p_c[0] * inv_z + T(cx_) - T(u_l_);
    residuals[1] = T(fy_) * p_c[1] * inv_z + T(cy_) - T(v_l_);
    if (has_right_) {
      residuals[2] = T(fx_) * (p_c[0] - T(b_)) * inv_z + T(cx_) - T(u_r_);
      residuals[3] = T(fy_) *  p_c[1]          * inv_z + T(cy_) - T(v_r_);
    } else {
      residuals[2] = T(0.0);
      residuals[3] = T(0.0);
    }
    return true;
  }

  static ceres::CostFunction* Create(double u_host, double v_host,
                                     double u_l, double v_l,
                                     double u_r, double v_r,
                                     double fx, double fy,
                                     double cx, double cy,
                                     double baseline_m,
                                     bool has_right) {
    return new ceres::AutoDiffCostFunction<
        InvDepthStereoReprojCost, 4, 3, 3, 3, 3, 1>(
        new InvDepthStereoReprojCost(u_host, v_host, u_l, v_l, u_r, v_r,
                                     fx, fy, cx, cy, baseline_m, has_right));
  }

 private:
  double u_host_, v_host_;
  double u_l_, v_l_, u_r_, v_r_;
  double fx_, fy_, cx_, cy_, b_;
  bool   has_right_;
};

// Host-frame inverse-depth cost: when the observation IS in the host KF
// itself, the left residual is structurally 0 (by construction of the
// back-projection). Only the right-cam disparity residual is informative.
// Ceres forbids passing the same parameter block twice as separate args,
// so we keep this cost distinct from the cross-frame one and take only
// (aa_host, t_host, rho).
struct InvDepthHostReprojCost {
  InvDepthHostReprojCost(double u_host, double v_host,
                          double u_r, double v_r,
                          double fx, double fy, double cx, double cy,
                          double baseline_m, bool has_right)
      : u_host_(u_host), v_host_(v_host), u_r_(u_r), v_r_(v_r),
        fx_(fx), fy_(fy), cx_(cx), cy_(cy), b_(baseline_m),
        has_right_(has_right) {}

  template <typename T>
  bool operator()(const T* const /*aa_h*/, const T* const /*t_h*/,
                  const T* const rho, T* residuals) const {
    // Host-frame back-projection re-projects to (u_host, v_host) trivially,
    // so the LEFT residual is zero by construction. The host's RIGHT pixel
    // observes the same point at disparity fx · baseline · ρ, which IS
    // informative for ρ.
    residuals[0] = T(0.0);
    residuals[1] = T(0.0);
    if (has_right_) {
      // Predicted right-cam u: u_h - fx · baseline · ρ (rectified stereo).
      const T u_r_pred = T(u_host_) - T(fx_ * b_) * rho[0];
      // Predicted right-cam v: same as host-left v.
      const T v_r_pred = T(v_host_);
      residuals[2] = u_r_pred - T(u_r_);
      residuals[3] = v_r_pred - T(v_r_);
    } else {
      residuals[2] = T(0.0);
      residuals[3] = T(0.0);
    }
    return true;
  }

  static ceres::CostFunction* Create(double u_host, double v_host,
                                     double u_r, double v_r,
                                     double fx, double fy,
                                     double cx, double cy,
                                     double baseline_m,
                                     bool has_right) {
    return new ceres::AutoDiffCostFunction<
        InvDepthHostReprojCost, 4, 3, 3, 1>(
        new InvDepthHostReprojCost(u_host, v_host, u_r, v_r,
                                   fx, fy, cx, cy, baseline_m, has_right));
  }

 private:
  double u_host_, v_host_;
  double u_r_, v_r_;
  double fx_, fy_, cx_, cy_, b_;
  bool   has_right_;
};

// Both-camera reprojection cost. Residual block size is fixed at 4 so each
// observation contributes a uniform LM weight; if the right pixel was not
// matched this KF, the right pair of residuals is zeroed.
struct StereoReprojectionCost {
  StereoReprojectionCost(double u_l, double v_l, double u_r, double v_r,
                         double fx, double fy, double cx, double cy,
                         double baseline_m, bool has_right)
      : u_l_(u_l), v_l_(v_l), u_r_(u_r), v_r_(v_r),
        fx_(fx), fy_(fy), cx_(cx), cy_(cy), b_(baseline_m),
        has_right_(has_right) {}

  template <typename T>
  bool operator()(const T* const aa, const T* const t,
                  const T* const pt_world, T* residuals) const {
    T p_cam[3];
    ceres::AngleAxisRotatePoint(aa, pt_world, p_cam);
    p_cam[0] += t[0];
    p_cam[1] += t[1];
    p_cam[2] += t[2];
    if (p_cam[2] < T(1.0e-3)) {
      residuals[0] = residuals[1] = residuals[2] = residuals[3] = T(0.0);
      return false;
    }
    const T inv_z = T(1.0) / p_cam[2];
    residuals[0] = T(fx_) * p_cam[0] * inv_z + T(cx_) - T(u_l_);
    residuals[1] = T(fy_) * p_cam[1] * inv_z + T(cy_) - T(v_l_);
    if (has_right_) {
      residuals[2] = T(fx_) * (p_cam[0] - T(b_)) * inv_z + T(cx_) - T(u_r_);
      residuals[3] = T(fy_) *  p_cam[1]          * inv_z + T(cy_) - T(v_r_);
    } else {
      residuals[2] = T(0.0);
      residuals[3] = T(0.0);
    }
    return true;
  }

  static ceres::CostFunction* Create(double u_l, double v_l,
                                     double u_r, double v_r,
                                     double fx, double fy,
                                     double cx, double cy,
                                     double baseline_m,
                                     bool has_right) {
    return new ceres::AutoDiffCostFunction<StereoReprojectionCost, 4, 3, 3, 3>(
        new StereoReprojectionCost(u_l, v_l, u_r, v_r,
                                   fx, fy, cx, cy, baseline_m, has_right));
  }

 private:
  double u_l_, v_l_, u_r_, v_r_;
  double fx_, fy_, cx_, cy_, b_;
  bool   has_right_;
};

}  // namespace

// =============================================================================
// LocalBA lifetime
// =============================================================================

LocalBA::LocalBA() : LocalBA(Config{}) {}

LocalBA::LocalBA(const Config& cfg) : cfg_(cfg) {
  if (cfg_.window_size < 2)
    throw std::invalid_argument("LocalBA: window_size must be >= 2");
}

std::uint32_t LocalBA::insert_keyframe_with_imu(
    double timestamp,
    const Eigen::Matrix4d& T_w_c,
    const Eigen::Vector3d& velocity_w,
    const ImuBias&         bias,
    const ImuPreintegration& preint_from_prev,
    const StereoIntrinsics& K,
    const std::vector<std::uint32_t>& landmark_ids,
    const std::vector<Eigen::Vector2d>& observations_left,
    const std::vector<Eigen::Vector2d>& observations_right,
    const std::vector<Eigen::Vector3d>& world_positions) {
  // Insert the visual-only KF first, then patch in the IMU state.
  const std::uint32_t kid = insert_keyframe(timestamp, T_w_c, K, landmark_ids,
                                            observations_left,
                                            observations_right,
                                            world_positions);
  auto& kf = kfs_.back();
  kf.velocity[0] = velocity_w.x();
  kf.velocity[1] = velocity_w.y();
  kf.velocity[2] = velocity_w.z();
  kf.bias[0] = bias.bg.x();  kf.bias[1] = bias.bg.y();  kf.bias[2] = bias.bg.z();
  kf.bias[3] = bias.ba.x();  kf.bias[4] = bias.ba.y();  kf.bias[5] = bias.ba.z();
  if ((int)kfs_.size() >= 2) {
    kf.preint_from_prev = preint_from_prev;
  }
  return kid;
}

std::uint32_t LocalBA::insert_keyframe(
    double timestamp,
    const Eigen::Matrix4d& T_w_c,
    const StereoIntrinsics& K,
    const std::vector<std::uint32_t>& landmark_ids,
    const std::vector<Eigen::Vector2d>& observations_left,
    const std::vector<Eigen::Vector2d>& observations_right,
    const std::vector<Eigen::Vector3d>& world_positions) {
  K_ = K;
  KeyFrame kf;
  kf.id = next_kf_id_++;
  kf.timestamp = timestamp;
  kf.set_T_w_c(T_w_c);
  kfs_.push_back(kf);

  if ((int)kfs_.size() > cfg_.window_size) drop_oldest_();

  const std::uint32_t kid = kfs_.back().id;
  const bool right_provided = (observations_right.size() == landmark_ids.size());
  const Eigen::Vector2d nan_uv(std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN());

  // Pre-compute the new KF pose (for inv-depth initialisation of new landmarks).
  const Eigen::Matrix3d R_wc_new = T_w_c.block<3, 3>(0, 0);
  const Eigen::Vector3d t_wc_new = T_w_c.block<3, 1>(0, 3);

  for (std::size_t i = 0; i < landmark_ids.size(); ++i) {
    const std::uint32_t lid = landmark_ids[i];
    Observation o;
    o.kf_id     = kid;
    o.uv_left   = observations_left[i];
    o.uv_right  = right_provided ? observations_right[i] : nan_uv;
    auto it = landmarks_.find(lid);
    if (it == landmarks_.end()) {
      Landmark lm;
      lm.id = lid;
      lm.point_world = world_positions[i];
      // Inv-depth seed: this KF becomes the host. Depth = z-component of the
      // landmark in this KF's cam coords = (R_wc * p_w + t_wc).z().
      lm.host_kf_id = kid;
      lm.u_host     = observations_left[i].x();
      lm.v_host     = observations_left[i].y();
      const Eigen::Vector3d p_cam = R_wc_new * world_positions[i] + t_wc_new;
      const double z = std::max(1.0e-3, p_cam.z());
      lm.inv_depth  = 1.0 / z;
      lm.obs.push_back(o);
      landmarks_.emplace(lid, std::move(lm));
    } else {
      it->second.obs.push_back(o);
    }
  }
  return kid;
}

void LocalBA::drop_oldest_() {
  if (kfs_.empty()) return;
  std::size_t idx_to_drop = 0;
  if (cfg_.fixed_gauge && kfs_.size() >= 2) idx_to_drop = 1;
  const std::uint32_t dropped_id = kfs_[idx_to_drop].id;

  // For inverse-depth: cache the dropped KF's pose so we can re-anchor
  // landmarks whose host is being dropped.
  Eigen::Matrix4d T_wc_dropped = Eigen::Matrix4d::Identity();
  if (cfg_.use_inv_depth) T_wc_dropped = kfs_[idx_to_drop].T_w_c();

  kfs_.erase(kfs_.begin() + idx_to_drop);

  for (auto it = landmarks_.begin(); it != landmarks_.end(); ) {
    auto& lm = it->second;
    auto& obs = lm.obs;
    obs.erase(std::remove_if(obs.begin(), obs.end(),
                             [dropped_id](const Observation& o) {
                               return o.kf_id == dropped_id;
                             }),
              obs.end());
    if (obs.empty()) { it = landmarks_.erase(it); continue; }

    // Inv-depth landmark whose host KF is being dropped: re-anchoring
    // empirically caused 2-3× ATE regression (bench 2026-05-18). Drop the
    // landmark instead — cleaner. Most KLT tracks die within the window's
    // 50-frame lifetime anyway, so this cost is small.
    if (cfg_.use_inv_depth && lm.host_kf_id == dropped_id) {
      it = landmarks_.erase(it);
      continue;
    }
    ++it;
  }
}

void LocalBA::prune_landmarks_() {
  for (auto it = landmarks_.begin(); it != landmarks_.end(); ) {
    if ((int)it->second.obs.size() < cfg_.min_observations_per_landmark)
      it = landmarks_.erase(it);
    else
      ++it;
  }
}

bool LocalBA::solve() {
  if ((int)kfs_.size() < 2) return false;
  prune_landmarks_();
  if (landmarks_.empty()) return false;

  ceres::Problem problem;
  // CauchyLoss(σ) — heavier outlier attenuation than Huber. OKVIS2-X uses
  // CauchyLoss(3.0) on all reprojection residuals. Below σ the residual is
  // ≈ linear; above it shrinks as log(1 + r²/σ²).
  std::unique_ptr<ceres::LossFunction> loss(
      new ceres::CauchyLoss(cfg_.huber_threshold_px));

  // KF pose params
  for (auto& kf : kfs_) {
    problem.AddParameterBlock(kf.angle_axis, 3);
    problem.AddParameterBlock(kf.translation, 3);
    if (cfg_.enable_imu) {
      problem.AddParameterBlock(kf.velocity, 3);
      problem.AddParameterBlock(kf.bias,     6);
    }
  }
  // Fix oldest KF pose (and VI state) as gauge.
  problem.SetParameterBlockConstant(kfs_.front().angle_axis);
  problem.SetParameterBlockConstant(kfs_.front().translation);
  if (cfg_.enable_imu) {
    problem.SetParameterBlockConstant(kfs_.front().velocity);
    problem.SetParameterBlockConstant(kfs_.front().bias);
  }

  // IMU factor chain between consecutive KFs (Forster 2017).
  if (cfg_.enable_imu) {
    for (std::size_t i = 1; i < kfs_.size(); ++i) {
      auto& kf_i = kfs_[i - 1];
      auto& kf_j = kfs_[i];
      if (!kf_j.preint_from_prev.has_value()) continue;
      auto* cost = ImuFactor::Create(*kf_j.preint_from_prev,
                                     cfg_.T_BS,
                                     cfg_.gravity_w,
                                     cfg_.bias_rw_gyro,
                                     cfg_.bias_rw_accel);
      problem.AddResidualBlock(cost, /*loss=*/nullptr,
                               kf_i.angle_axis, kf_i.translation,
                               kf_i.velocity,   kf_i.bias,
                               kf_j.angle_axis, kf_j.translation,
                               kf_j.velocity,   kf_j.bias);
    }
  }

  std::unordered_map<std::uint32_t, KeyFrame*> kf_by_id;
  kf_by_id.reserve(kfs_.size());
  for (auto& kf : kfs_) kf_by_id.emplace(kf.id, &kf);

  // OKVIS-style parallax gating: a landmark contributes to BA only when its
  // bearing direction spans a meaningful angle across observations. Without
  // sufficient parallax, depth is unobservable and the optimizer can move
  // the point to arbitrary positions, biasing pose.
  auto compute_parallax_quality = [&](const Landmark& lm) -> double {
    if ((int)lm.obs.size() < 2) return 0.0;
    // Bearing direction for each observation (in world frame).
    std::vector<Eigen::Vector3d> dirs;
    dirs.reserve(lm.obs.size());
    for (const auto& o : lm.obs) {
      auto kit = kf_by_id.find(o.kf_id);
      if (kit == kf_by_id.end()) continue;
      const KeyFrame* kf = kit->second;
      // Camera optical centre in world: c = -R_w_c^T · t_w_c.
      const Eigen::Matrix4d T_wc = kf->T_w_c();
      const Eigen::Matrix3d R_wc = T_wc.block<3, 3>(0, 0);
      const Eigen::Vector3d t_wc = T_wc.block<3, 1>(0, 3);
      const Eigen::Vector3d c_w  = -R_wc.transpose() * t_wc;
      Eigen::Vector3d d_w = (lm.point_world - c_w);
      const double n = d_w.norm();
      if (n < 1.0e-6) continue;
      dirs.push_back(d_w / n);
    }
    if (dirs.size() < 2) return 0.0;
    // Std dev of bearing direction vector across observations — OKVIS metric.
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& d : dirs) mean += d;
    mean /= (double)dirs.size();
    double var = 0.0;
    for (const auto& d : dirs) var += (d - mean).squaredNorm();
    var /= (double)dirs.size();
    return std::sqrt(var);
  };

  for (auto& [lid, lm] : landmarks_) {
    if ((int)lm.obs.size() < cfg_.min_observations_per_landmark) continue;
    if (cfg_.parallax_quality_min > 0.0) {
      const double q = compute_parallax_quality(lm);
      if (q < cfg_.parallax_quality_min) continue;
    }
    if (cfg_.use_inv_depth) {
      // Locate the host KF — must be in the current window.
      auto hit = kf_by_id.find(lm.host_kf_id);
      if (hit == kf_by_id.end()) continue;
      KeyFrame* host_kf = hit->second;
      problem.AddParameterBlock(&lm.inv_depth, 1);
      problem.SetParameterLowerBound(&lm.inv_depth, 0, cfg_.inv_depth_min);
      problem.SetParameterUpperBound(&lm.inv_depth, 0, cfg_.inv_depth_max);
      for (const auto& o : lm.obs) {
        auto kit = kf_by_id.find(o.kf_id);
        if (kit == kf_by_id.end()) continue;
        const bool has_right = std::isfinite(o.uv_right.x()) &&
                               std::isfinite(o.uv_right.y());
        if (o.kf_id == lm.host_kf_id) {
          // Host KF observation — left residual is structurally zero; only
          // the right-cam disparity constrains ρ. Cost takes (aa_h, t_h, ρ).
          if (!has_right) continue;  // nothing to constrain ρ here
          auto* cost = InvDepthHostReprojCost::Create(
              lm.u_host, lm.v_host,
              o.uv_right.x(), o.uv_right.y(),
              K_.fx, K_.fy, K_.cx, K_.cy, K_.baseline_m,
              has_right);
          problem.AddResidualBlock(cost, loss.get(),
                                   host_kf->angle_axis,
                                   host_kf->translation,
                                   &lm.inv_depth);
        } else {
          auto* cost = InvDepthStereoReprojCost::Create(
              lm.u_host, lm.v_host,
              o.uv_left.x(),  o.uv_left.y(),
              o.uv_right.x(), o.uv_right.y(),
              K_.fx, K_.fy, K_.cx, K_.cy, K_.baseline_m,
              has_right);
          problem.AddResidualBlock(cost, loss.get(),
                                   host_kf->angle_axis,
                                   host_kf->translation,
                                   kit->second->angle_axis,
                                   kit->second->translation,
                                   &lm.inv_depth);
        }
      }
    } else {
      problem.AddParameterBlock(lm.point_world.data(), 3);
      for (const auto& o : lm.obs) {
        auto kit = kf_by_id.find(o.kf_id);
        if (kit == kf_by_id.end()) continue;
        const bool has_right = std::isfinite(o.uv_right.x()) &&
                               std::isfinite(o.uv_right.y());
        auto* cost = StereoReprojectionCost::Create(
            o.uv_left.x(),  o.uv_left.y(),
            o.uv_right.x(), o.uv_right.y(),
            K_.fx, K_.fy, K_.cx, K_.cy, K_.baseline_m,
            has_right);
        problem.AddResidualBlock(cost, loss.get(),
                                 kit->second->angle_axis,
                                 kit->second->translation,
                                 lm.point_world.data());
      }
    }
  }

  ceres::Solver::Options opts;
  opts.linear_solver_type = ceres::SPARSE_SCHUR;
  opts.max_num_iterations = cfg_.max_iterations;
  opts.function_tolerance = cfg_.function_tolerance;
  opts.minimizer_progress_to_stdout = cfg_.verbose;
  opts.num_threads = 1;

  ceres::Solver::Summary summary;
  ceres::Solve(opts, &problem, &summary);
  // Loss is consumed by problem; release ownership so we don't double-free.
  (void)loss.release();

  // Inv-depth path: sync each landmark's Euclidean world position from its
  // refined inverse depth + (refined) host pose, so node-side queries and the
  // parallax gate see a consistent point_world.
  if (cfg_.use_inv_depth) {
    for (auto& [lid, lm] : landmarks_) {
      const KeyFrame* host_kf = nullptr;
      for (const auto& k : kfs_) {
        if (k.id == lm.host_kf_id) { host_kf = &k; break; }
      }
      if (host_kf == nullptr) continue;
      const double inv_rho = 1.0 / std::max(lm.inv_depth, cfg_.inv_depth_min);
      const Eigen::Vector3d p_h_cam(
          inv_rho * ((lm.u_host - K_.cx) / K_.fx),
          inv_rho * ((lm.v_host - K_.cy) / K_.fy),
          inv_rho);
      const Eigen::Matrix4d T_wc = host_kf->T_w_c();
      lm.point_world = T_wc.block<3, 3>(0, 0).transpose() *
                       (p_h_cam - T_wc.block<3, 1>(0, 3));
    }
  }

  return summary.IsSolutionUsable();
}

bool LocalBA::latest_pose(Eigen::Matrix4d& T_out) const {
  if (kfs_.empty()) return false;
  T_out = kfs_.back().T_w_c();
  return true;
}

bool LocalBA::latest_pose_id(std::uint32_t& kf_id) const {
  if (kfs_.empty()) return false;
  kf_id = kfs_.back().id;
  return true;
}

bool LocalBA::latest_velocity(Eigen::Vector3d& v_out) const {
  if (kfs_.empty()) return false;
  v_out = Eigen::Vector3d(kfs_.back().velocity[0], kfs_.back().velocity[1],
                          kfs_.back().velocity[2]);
  return true;
}

bool LocalBA::latest_bias(ImuBias& b_out) const {
  if (kfs_.empty()) return false;
  b_out.bg = Eigen::Vector3d(kfs_.back().bias[0], kfs_.back().bias[1],
                             kfs_.back().bias[2]);
  b_out.ba = Eigen::Vector3d(kfs_.back().bias[3], kfs_.back().bias[4],
                             kfs_.back().bias[5]);
  return true;
}

bool LocalBA::landmark_world(std::uint32_t lid, Eigen::Vector3d& out) const {
  auto it = landmarks_.find(lid);
  if (it == landmarks_.end()) return false;
  out = it->second.point_world;
  return true;
}

const KeyFrame* LocalBA::latest_kf() const {
  if (kfs_.empty()) return nullptr;
  return &kfs_.back();
}

}  // namespace slamko_vio
