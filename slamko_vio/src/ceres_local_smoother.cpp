// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// CeresLocalSmoother — see ceres_local_smoother.hpp for the design + the two
// boundary conversions (pose frame, raw-IMU preintegration). Behaviour is the
// P0 baseline: the wrapped LocalBA and the preintegration call are identical to
// what VioPipeline used to do inline; the only added arithmetic is the
// T_w_c ↔ T_WB conversion (exact in double up to the SE3 quaternion seam).

#include "slamko_vio/ceres_local_smoother.hpp"

#include <limits>

#include "slamko_vio/imu_preintegration.hpp"

namespace slamko_vio {

CeresLocalSmoother::CeresLocalSmoother(const CeresLocalSmootherConfig& cfg)
    : ba_cfg_(cfg.ba), noise_(cfg.noise) {
  E_     = ba_cfg_.T_BS;
  E_inv_ = E_.inverse();
  rebuild();
}

CeresLocalSmoother::~CeresLocalSmoother() = default;

void CeresLocalSmoother::rebuild() {
  ba_ = std::make_unique<LocalBA>(ba_cfg_);
}

void CeresLocalSmoother::setImuParams(const slamko::ImuParams& p) {
  // Preintegration noise (rate_hz is NOT carried by ImuParams — keep it).
  noise_.accel_noise_density = p.accel_noise_density;
  noise_.gyro_noise_density  = p.gyro_noise_density;
  noise_.accel_random_walk   = p.accel_bias_rw;
  noise_.gyro_random_walk    = p.gyro_bias_rw;
  // LocalBA IMU-factor params. Mirrors VioPipeline's gravity-calib rebuild:
  // gravity changes, the rest is re-applied identically. T_BS is preserved.
  ba_cfg_.gravity_w     = p.gravity;
  ba_cfg_.bias_rw_gyro  = p.gyro_bias_rw;
  ba_cfg_.bias_rw_accel = p.accel_bias_rw;
  rebuild();
}

void CeresLocalSmoother::setStereoCalib(const slamko::StereoCalib& c) {
  K_.fx = static_cast<float>(c.fx);
  K_.fy = static_cast<float>(c.fy);
  K_.cx = static_cast<float>(c.cx);
  K_.cy = static_cast<float>(c.cy);
  K_.baseline_m = static_cast<float>(c.baseline);
}

void CeresLocalSmoother::setExtrinsics(const slamko::SE3& body_T_cam) {
  E_     = body_T_cam.matrix();
  E_inv_ = body_T_cam.inverse().matrix();
  ba_cfg_.T_BS = E_;
  rebuild();
}

void CeresLocalSmoother::insertKeyframe(
    double t,
    const slamko::SE3& T_WB_init,
    const Eigen::Vector3d& velocity_init,
    const slamko::ImuBias& bias_init,
    const std::vector<slamko::ImuSample>& imu_since_prev,
    const std::vector<slamko::StereoObservation>& observations) {
  // T_WB (body→world) → T_w_c (world→cam): T_w_c = E⁻¹ · T_WB⁻¹ (= (T_WB·E)⁻¹).
  const Eigen::Matrix4d T_w_c = E_inv_ * T_WB_init.matrix().inverse();

  // Unpack observations IN ORDER into LocalBA's parallel arrays. Order is
  // load-bearing: world_init seeds the new-landmark position positionally, so a
  // reorder would change which seed binds to which landmark.
  const double nan_d = std::numeric_limits<double>::quiet_NaN();
  std::vector<std::uint32_t>   lids;
  std::vector<Eigen::Vector2d> uvs_l, uvs_r;
  std::vector<Eigen::Vector3d> wps;
  lids.reserve(observations.size());
  uvs_l.reserve(observations.size());
  uvs_r.reserve(observations.size());
  wps.reserve(observations.size());
  for (const auto& o : observations) {
    lids.push_back(static_cast<std::uint32_t>(o.landmark_id));
    uvs_l.push_back(o.uv_left);
    uvs_r.push_back(o.hasRight() ? o.uv_right : Eigen::Vector2d(nan_d, nan_d));
    wps.push_back(o.world_init);
  }

  // Contract ImuBias {gyro, accel} → klt_vo ImuBias {bg, ba}.
  ImuBias bias_vio;
  bias_vio.bg = bias_init.gyro;
  bias_vio.ba = bias_init.accel;

  // IMU samples present (and enabled) ⇒ preintegrate + IMU-aware insert;
  // otherwise visual-only — the exact branch VioPipeline made on used_imu_insert.
  if (ba_cfg_.enable_imu && imu_since_prev.size() >= 2) {
    std::vector<ImuSample> s;
    s.reserve(imu_since_prev.size());
    for (const auto& m : imu_since_prev) {
      ImuSample x;
      x.t = m.timestamp;   // contract: timestamp → klt_vo: t
      x.a = m.accel;       //           accel     →         a
      x.w = m.gyro;        //           gyro      →         w
      s.push_back(x);
    }
    ImuPreintegration pi(bias_vio, noise_);
    pi.integrate_span(s.data(), s.size());
    ba_->insert_keyframe_with_imu(t, T_w_c, velocity_init, bias_vio, pi,
                                  K_, lids, uvs_l, uvs_r, wps);
  } else {
    ba_->insert_keyframe(t, T_w_c, K_, lids, uvs_l, uvs_r, wps);
  }
}

bool CeresLocalSmoother::optimize() { return ba_->solve(); }

slamko::SE3 CeresLocalSmoother::latestPose() const {
  Eigen::Matrix4d T_w_c;
  if (!ba_->latest_pose(T_w_c)) return slamko::SE3();
  // T_w_c (world→cam) → T_WB (body→world): T_WB = (E·T_w_c)⁻¹.
  return slamko::SE3(Eigen::Matrix4d((E_ * T_w_c).inverse()));
}

Eigen::Vector3d CeresLocalSmoother::latestVelocity() const {
  Eigen::Vector3d v;
  if (!ba_->latest_velocity(v)) return Eigen::Vector3d::Zero();
  return v;
}

slamko::ImuBias CeresLocalSmoother::latestBias() const {
  ImuBias b_vio;
  slamko::ImuBias b;
  if (ba_->latest_bias(b_vio)) {
    b.gyro  = b_vio.bg;
    b.accel = b_vio.ba;
  }
  return b;
}

bool CeresLocalSmoother::landmark(std::uint64_t id, Eigen::Vector3d& out) const {
  return ba_->landmark_world(static_cast<std::uint32_t>(id), out);
}

slamko::HealthSignal CeresLocalSmoother::health() const {
  // LocalBA exposes no observability probe yet; report the neutral signal.
  // Real degeneracy/eigenvalue probes land with the GTSAM backend (P1c).
  return slamko::HealthSignal{};
}

}  // namespace slamko_vio
