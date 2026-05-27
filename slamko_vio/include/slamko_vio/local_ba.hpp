// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Sliding-window local bundle adjustment.
//
// Maintains the last N keyframes and the 3D landmarks visible in at least
// two of them. Each Solve() pass:
//   - Optimises all KF poses (parameterised as angle-axis + translation,
//     6 DOF each) and all landmark world positions.
//   - Anchors the oldest KF pose as the gauge (held constant).
//   - Uses reprojection residuals with a Huber loss.
//   - Returns the refined pose of the latest KF.
//
// Pose convention is world-to-camera: p_cam = R * p_world + t. This matches
// OpenCV PnP / OKVIS2-X.

#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slamko_vio/imu_preintegration.hpp"
#include "slamko_vio/imu_types.hpp"
#include "slamko_vio/types.hpp"

namespace slamko_vio {

struct Observation {
  std::uint32_t   kf_id;
  Eigen::Vector2d uv_left;
  // Right-camera pixel; NaN-x marks "no stereo match this KF" — the right
  // residual is then skipped in BA.
  Eigen::Vector2d uv_right = Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(),
                                             std::numeric_limits<double>::quiet_NaN());
};

struct Landmark {
  std::uint32_t id;
  Eigen::Vector3d point_world;       // refined by BA (Euclidean path) or
                                     // recomputed from (host KF, uv, ρ) on demand
                                     // (inverse-depth path).
  std::vector<Observation> obs;      // one entry per KF that sees it

  // OV2SLAM-style anchored inverse-depth state. The landmark is parameterised
  // by its inverse depth in a fixed *host* keyframe — the KF that first
  // observed it. Optimising 1 DOF (inv_depth) instead of 3 (point_world)
  // decouples depth from the pose Jacobian and is well-conditioned for far
  // points. Used only when LocalBA::Config::use_inv_depth = true.
  std::uint32_t host_kf_id = 0;
  double        u_host     = 0.0;
  double        v_host     = 0.0;
  double        inv_depth  = 1.0;    // BA parameter block (1 double)
};

struct KeyFrame {
  std::uint32_t id;
  double timestamp;
  // World-to-camera. p_cam = R * p_world + t.
  double angle_axis[3];
  double translation[3];

  // Sprint 4 visual-inertial state (in world frame, m/s and rad/s + m/s²).
  double velocity[3] = {0.0, 0.0, 0.0};       // body velocity in world
  double bias[6]     = {0.0, 0.0, 0.0,         // bias_g
                        0.0, 0.0, 0.0};        // bias_a

  // Preintegrated IMU samples between the *previous* KF (in insertion order)
  // and this KF. nullopt for the oldest KF in the window or when no IMU is
  // provided.
  std::optional<ImuPreintegration> preint_from_prev;

  Eigen::Matrix4d T_w_c() const;     // 4x4 world-to-camera
  void set_T_w_c(const Eigen::Matrix4d& T);
};

class LocalBA {
 public:
  struct Config {
    int    window_size           = 5;
    float  huber_threshold_px    = 1.0f;
    int    max_iterations        = 10;
    double function_tolerance    = 1.0e-4;
    bool   verbose               = false;
    int    min_observations_per_landmark = 2;
    // OKVIS-style parallax quality gate: bearing-direction std-dev across
    // observations. 0.04 ≈ 2.3° bearing spread. Below this, depth is
    // poorly observed and the landmark is excluded from BA. 0 disables.
    double parallax_quality_min          = 0.0;

    // OV2SLAM-style anchored inverse-depth parameterisation. When enabled,
    // each landmark is optimised as a 1-D inverse depth in its host KF,
    // rather than a 3-D world point. Decouples depth from pose Jacobian and
    // is well-conditioned for far points (where Euclidean Jacobians vanish).
    // Predicted ~30-50 % ATE reduction on EuRoC stereo per OV2SLAM Table II.
    bool   use_inv_depth                 = false;
    // Lower bound on inverse depth (m⁻¹). Caps depth at 1/min — prevents
    // landmarks from drifting to infinity during BA.
    double inv_depth_min                 = 1.0 / 200.0;  // 200 m cap
    double inv_depth_max                 = 1.0 / 0.10;   // 10 cm floor

    // Sprint 4 visual-inertial. enable_imu must be true and T_BS / gravity_w
    // populated for the IMU factor chain to be added. bias_rw_* are the
    // discrete-time bias random-walk standard deviations.
    // R2 stability: keep the very first KF as a permanent gauge anchor;
    // drop the second-oldest when the window slides. This preserves the
    // world-coordinate frame between consecutive BA solves so PnP-against-
    // map stays consistent.
    // fixed_gauge=true keeps KF 0 anchored permanently. Empirically this
    // causes landmark eviction cascades in narrow windows (early KFs get
    // dropped second-oldest and take their landmarks with them). Default
    // off; revisit when a wider window or marginalization lands.
    bool             fixed_gauge     = false;

    bool             enable_imu      = false;
    Eigen::Matrix4d  T_BS            = Eigen::Matrix4d::Identity(); // cam → body
    Eigen::Vector3d  gravity_w       = Eigen::Vector3d(0.0, 0.0, -9.81);
    double           bias_rw_gyro    = 1.9393e-5;   // EuRoC ADIS16448
    double           bias_rw_accel   = 3.0e-3;
  };

  LocalBA();
  explicit LocalBA(const Config& cfg);

  // Insert a new keyframe with its observed landmarks. Each landmark is
  // identified by a stable id; landmarks new to the window get an initial
  // world position from `world_positions`. `observations_right` may be empty
  // (falls back to left-cam-only BA) or carry NaN-x entries for individual
  // landmarks that had no stereo match this KF.
  std::uint32_t insert_keyframe(
      double timestamp,
      const Eigen::Matrix4d& T_w_c,
      const StereoIntrinsics& K,
      const std::vector<std::uint32_t>& landmark_ids,
      const std::vector<Eigen::Vector2d>& observations_left,
      const std::vector<Eigen::Vector2d>& observations_right,
      const std::vector<Eigen::Vector3d>& world_positions);

  // Backwards-compatible single-camera form.
  std::uint32_t insert_keyframe(
      double timestamp,
      const Eigen::Matrix4d& T_w_c,
      const StereoIntrinsics& K,
      const std::vector<std::uint32_t>& landmark_ids,
      const std::vector<Eigen::Vector2d>& observations_left,
      const std::vector<Eigen::Vector3d>& world_positions) {
    return insert_keyframe(timestamp, T_w_c, K, landmark_ids,
                           observations_left, {}, world_positions);
  }

  // Variant that also attaches the preintegrated IMU measurement from the
  // *previous* KF to this newly-inserted KF. The seed velocity (world frame)
  // and bias (gyro+accel) come from the caller's bootstrap; LocalBA will
  // optimise them. Pass enable_imu=true in the config or the preint is
  // stored but ignored at solve time.
  std::uint32_t insert_keyframe_with_imu(
      double timestamp,
      const Eigen::Matrix4d& T_w_c,
      const Eigen::Vector3d& velocity_w,
      const ImuBias&         bias,
      const ImuPreintegration& preint_from_prev,
      const StereoIntrinsics& K,
      const std::vector<std::uint32_t>& landmark_ids,
      const std::vector<Eigen::Vector2d>& observations_left,
      const std::vector<Eigen::Vector2d>& observations_right,
      const std::vector<Eigen::Vector3d>& world_positions);

  // Solve local BA on the current window. Returns true on success.
  bool solve();

  // Latest KF velocity and bias (refined by IMU-aware solve).
  bool latest_velocity(Eigen::Vector3d& v_out) const;
  bool latest_bias(ImuBias& b_out) const;

  // Latest inserted KF's refined pose (after solve()).
  bool latest_pose(Eigen::Matrix4d& T_w_c_out) const;
  bool latest_pose_id(std::uint32_t& kf_id) const;

  // Refined world position of a landmark, if it's still in the window.
  bool landmark_world(std::uint32_t lid, Eigen::Vector3d& out) const;

  int  window_size() const { return (int)kfs_.size(); }
  int  landmark_count() const { return (int)landmarks_.size(); }
  const KeyFrame* latest_kf() const;

 private:
  void drop_oldest_();      // remove the front KF, prune orphaned landmarks
  void prune_landmarks_();  // drop landmarks observed in < min_observations_per_landmark KFs

  Config cfg_;
  std::vector<KeyFrame> kfs_;
  std::unordered_map<std::uint32_t, Landmark> landmarks_;
  StereoIntrinsics K_{};
  std::uint32_t next_kf_id_ = 1;
};

}  // namespace slamko_vio
