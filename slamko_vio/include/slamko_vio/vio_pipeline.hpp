// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// VioPipeline — the ROS-agnostic stereo-inertial VIO core. Owns the device
// buffers, the front-end stages, the tracking/BA/IMU/dead-reckoning state, and
// the swappable slamko_core::FeatureSource (detector). Inputs are plain
// ImageView + ImuSample + StereoIntrinsics; outputs are the world pose + per-
// frame stats + a HealthSignal. No ROS, no TF, no rclcpp — that is the node's
// job. This is the structural upgrade over klt_vo's monolithic node: the
// pipeline is bag/sim/real-portable and unit-testable, the node is thin glue.
//
// Behaviour is verbatim klt_vo (the validated baseline); only the I/O boundary
// changed (ROS msgs → ImageView, TF T_BS → setExtrinsics, RViz publish → node).

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <cuda_runtime.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slamko_core/feature_source.hpp"
#include "slamko_core/health.hpp"
#include "slamko_core/image_view.hpp"
#include "slamko_core/local_smoother.hpp"
#include "slamko_core/submap.hpp"

#include "slamko_vio/imu_types.hpp"
#include "slamko_vio/imu_preintegration.hpp"
#include "slamko_vio/klt_tracker.hpp"
#include "slamko_vio/local_ba.hpp"
#include "slamko_vio/pose_estimator.hpp"
#include "slamko_vio/stereo_matcher.hpp"
#include "slamko_vio/triangulator.hpp"
#include "slamko_vio/types.hpp"
#include "slamko_vio/vio_config.hpp"

namespace slamko_vio {

// Per-frame tracked feature with its stereo 3D + landmark / LMP bookkeeping.
struct StereoTrack {
  std::uint32_t id;
  float left_prev_x, left_prev_y;
  float left_curr_x, left_curr_y;
  float right_curr_x = 0.f;
  float right_curr_y = 0.f;
  bool  has_right_curr = false;
  bool  has_3d_prev    = false;
  Eigen::Vector3f point_3d_prev;
  bool  has_3d_curr    = false;
  Eigen::Vector3f point_3d_curr;
  std::uint32_t landmark_id = 0;
  std::uint32_t age = 0;
  float left_at_last_kf_x = 0.f;
  float left_at_last_kf_y = 0.f;
  bool  has_at_last_kf    = false;
  bool  from_lmp          = false;
  // 64-d XFeat descriptor captured at the track's birth (when the FeatureSource
  // provides one); copied to the landmark at KF rate → the reloc map for free.
  std::array<float, 64> desc{};
  bool  has_desc          = false;
};

// Per-frame telemetry (mirrors the klt_vo timing CSV columns).
struct FrameStats {
  float ms_total = 0, ms_klt = 0, ms_stereo = 0, ms_pnp = 0, ms_detect = 0, ms_ba = 0, ms_lmp = 0;
  int n_prev = 0, n_active = 0, n_3d_prev = 0, n_pnp_in = 0, n_stereo_match = 0;
  int n_new = 0, n_total = 0, n_ba_landmarks = 0, n_lmp_attempt = 0, n_lmp_promote = 0;
  bool ba_solved = false;
};

class VioPipeline {
 public:
  explicit VioPipeline(const VioConfig& cfg);
  // Composition-root seam (Tier-2 backend injection). The default ctor builds a
  // CeresLocalSmoother (klt_vo LocalBA) from cfg; this overload takes a
  // caller-constructed backend (e.g. slamko_fusion::GtsamLocalSmoother) so the
  // pipeline never names a concrete backend. Pass nullptr ⇒ default ceres.
  VioPipeline(const VioConfig& cfg, std::unique_ptr<slamko::LocalSmoother> smoother);
  ~VioPipeline();

  // Buffer an IMU sample (thread-safe wrt processStereo).
  void addImu(const ImuSample& s);

  // Provide the cam→body extrinsic once the caller resolves it (e.g. from TF).
  // Until provided, the IMU VI-init waits (visual-only KFs meanwhile).
  void setExtrinsics(const Eigen::Matrix4d& T_BS);

  // Run the full pipeline on one stereo pair. left/right are mono8 ImageViews.
  void processStereo(const slamko::ImageView& left, const slamko::ImageView& right,
                     double timestamp, const StereoIntrinsics& K);

  // Outputs for the node to publish.
  const Eigen::Matrix4f& worldPose() const { return world_pose_; }  // T_world_cam (cam-in-world)
  const std::vector<StereoTrack>& tracks() const { return tracks_; }
  slamko::HealthSignal health() const { return health_; }
  std::uint32_t frameIdx() const { return frame_idx_; }
  // Largest landmark id created so far. IDs are monotonic, so this is the seam
  // between pre- and post- a given instant — used by the node to partition the
  // map per never-lost submap (everything ≤ this belongs to the just-sealed one).
  std::uint64_t maxLandmarkId() const {
    std::uint64_t m = 0;
    for (const auto& kv : landmark_world_) m = std::max<std::uint64_t>(m, kv.first);
    return m;
  }
  // Never-lost submap partitioning: start a new submap epoch (called by the node on
  // BRANCH). Landmarks created from now carry the new epoch, so buildSubMap() returns
  // ONLY the active submap's own landmarks → disjoint, self-contained sealed submaps
  // (no cumulative-superset duplication in the reloc DB). Epoch stays 0 with no
  // branch, so a normal/no-loss run is byte-identical to before.
  void beginSubmap() { ++submap_epoch_; }

  // Assemble the current global map as a slamko_core::SubMap (landmarks +
  // their XFeat descriptor index). One submap for now; submap splitting is P2.
  slamko::SubMap buildSubMap() const;

 private:
  // LMP appearance-patch helpers (now ImageView-based, ROS-free).
  static constexpr int kLmpPatchSide = 11;
  static constexpr int kLmpPatchHalf = 5;
  static constexpr int kLmpPatchPx   = 121;
  static constexpr double kGravityMag = 9.81;
  bool lmp_extract_patch(const slamko::ImageView& img, float u, float v,
                         std::array<std::uint8_t, kLmpPatchPx>& out) const;
  static float lmp_ncc(const std::array<std::uint8_t, kLmpPatchPx>& a,
                       const std::array<std::uint8_t, kLmpPatchPx>& b);
  std::vector<ImuSample> drain_imu_window(double t_lo, double t_hi);
  void dump_landmarks() const;

  // ---- config-derived ----
  int image_width_, image_height_;
  int max_corners_, redetect_thr_;
  double dedup_radius_;
  int patch_size_, pyramid_lvls_;
  std::string timing_csv_;
  std::string landmark_dump_path_;

  StereoIntrinsics K_{};
  bool have_K_ = false;
  float min_depth_m_ = 0.3f, max_depth_m_ = 30.f;
  bool  use_pnp_guess_ = false;
  bool             have_world_pose_ = false;
  Eigen::Matrix4f  last_T_pp_       = Eigen::Matrix4f::Identity();

  // ---- visual-inertial ----
  bool             enable_imu_      = true;
  ImuNoise         imu_noise_;
  int              imu_init_warmup_samples_ = 80;
  LocalBA::Config  ba_cfg_;
  std::mutex                 imu_mutex_;
  std::deque<ImuSample>      imu_buffer_;
  bool             T_BS_resolved_     = false;
  Eigen::Matrix4d  T_BS_              = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d  provided_T_BS_     = Eigen::Matrix4d::Identity();
  bool             have_provided_T_BS_ = false;
  ImuBias          bias_lin_;
  Eigen::Vector3d  velocity_w_        = Eigen::Vector3d::Zero();
  bool             imu_initialised_   = false;
  bool             gravity_calibrated_ = false;
  double           last_kf_ts_        = 0.0;

  // ---- dead-reckoning (Workstream R) ----
  bool             dr_enabled_         = false;
  double           dr_max_s_           = 1.0;
  double           last_frame_ts_      = 0.0;
  bool             in_dead_reckoning_  = false;
  double           dr_start_ts_        = 0.0;
  int              dr_frames_          = 0;
  double           seq_t0_             = 0.0;
  bool             have_seq_t0_        = false;
  double           dr_force_loss_start_ = -1.0;
  double           dr_force_loss_end_   = -1.0;
  std::vector<std::pair<double, double>> dr_force_loss_windows_;  // extra [start,end)s

  // ---- visual-rotation gyro-bias init ----
  bool             bias_g_initialised_ = false;
  Eigen::Vector3d  bias_g_estimate_    = Eigen::Vector3d::Zero();
  int              bias_g_count_       = 0;
  int              bias_init_samples_  = 15;
  Eigen::Matrix3d  R_w_b_prev_         = Eigen::Matrix3d::Identity();
  bool             has_R_w_b_prev_     = false;
  double           bias_init_prev_ts_  = 0.0;

  // ---- stages ----
  std::unique_ptr<slamko::FeatureSource> feature_source_;   // swappable detector
  std::unique_ptr<KltTracker>            tracker_;
  std::unique_ptr<StereoMatcher>         matcher_;
  std::unique_ptr<PoseEstimator>         pose_estimator_;
  std::unique_ptr<slamko::LocalSmoother> smoother_;       // Tier-2 backend (ceres default)

  // ---- device buffers ----
  std::uint8_t*  d_left_  = nullptr;
  std::uint8_t*  d_right_ = nullptr;
  float*  d_prev_xy_  = nullptr;
  float*  d_curr_xy_  = nullptr;
  float*  d_right_xy_ = nullptr;
  std::int8_t*  d_status_ = nullptr;
  std::int8_t*  d_stereo_status_ = nullptr;
  cudaEvent_t   ev_start_{}, ev_stop_{};

  // ---- tracking + map state ----
  std::vector<StereoTrack> tracks_;
  std::uint32_t  next_id_ = 1;
  bool           tracker_has_prev_ = false;
  std::uint32_t  frame_idx_ = 0;
  std::ofstream  csv_out_;
  Eigen::Matrix4f world_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4d T_w_c_ = Eigen::Matrix4d::Identity();
  std::unordered_map<std::uint32_t, Eigen::Vector3d> landmark_world_;
  std::unordered_map<std::uint32_t, int> landmark_obs_count_;
  std::unordered_map<std::uint32_t, int> landmark_epoch_;  // submap epoch at creation
  int submap_epoch_ = 0;                                   // bumped by beginSubmap()
  std::unordered_map<std::uint32_t, std::array<std::uint8_t, kLmpPatchPx>> landmark_patch_;
  std::unordered_map<std::uint32_t, std::array<float, 64>> landmark_descriptors_;  // reloc map
  int            mature_obs_thr_ = 1000;
  std::uint32_t  next_lid_ = 1;
  bool   lmp_enabled_           = true;
  int    lmp_trigger_active_    = 800;
  float  lmp_snap_radius_px_    = 3.f;
  float  lmp_min_depth_m_       = 0.3f;
  float  lmp_max_depth_m_       = 25.f;
  int    lmp_max_added_per_frame_ = 400;
  int    lmp_image_border_px_   = 12;
  float  lmp_ncc_threshold_     = 0.70f;
  int    frames_since_last_kf_  = 0;
  float  kf_translation_thr_m_  = 0.15f;
  float  kf_rotation_thr_rad_   = 0.087f;
  float  kf_inlier_drop_ratio_  = 0.7f;
  float  kf_parallax_px_        = 15.0f;
  int    kf_max_frames_         = 30;
  int    first_kf_inliers_      = 0;
  Eigen::Matrix4d T_w_c_at_last_kf_ = Eigen::Matrix4d::Identity();
  bool   have_last_kf_ = false;

  slamko::HealthSignal health_;
};

}  // namespace slamko_vio
