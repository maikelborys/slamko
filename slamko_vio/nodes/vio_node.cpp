// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// slamko_vio_node — sprint 2 stereo VO node.
//
// Pipeline per stereo frame:
//   1. Upload left, right mono8 to device.
//   2. KLT track previous left tracks → current left (status filter).
//   3. Stereo match current left → current right (NCC search along epipolar).
//   4. Triangulate stereo-matched pairs → 3D in current camera coords.
//   5. With (3D_prev, 2D_curr) pairs run PnP RANSAC → T_{prev->cur}.
//   6. Update accumulated world pose, publish nav_msgs/Odometry.
//   7. Detect new Shi-Tomasi corners (grid mode), stereo-match, triangulate,
//      add to track pool.
//
// Coordinate convention: left optical frame (x-right, y-down, z-forward).
// world_pose_ is the camera in the world frame: pose_t = pose_{t-1} *
// T_{cur->prev} = pose_{t-1} * inv(T_{prev->cur}).

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <vector>

#include <cuda_runtime.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "slamko_vio/shitomasi.hpp"
#include "slamko_vio/klt_tracker.hpp"
#include "slamko_vio/stereo_matcher.hpp"
#include "slamko_vio/triangulator.hpp"
#include "slamko_vio/pose_estimator.hpp"
#include "slamko_vio/local_ba.hpp"
#include "slamko_vio/imu_preintegration.hpp"
#include "slamko_vio/imu_types.hpp"
#include "slamko_vio/types.hpp"

#include <mutex>

#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace {

struct StereoTrack {
  std::uint32_t id;                   // track id (never reused)
  float left_prev_x, left_prev_y;
  float left_curr_x, left_curr_y;
  float right_curr_x = 0.f;
  float right_curr_y = 0.f;
  bool  has_right_curr = false;       // right-cam stereo match this frame
  bool  has_3d_prev    = false;       // 3D from PREVIOUS frame's stereo, PnP input
  Eigen::Vector3f point_3d_prev;      // in previous-frame cam coords
  bool  has_3d_curr    = false;       // 3D from CURRENT frame's stereo, becomes prev
  Eigen::Vector3f point_3d_curr;      // in current-frame cam coords
  std::uint32_t landmark_id = 0;      // 0 == not yet in BA world map
  std::uint32_t age = 0;
  // OV2SLAM parallax-triggered KF: snapshot the left-cam pixel at the last
  // KF insertion. Median |left_curr - left_at_last_kf| across all tracks is
  // the parallax that triggers the next KF.
  float left_at_last_kf_x = 0.f;
  float left_at_last_kf_y = 0.f;
  bool  has_at_last_kf    = false;
  // LMP (Local Map Projection): set true when this track was created by
  // re-acquiring a landmark from landmark_world_ rather than via Shi-Tomasi.
  // Used as a fallback flag — if this frame's stereo match fails on the
  // recovered pixel, point_3d_curr is restored from the world map so PnP
  // still has a 3D anchor next frame.
  bool  from_lmp          = false;
};

}  // namespace

class KltVoNode : public rclcpp::Node {
 public:
  KltVoNode() : Node("slamko_vio_node") {
    image_width_  = declare_parameter<int>("image_width", 752);
    image_height_ = declare_parameter<int>("image_height", 480);
    max_corners_  = declare_parameter<int>("max_corners", 1500);
    redetect_thr_ = declare_parameter<int>("redetect_threshold", 1500);
    dedup_radius_ = declare_parameter<double>("dedup_radius_px", 5.0);
    patch_size_   = declare_parameter<int>("patch_size", 9);
    pyramid_lvls_ = declare_parameter<int>("pyramid_levels", 4);
    timing_csv_   = declare_parameter<std::string>("timing_csv_path", "");
    odom_frame_   = declare_parameter<std::string>("odom_frame_id", "slamko_vio_world");
    child_frame_  = declare_parameter<std::string>("child_frame_id", "slamko_vio_cam");
    publish_tf_   = declare_parameter<bool>("publish_tf", true);
    // Optional: dump the final BA landmark world map (id x y z obs_count) at
    // shutdown for offline viz. Empty = disabled. PLY-friendly CSV.
    landmark_dump_path_ = declare_parameter<std::string>("landmark_dump_path", "");

    slamko_vio::ShiTomasiDetector::Config scfg;
    scfg.max_corners = max_corners_;
    scfg.min_quality = 1.0e-4f;
    scfg.nms_radius  = 3;
    scfg.border      = 8;
    scfg.grid_cols   = declare_parameter<int>("grid_cols", 8);
    scfg.grid_rows   = declare_parameter<int>("grid_rows", 6);
    scfg.k_per_cell  = declare_parameter<int>("k_per_cell", 32);
    detector_ = std::make_unique<slamko_vio::ShiTomasiDetector>(
        image_width_, image_height_, scfg);

    slamko_vio::KltTracker::Config kcfg;
    kcfg.pyramid_levels = pyramid_lvls_;
    kcfg.patch_size     = patch_size_;
    tracker_ = std::make_unique<slamko_vio::KltTracker>(
        image_width_, image_height_, kcfg);

    slamko_vio::StereoMatcher::Config mcfg;
    mcfg.patch_size    = declare_parameter<int>("stereo_patch_size", 11);
    mcfg.min_disparity = declare_parameter<int>("min_disparity", 1);
    mcfg.max_disparity = declare_parameter<int>("max_disparity", 100);
    mcfg.ncc_threshold = (float)declare_parameter<double>("stereo_ncc_thr", 0.6);
    mcfg.border        = declare_parameter<int>("stereo_border", 12);
    matcher_ = std::make_unique<slamko_vio::StereoMatcher>(
        image_width_, image_height_, mcfg);

    slamko_vio::PoseEstimator::Config pcfg;
    // R4: tighter RANSAC threshold; 1.5 px was ~30× the ~0.02 px stereo
    // sub-pixel noise floor and let mis-tracked points into LM refine.
    pcfg.reprojection_threshold_px = (float)declare_parameter<double>("pnp_reproj_thr_px", 0.8);
    pcfg.max_ransac_iters          = declare_parameter<int>("pnp_max_iters", 200);
    pcfg.min_inliers               = declare_parameter<int>("pnp_min_inliers", 12);
    // R4 two-pass: after the first Ceres refine, re-prune to this pixel
    // threshold and refine once more. 0 disables. Default to the same value
    // as RANSAC threshold for now.
    pcfg.refine_pixel_threshold    = (float)declare_parameter<double>("pnp_refine_thr_px", 0.8);
    pcfg.use_cuda_ransac           = declare_parameter<bool>("pnp_use_cuda", false);
    pcfg.cuda_max_points           = declare_parameter<int>("pnp_cuda_max_points", 4096);
    // Sweet-spot default empirically: iters=5 with 2-pass refine. On V1_03
    // it beats iters=10 on BOTH axes (ATE 0.568 vs 0.900, fps 279 vs 161);
    // on MH_01 ATE is unchanged. iters>5 just wastes cycles in Ceres
    // convergence checks.
    pcfg.lm_max_iters              = declare_parameter<int>("pnp_lm_max_iters", 5);
    pcfg.refine_second_pass        = declare_parameter<bool>("pnp_refine_second_pass", true);
    pose_estimator_ = std::make_unique<slamko_vio::PoseEstimator>(pcfg);

    slamko_vio::LocalBA::Config bcfg;
    bcfg.window_size        = declare_parameter<int>("ba_window_size", 10);
    // Loss scale for Cauchy (was Huber 1.0 px). OKVIS2-X uses 3.0.
    bcfg.huber_threshold_px = (float)declare_parameter<double>("ba_huber_px", 3.0);
    bcfg.max_iterations     = declare_parameter<int>("ba_max_iters", 30);
    bcfg.function_tolerance = declare_parameter<double>("ba_function_tol", 1.0e-6);
    bcfg.min_observations_per_landmark =
        declare_parameter<int>("ba_min_obs_per_landmark", 2);
    // IMU integration: gyro bias is initialised from the visual rotation
    // chain (mean(ω_imu - ω_visual) over the first 15 KF pairs after T_BS
    // and gravity are calibrated). Accel bias starts at 0; BA refines.
    enable_imu_           = declare_parameter<bool>("enable_imu", true);
    // OV2SLAM anchored inverse-depth landmark parameterisation.
    bcfg.use_inv_depth    = declare_parameter<bool>("ba_use_inv_depth", true);
    bcfg.enable_imu       = enable_imu_;
    bcfg.bias_rw_gyro     = declare_parameter<double>("imu_bias_rw_gyro",  1.9393e-5);
    bcfg.bias_rw_accel    = declare_parameter<double>("imu_bias_rw_accel", 3.0e-3);
    imu_noise_.gyro_noise_density  =
        declare_parameter<double>("imu_gyro_noise_density",  1.6968e-4);
    imu_noise_.accel_noise_density =
        declare_parameter<double>("imu_accel_noise_density", 2.0e-3);
    imu_noise_.rate_hz =
        declare_parameter<double>("imu_rate_hz", 200.0);
    imu_init_warmup_samples_ =
        declare_parameter<int>("imu_init_warmup_samples", 80);  // ~0.4 s @ 200Hz
    // Workstream R: IMU dead-reckoning on tracking loss.
    // Default OFF: dead-reckoning is mechanism-complete but open-loop drift is
    // dominated by the init gravity-DIRECTION error (~16° from motion-
    // contaminated calib) → diverges on multi-second gaps. Needs clean gravity-
    // direction init (low-motion-gated / rotation-compensated window) before it
    // is a net win. See docs/13. Enable explicitly to experiment.
    dr_enabled_ = declare_parameter<bool>("dr_enabled", false);
    dr_max_s_   = declare_parameter<double>("dr_max_s", 1.0);
    dr_force_loss_start_ =
        declare_parameter<double>("dr_force_loss_start_s", -1.0);
    dr_force_loss_end_ =
        declare_parameter<double>("dr_force_loss_end_s", -1.0);
    // T_BS (cam-in-body): set when the static tf imu→left_rect arrives.
    bcfg.T_BS = Eigen::Matrix4d::Identity();
    bcfg.gravity_w = Eigen::Vector3d(0.0, 0.0, -9.81);
    ba_cfg_ = bcfg;
    local_ba_ = std::make_unique<slamko_vio::LocalBA>(bcfg);

    kf_translation_thr_m_ = (float)declare_parameter<double>("kf_translation_m", 0.15);
    kf_rotation_thr_rad_  = (float)declare_parameter<double>("kf_rotation_rad", 0.087);  // ~5°
    kf_inlier_drop_ratio_ = (float)declare_parameter<double>("kf_inlier_drop", 0.7);
    // Fix A (OV2SLAM): median rotation-uncompensated parallax trigger and a
    // frame-count safety net.
    kf_parallax_px_       = (float)declare_parameter<double>("kf_parallax_px", 15.0);
    kf_max_frames_        = declare_parameter<int>("kf_max_frames", 30);
    // R2 mature-landmark gating: empirically did not improve ATE at thr={1,2,3}
    // and kf_interval={2,5}. Left in the code path; effectively disabled by
    // default with a very high threshold. Re-enable if a different BA strategy
    // (marginalization / persistent gauge) is implemented.
    mature_obs_thr_       = declare_parameter<int>("mature_obs_thr", 1000);

    min_depth_m_ = (float)declare_parameter<double>("min_depth_m", 0.3);
    // Tight max depth: stereo depth uncertainty grows as Z²/(fx·b)·σ_d, so
    // points beyond ~15 m at EuRoC's geometry have several-cm depth noise
    // that destabilises frame-to-frame PnP.
    max_depth_m_ = (float)declare_parameter<double>("max_depth_m", 30.0);
    // useExtrinsicGuess forces SOLVEPNP_ITERATIVE which proved noisier than
    // EPnP on EuRoC; default false until we revisit.
    use_pnp_guess_ = declare_parameter<bool>("pnp_use_guess", false);

    // -------- Local map projection (ORB-SLAM3-style re-acquisition) ---------
    // When KLT survival drops below `lmp_trigger_active_`, project all
    // landmark_world_ entries that aren't already being tracked into the
    // current camera, snap to the closest fresh Shi-Tomasi corner within
    // `lmp_snap_radius_px_`, and create a StereoTrack with the existing
    // landmark_id + a known initial 3D from the world map. The new track
    // contributes to PnP starting next frame (after KLT has confirmed it).
    lmp_enabled_              = declare_parameter<bool>("lmp_enabled", true);
    lmp_trigger_active_       = declare_parameter<int>("lmp_trigger_active", 800);
    lmp_snap_radius_px_       = (float)declare_parameter<double>("lmp_snap_radius_px", 3.0);
    lmp_min_depth_m_          = (float)declare_parameter<double>("lmp_min_depth_m", 0.3);
    lmp_max_depth_m_          = (float)declare_parameter<double>("lmp_max_depth_m", 25.0);
    lmp_max_added_per_frame_  = declare_parameter<int>("lmp_max_added_per_frame", 400);
    lmp_image_border_px_      = declare_parameter<int>("lmp_image_border_px", 12);
    lmp_ncc_threshold_        = (float)declare_parameter<double>("lmp_ncc_threshold", 0.70);

    const std::size_t img_bytes = (std::size_t)image_width_ * image_height_;
    cudaMalloc(&d_left_,    img_bytes);
    cudaMalloc(&d_right_,   img_bytes);
    cudaMalloc(&d_prev_xy_, sizeof(float) * 2 * max_corners_);
    cudaMalloc(&d_curr_xy_, sizeof(float) * 2 * max_corners_);
    cudaMalloc(&d_right_xy_, sizeof(float) * 2 * max_corners_);
    cudaMalloc(&d_status_,  sizeof(std::int8_t) * max_corners_);
    cudaMalloc(&d_stereo_status_, sizeof(std::int8_t) * max_corners_);
    cudaEventCreate(&ev_start_);
    cudaEventCreate(&ev_stop_);

    if (!timing_csv_.empty()) {
      csv_out_.open(timing_csv_);
      csv_out_ << "frame,ms_total,ms_klt,ms_stereo,ms_pnp,ms_detect,ms_ba,"
                  "n_tracked,n_active,n_3d_prev,n_pnp_inliers,n_stereo_match,"
                  "n_new,n_total,n_ba_landmarks,ba_solved,"
                  "n_lmp_attempt,n_lmp_promote,ms_lmp\n";
      csv_out_.flush();
    }

    using namespace message_filters;
    using ImgSub  = message_filters::Subscriber<sensor_msgs::msg::Image>;
    using CamSub  = message_filters::Subscriber<sensor_msgs::msg::CameraInfo>;
    sub_left_  = std::make_shared<ImgSub>(this, "left/image_rect_raw",
                                          rmw_qos_profile_sensor_data);
    sub_right_ = std::make_shared<ImgSub>(this, "right/image_rect_raw",
                                          rmw_qos_profile_sensor_data);
    sub_lcam_  = std::make_shared<CamSub>(this, "left/camera_info",
                                          rmw_qos_profile_sensor_data);
    sub_rcam_  = std::make_shared<CamSub>(this, "right/camera_info",
                                          rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<Synchronizer<SyncPolicy>>(SyncPolicy(20),
                                                       *sub_left_, *sub_right_,
                                                       *sub_lcam_, *sub_rcam_);
    sync_->registerCallback(
        std::bind(&KltVoNode::on_stereo, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4));

    // EuRoC publisher uses a RELIABLE QoS on /euroc/imu — match it.
    auto qos_imu = rclcpp::QoS(rclcpp::KeepLast(200))
                       .reliable().durability_volatile();
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", qos_imu,
        std::bind(&KltVoNode::on_imu, this, std::placeholders::_1));

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_odom_   = create_publisher<nav_msgs::msg::Odometry>("/slamko_vio/odometry", 10);
    pub_kp_     = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/slamko_vio/keypoints", 10);
    pub_tracks_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/slamko_vio/tracks", 10);
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    world_pose_.setIdentity();

    RCLCPP_INFO(get_logger(),
                "slamko_vio_node sprint 2 started — max_corners=%d, patch=%d, "
                "pyr_lvls=%d, stereo_disp=[%d,%d], pnp_thr=%.2f",
                max_corners_, patch_size_, pyramid_lvls_,
                mcfg.min_disparity, mcfg.max_disparity,
                pcfg.reprojection_threshold_px);
  }

  ~KltVoNode() override {
    if (d_left_)    cudaFree(d_left_);
    if (d_right_)   cudaFree(d_right_);
    if (d_prev_xy_) cudaFree(d_prev_xy_);
    if (d_curr_xy_) cudaFree(d_curr_xy_);
    if (d_right_xy_) cudaFree(d_right_xy_);
    if (d_status_)   cudaFree(d_status_);
    if (d_stereo_status_) cudaFree(d_stereo_status_);
    cudaEventDestroy(ev_start_);
    cudaEventDestroy(ev_stop_);
    if (csv_out_.is_open()) csv_out_.close();
    if (!landmark_dump_path_.empty()) dump_landmarks();
  }

  // Write the final BA world map to a CSV (id,x,y,z,obs) for offline viz.
  void dump_landmarks() {
    std::ofstream f(landmark_dump_path_);
    if (!f.is_open()) {
      RCLCPP_WARN(get_logger(), "landmark dump: cannot open %s",
                  landmark_dump_path_.c_str());
      return;
    }
    f << "id,x,y,z,obs\n";
    for (const auto& [lid, p] : landmark_world_) {
      int obs = 0;
      auto it = landmark_obs_count_.find(lid);
      if (it != landmark_obs_count_.end()) obs = it->second;
      f << lid << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
        << obs << '\n';
    }
    RCLCPP_INFO(get_logger(), "landmark dump: %zu landmarks -> %s",
                landmark_world_.size(), landmark_dump_path_.c_str());
  }

 private:
  // LMP patch sizing constants (declared up here so the helpers below can see
  // them; the actual patch storage map is declared further down with the
  // rest of the landmark state).
  static constexpr int kLmpPatchSide = 11;
  static constexpr int kLmpPatchHalf = 5;
  static constexpr int kLmpPatchPx   = 121;  // 11*11
  // Earth gravity magnitude, LOCKED (never estimated — stereo gives metric
  // scale, so VI init only needs the gravity direction). See gravity calib.
  static constexpr double kGravityMag = 9.81;

  // Extract an 11×11 mono8 patch from msg_l around (u, v). Returns true on
  // success (in-bounds), false if too close to border.
  bool lmp_extract_patch(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                         float u, float v,
                         std::array<std::uint8_t, kLmpPatchPx>& out) const {
    const int xi = (int)(u + 0.5f);
    const int yi = (int)(v + 0.5f);
    if (xi < kLmpPatchHalf || xi >= image_width_  - kLmpPatchHalf) return false;
    if (yi < kLmpPatchHalf || yi >= image_height_ - kLmpPatchHalf) return false;
    const std::uint8_t* base = msg->data.data();
    const std::size_t   step = msg->step;
    for (int dy = -kLmpPatchHalf; dy <= kLmpPatchHalf; ++dy) {
      const std::uint8_t* row = base + (yi + dy) * step;
      for (int dx = -kLmpPatchHalf; dx <= kLmpPatchHalf; ++dx) {
        out[(dy + kLmpPatchHalf) * kLmpPatchSide + (dx + kLmpPatchHalf)] =
            row[xi + dx];
      }
    }
    return true;
  }

  // Zero-mean normalized cross-correlation between two 11×11 patches. Returns
  // value in [-1, 1]; 0 if either patch has zero variance.
  static float lmp_ncc(const std::array<std::uint8_t, kLmpPatchPx>& a,
                       const std::array<std::uint8_t, kLmpPatchPx>& b) {
    float sa = 0.f, sb = 0.f;
    for (int i = 0; i < kLmpPatchPx; ++i) { sa += a[i]; sb += b[i]; }
    const float ma = sa / (float)kLmpPatchPx;
    const float mb = sb / (float)kLmpPatchPx;
    float num = 0.f, da = 0.f, db = 0.f;
    for (int i = 0; i < kLmpPatchPx; ++i) {
      const float ax = (float)a[i] - ma;
      const float bx = (float)b[i] - mb;
      num += ax * bx;
      da  += ax * ax;
      db  += bx * bx;
    }
    if (da < 1e-3f || db < 1e-3f) return 0.f;
    return num / std::sqrt(da * db);
  }

  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    slamko_vio::ImuSample s;
    s.t = (double)msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    s.a = Eigen::Vector3d(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z);
    s.w = Eigen::Vector3d(msg->angular_velocity.x,
                          msg->angular_velocity.y,
                          msg->angular_velocity.z);
    std::lock_guard<std::mutex> lk(imu_mutex_);
    imu_buffer_.push_back(s);
    // Trim ancient samples (keep ~30 s worth).
    while (!imu_buffer_.empty() &&
           imu_buffer_.back().t - imu_buffer_.front().t > 30.0) {
      imu_buffer_.pop_front();
    }
  }

  // Pop and return IMU samples with timestamps in (t_lo, t_hi]. Adds the
  // bracketing samples on each side so the integration spans the exact window.
  std::vector<slamko_vio::ImuSample> drain_imu_window(double t_lo, double t_hi) {
    std::vector<slamko_vio::ImuSample> out;
    std::lock_guard<std::mutex> lk(imu_mutex_);
    if (imu_buffer_.empty()) return out;
    // Find the first sample at or after t_lo, and last <= t_hi.
    out.reserve(imu_buffer_.size());
    for (const auto& s : imu_buffer_) {
      if (s.t < t_lo - 0.01)  continue;          // discard far-past
      if (s.t > t_hi + 0.01)  break;             // future
      out.push_back(s);
    }
    return out;
  }

  // Resolve T_BS (left-rect-cam → imu body) from the static TF tree the
  // publisher emits. Returns identity if not yet available.
  Eigen::Matrix4d try_resolve_T_BS(const std::string& cam_frame,
                                   const std::string& imu_frame) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (!tf_buffer_) return T;
    try {
      auto tf = tf_buffer_->lookupTransform(
          imu_frame, cam_frame, tf2::TimePointZero,
          tf2::durationFromSec(0.0));
      const auto& q = tf.transform.rotation;
      const auto& p = tf.transform.translation;
      Eigen::Quaterniond qd(q.w, q.x, q.y, q.z);
      T.block<3, 3>(0, 0) = qd.toRotationMatrix();
      T(0, 3) = p.x;  T(1, 3) = p.y;  T(2, 3) = p.z;
    } catch (...) {
      // not yet available; keep identity
    }
    return T;
  }

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>;

  void update_intrinsics(const sensor_msgs::msg::CameraInfo& cl,
                         const sensor_msgs::msg::CameraInfo& cr) {
    if (have_K_) return;
    // P = [fx 0 cx Tx; 0 fy cy 0; 0 0 1 0]; baseline = -Tx / fx (for right camera).
    K_.fx = (float)cl.p[0];
    K_.fy = (float)cl.p[5];
    K_.cx = (float)cl.p[2];
    K_.cy = (float)cl.p[6];
    // baseline from right camera P[3] = -fx * b
    const float Tx_right = (float)cr.p[3];
    const float fx = (float)cr.p[0];
    K_.baseline_m = (fx > 1.0e-6f) ? -Tx_right / fx : 0.f;
    if (!(K_.baseline_m > 0.f)) {
      RCLCPP_WARN(get_logger(),
                  "Stereo baseline came out non-positive (Tx_right=%.4f, fx=%.2f). "
                  "Using |baseline| as a fallback.", Tx_right, fx);
      K_.baseline_m = std::abs(K_.baseline_m);
    }
    have_K_ = true;
    RCLCPP_INFO(get_logger(), "stereo intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f baseline=%.4fm",
                K_.fx, K_.fy, K_.cx, K_.cy, K_.baseline_m);
  }

  void on_stereo(const sensor_msgs::msg::Image::ConstSharedPtr& msg_l,
                 const sensor_msgs::msg::Image::ConstSharedPtr& msg_r,
                 const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_l,
                 const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_r) {
    if (msg_l->encoding != "mono8" || msg_r->encoding != "mono8") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Expected mono8 images");
      return;
    }
    if ((int)msg_l->width != image_width_ || (int)msg_l->height != image_height_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "image size mismatch");
      return;
    }
    update_intrinsics(*cam_l, *cam_r);
    if (!have_K_) return;

    cudaEventRecord(ev_start_);

    // Upload mono8 stereo pair.
    cudaMemcpy2DAsync(d_left_,  image_width_,
                      msg_l->data.data(), msg_l->step,
                      image_width_, image_height_, cudaMemcpyHostToDevice);
    cudaMemcpy2DAsync(d_right_, image_width_,
                      msg_r->data.data(), msg_r->step,
                      image_width_, image_height_, cudaMemcpyHostToDevice);

    tracker_->set_image(d_left_, image_width_);
    matcher_->set_images(d_left_, image_width_, d_right_, image_width_);

    // ------------------------------------------------------------- KLT track
    const int n_prev = (int)tracks_.size();
    int n_active = 0;
    float ms_klt = 0.f;
    if (n_prev > 0 && tracker_has_prev_) {
      std::vector<float> h_prev(2 * n_prev);
      for (int i = 0; i < n_prev; ++i) {
        h_prev[2*i + 0] = tracks_[i].left_curr_x;  // promotes to "prev" this frame
        h_prev[2*i + 1] = tracks_[i].left_curr_y;
      }
      cudaMemcpyAsync(d_prev_xy_, h_prev.data(), sizeof(float) * 2 * n_prev,
                      cudaMemcpyHostToDevice);

      cudaEvent_t a, b; cudaEventCreate(&a); cudaEventCreate(&b);
      cudaEventRecord(a);
      tracker_->track(d_prev_xy_, d_curr_xy_, d_status_, n_prev);
      cudaEventRecord(b);
      cudaEventSynchronize(b);
      cudaEventElapsedTime(&ms_klt, a, b);
      cudaEventDestroy(a); cudaEventDestroy(b);

      std::vector<float>        h_curr(2 * n_prev);
      std::vector<std::int8_t>  h_stat(n_prev);
      cudaMemcpy(h_curr.data(), d_curr_xy_, sizeof(float) * 2 * n_prev,
                 cudaMemcpyDeviceToHost);
      cudaMemcpy(h_stat.data(), d_status_,  sizeof(std::int8_t) * n_prev,
                 cudaMemcpyDeviceToHost);

      std::vector<StereoTrack> kept;
      kept.reserve(n_prev);
      for (int i = 0; i < n_prev; ++i) {
        if (h_stat[i] != 1) continue;
        StereoTrack t = tracks_[i];
        t.left_prev_x = t.left_curr_x;
        t.left_prev_y = t.left_curr_y;
        t.left_curr_x = h_curr[2*i + 0];
        t.left_curr_y = h_curr[2*i + 1];
        t.age += 1;
        kept.push_back(t);
      }
      tracks_ = std::move(kept);
      n_active = (int)tracks_.size();
    }

    // -------------------- Detect new corners (before stereo match + PnP) ----
    int n_new = 0;
    float ms_detect = 0.f;
    if ((int)tracks_.size() < redetect_thr_) {
      cudaEvent_t a, b; cudaEventCreate(&a); cudaEventCreate(&b);
      cudaEventRecord(a);
      detector_->detect(d_left_, image_width_);
      cudaEventRecord(b);
      cudaEventSynchronize(b);
      cudaEventElapsedTime(&ms_detect, a, b);
      cudaEventDestroy(a); cudaEventDestroy(b);
      const auto kps = detector_->get_keypoints();
      const float r2 = (float)(dedup_radius_ * dedup_radius_);
      for (const auto& k : kps) {
        bool dup = false;
        for (const auto& t : tracks_) {
          const float dx = t.left_curr_x - k.x;
          const float dy = t.left_curr_y - k.y;
          if (dx*dx + dy*dy < r2) { dup = true; break; }
        }
        if (dup) continue;
        StereoTrack nt;
        nt.id = next_id_++;
        nt.left_prev_x = k.x;
        nt.left_prev_y = k.y;
        nt.left_curr_x = k.x;
        nt.left_curr_y = k.y;
        nt.has_right_curr = false;
        nt.has_3d_prev    = false;
        nt.has_3d_curr    = false;
        nt.age = 0;
        tracks_.push_back(nt);
        ++n_new;
        if ((int)tracks_.size() >= max_corners_) break;
      }
    }

    // -------------------- Local Map Projection (LMP) ------------------------
    // ORB-SLAM3-style re-acquisition. For each landmark in the world map that
    // we are NOT currently tracking, project it into the current camera using
    // the previous frame's pose (T_w_c_, frame N-1). If the projection lands
    // inside the image and a recently-detected Shi-Tomasi corner exists within
    // lmp_snap_radius_px_ of the prediction, promote that corner-track to a
    // recovered LMP track: assign the existing landmark_id and seed
    // point_3d_curr from the world map.
    //
    // Why this works: the new-corner tracks just added above have landmark_id=
    // 0 (no map identity). When a recently-known landmark projects onto one of
    // these corners, it's almost certainly the same physical feature reborn
    // after a KLT loss. The promotion gives the next-frame PnP an instant 3D
    // anchor instead of waiting 2 frames for fresh stereo triangulation, and
    // restores the track's age + landmark history (BA can re-use it).
    int n_lmp_attempt = 0, n_lmp_promote = 0;
    float ms_lmp = 0.f;
    // Trigger: when KLT lost enough tracks (n_active is the post-KLT count;
    // tracks_.size() at this point has been topped up by redetect).
    if (lmp_enabled_ && have_world_pose_ && have_K_ &&
        n_active < lmp_trigger_active_ &&
        !landmark_world_.empty()) {
      const auto lmp_t0 = std::chrono::steady_clock::now();
      // 1. Build a flat list of corner-track indices (those with landmark_id=0
      //    and freshly born this frame — i.e., the new tracks the redetect
      //    block just appended) along with their pixel positions.
      const int Ntr = (int)tracks_.size();
      std::vector<int> cand_idx;
      cand_idx.reserve(Ntr);
      for (int i = 0; i < Ntr; ++i) {
        if (tracks_[i].landmark_id == 0 && tracks_[i].age == 0) {
          cand_idx.push_back(i);
        }
      }
      if (!cand_idx.empty()) {
        // 2. Spatial hash over the candidate corner-tracks. Image is image_width_
        //    × image_height_; choose ~24 px cells. Cell coords: (x/24, y/24).
        const int cell_sz = 24;
        const int gx = (image_width_  + cell_sz - 1) / cell_sz;
        const int gy = (image_height_ + cell_sz - 1) / cell_sz;
        std::vector<std::vector<int>> grid(gx * gy);
        for (int j : cand_idx) {
          const int cx = std::min(std::max((int)(tracks_[j].left_curr_x / cell_sz), 0), gx - 1);
          const int cy = std::min(std::max((int)(tracks_[j].left_curr_y / cell_sz), 0), gy - 1);
          grid[cy * gx + cx].push_back(j);
        }
        // 3. Build "currently-tracked landmark_ids" set so we don't re-add.
        std::unordered_set<std::uint32_t> live_lids;
        live_lids.reserve(Ntr);
        for (const auto& t : tracks_) {
          if (t.landmark_id != 0) live_lids.insert(t.landmark_id);
        }
        // 4. Project & match. T_w_c_ is the world→cam transform at frame N-1
        //    (small motion error absorbed by the snap radius).
        const float fx = (float)K_.fx, fy_ = (float)K_.fy;
        const float cx_ = (float)K_.cx, cy_ = (float)K_.cy;
        const float border = (float)lmp_image_border_px_;
        const float r2 = lmp_snap_radius_px_ * lmp_snap_radius_px_;
        // Track index → "already promoted this frame" flag. O(1) lookup
        // replaces the O(N) std::find that killed V1_03 perf.
        std::vector<char> track_used(Ntr, 0);
        // Fast culling: world-space distance from camera origin to landmark.
        // Camera origin in world = -R^T * t = world_pose_.block<3,1>(0,3).
        const Eigen::Vector3d cam_origin_w =
            world_pose_.block<3, 1>(0, 3).cast<double>();
        const double max_dist2 =
            (double)lmp_max_depth_m_ * (double)lmp_max_depth_m_ * 1.5;  // generous
        for (const auto& kv : landmark_world_) {
          const std::uint32_t lid = kv.first;
          if (live_lids.count(lid)) continue;
          if (n_lmp_promote >= lmp_max_added_per_frame_) break;
          // World-space distance gate (cheap pre-filter — avoids projection).
          const Eigen::Vector3d dw = kv.second - cam_origin_w;
          if (dw.squaredNorm() > max_dist2) continue;
          ++n_lmp_attempt;
          const Eigen::Vector4d p_w(kv.second.x(), kv.second.y(), kv.second.z(), 1.0);
          const Eigen::Vector4d p_c = T_w_c_ * p_w;
          if (p_c.z() < (double)lmp_min_depth_m_ ||
              p_c.z() > (double)lmp_max_depth_m_) continue;
          const float inv_z = 1.f / (float)p_c.z();
          const float u = fx * (float)p_c.x() * inv_z + cx_;
          const float v = fy_ * (float)p_c.y() * inv_z + cy_;
          if (u < border || u > (float)image_width_  - border) continue;
          if (v < border || v > (float)image_height_ - border) continue;
          const int cx_i = std::min(std::max((int)(u / cell_sz), 0), gx - 1);
          const int cy_i = std::min(std::max((int)(v / cell_sz), 0), gy - 1);
          int best_track = -1;
          float best_d2 = r2 + 1.f;
          for (int dy = -1; dy <= 1; ++dy) {
            const int yy = cy_i + dy;
            if (yy < 0 || yy >= gy) continue;
            for (int dx = -1; dx <= 1; ++dx) {
              const int xx = cx_i + dx;
              if (xx < 0 || xx >= gx) continue;
              const auto& bucket = grid[yy * gx + xx];
              for (int ti : bucket) {
                if (track_used[ti]) continue;
                const float du = tracks_[ti].left_curr_x - u;
                const float dv = tracks_[ti].left_curr_y - v;
                const float d2 = du*du + dv*dv;
                if (d2 < best_d2) { best_d2 = d2; best_track = ti; }
              }
            }
          }
          if (best_track < 0) continue;
          // LMP v4: NCC patch verification. Reject snap-to-corner matches
          // where the local appearance differs from the stored landmark
          // patch. Without this, false matches in textured regions
          // regressed MH_05 / V1_01 in v3.
          auto pit = landmark_patch_.find(lid);
          if (pit != landmark_patch_.end()) {
            std::array<std::uint8_t, kLmpPatchPx> cur_patch;
            if (!lmp_extract_patch(msg_l,
                                   tracks_[best_track].left_curr_x,
                                   tracks_[best_track].left_curr_y,
                                   cur_patch)) continue;
            const float ncc = lmp_ncc(pit->second, cur_patch);
            if (ncc < lmp_ncc_threshold_) continue;
          }
          tracks_[best_track].landmark_id  = lid;
          tracks_[best_track].from_lmp     = true;
          tracks_[best_track].point_3d_curr = p_c.head<3>().cast<float>();
          tracks_[best_track].has_3d_curr  = true;
          track_used[best_track] = 1;
          ++n_lmp_promote;
        }
      }
      const auto lmp_t1 = std::chrono::steady_clock::now();
      ms_lmp = std::chrono::duration<float, std::milli>(lmp_t1 - lmp_t0).count();
    }
    (void)n_lmp_attempt; (void)ms_lmp;

    // --------------- Stereo match + triangulate (BEFORE PnP, R1+R5) ---------
    // PnP needs the right-cam pixel in the current frame for both-camera
    // reprojection. The triangulated 3D point is also used as PnP's
    // previous-frame anchor: after PnP we commit point_3d_curr → point_3d_prev
    // for next frame. (R2 — persistent-map PnP — regressed badly in
    // experiments because new landmarks anchor to a possibly-drifting pose
    // at birth and BA refinements desync with the next-frame PnP; deferred.)
    int n_stereo_match = 0;
    float ms_stereo = 0.f;
    if (!tracks_.empty()) {
      const int N = (int)tracks_.size();
      std::vector<float> h_left(2 * N);
      for (int i = 0; i < N; ++i) {
        h_left[2*i + 0] = tracks_[i].left_curr_x;
        h_left[2*i + 1] = tracks_[i].left_curr_y;
      }
      cudaMemcpyAsync(d_curr_xy_, h_left.data(), sizeof(float) * 2 * N,
                      cudaMemcpyHostToDevice);

      cudaEvent_t a, b; cudaEventCreate(&a); cudaEventCreate(&b);
      cudaEventRecord(a);
      matcher_->match(d_curr_xy_, d_right_xy_, d_stereo_status_, N);
      cudaEventRecord(b);
      cudaEventSynchronize(b);
      cudaEventElapsedTime(&ms_stereo, a, b);
      cudaEventDestroy(a); cudaEventDestroy(b);

      std::vector<float>       h_right(2 * N);
      std::vector<std::int8_t> h_smatch(N);
      cudaMemcpy(h_right.data(),  d_right_xy_,       sizeof(float) * 2 * N,
                 cudaMemcpyDeviceToHost);
      cudaMemcpy(h_smatch.data(), d_stereo_status_,  sizeof(std::int8_t) * N,
                 cudaMemcpyDeviceToHost);

      // Cache LMP-derived 3D before the stereo loop resets has_3d_curr; this
      // lets us restore the map-derived anchor for any LMP track that stereo
      // fails to match this frame.
      std::vector<Eigen::Vector3f> lmp_3d_backup(N);
      std::vector<char>           lmp_3d_have(N, 0);
      for (int i = 0; i < N; ++i) {
        if (tracks_[i].from_lmp && tracks_[i].has_3d_curr) {
          lmp_3d_backup[i] = tracks_[i].point_3d_curr;
          lmp_3d_have[i] = 1;
        }
      }
      for (int i = 0; i < N; ++i) {
        tracks_[i].has_right_curr = false;
        tracks_[i].has_3d_curr    = false;
        if (h_smatch[i] != 1) continue;
        const float ux_r = h_right[2*i + 0];
        const float uy_r = h_right[2*i + 1];
        Eigen::Vector3f p3;
        if (slamko_vio::triangulate_stereo(h_left[2*i + 0], ux_r,
                                       h_left[2*i + 1], K_,
                                       min_depth_m_, max_depth_m_, p3)) {
          tracks_[i].right_curr_x   = ux_r;
          tracks_[i].right_curr_y   = uy_r;
          tracks_[i].has_right_curr = true;
          tracks_[i].point_3d_curr  = p3;
          tracks_[i].has_3d_curr    = true;
          ++n_stereo_match;
        }
      }
      // LMP fallback: restore map-derived 3D for LMP tracks where stereo
      // failed. Without this, the LMP recovery contributes nothing in the
      // very conditions (blur, low texture) it was designed to fix.
      for (int i = 0; i < N; ++i) {
        if (lmp_3d_have[i] && !tracks_[i].has_right_curr) {
          tracks_[i].point_3d_curr = lmp_3d_backup[i];
          tracks_[i].has_3d_curr   = true;
        }
      }
    }

    // Frame timestamp + forced-loss window (Workstream R test hook). Computed
    // BEFORE PnP so a forced loss actually SUPPRESSES the PnP pose update
    // (simulating real visual loss) rather than running dead-reckoning on top
    // of a still-working PnP.
    const double frame_ts =
        msg_l->header.stamp.sec + msg_l->header.stamp.nanosec * 1e-9;
    if (!have_seq_t0_) { seq_t0_ = frame_ts; have_seq_t0_ = true; }
    const double t_rel = frame_ts - seq_t0_;
    const bool forced_loss =
        (dr_force_loss_start_ >= 0.0 && t_rel >= dr_force_loss_start_ &&
         t_rel < dr_force_loss_end_);

    // ------------------------- PnP RANSAC + both-cam Ceres refine (R1 + R2)
    // R2 (mature-landmark gating): each landmark accumulates a count of how
    // many BA keyframes it has appeared in (`landmark_obs_count_`). For
    // tracks whose landmark is mature (count ≥ mature_obs_thr_), we use the
    // BA-refined world position transformed to the previous-frame camera
    // coords as PnP's 3D anchor — this breaks the fresh-triangulation
    // correlated-noise loop. Young tracks fall back to fresh stereo
    // triangulation in cam coords.
    Eigen::Matrix4f T_pp = Eigen::Matrix4f::Identity();
    int n_pnp_in = 0;
    float ms_pnp = 0.f;
    int n_3d_prev = 0;
    int n_mature  = 0;
    bool pnp_ok = false;   // did PnP produce a pose this frame? (dead-reckoning)
    if (!tracks_.empty()) {
      const Eigen::Matrix4d T_w_c_prev = T_w_c_;        // pose BEFORE this PnP
      std::vector<Eigen::Vector3f> p3d;
      std::vector<Eigen::Vector2f> p2d_l, p2d_r;
      std::vector<int> idx_map;
      const int Ntr = (int)tracks_.size();
      p3d.reserve(Ntr); p2d_l.reserve(Ntr); p2d_r.reserve(Ntr);
      idx_map.reserve(Ntr);
      const float nan_f = std::numeric_limits<float>::quiet_NaN();
      for (int i = 0; i < Ntr; ++i) {
        const auto& t = tracks_[i];
        // We need both a current-frame left observation (always true: tracks
        // exist after KLT) and SOME 3D anchor. Prefer mature world landmark.
        bool   used   = false;
        Eigen::Vector3f p_prev;
        if (t.landmark_id != 0) {
          auto cit = landmark_obs_count_.find(t.landmark_id);
          auto wit = landmark_world_.find(t.landmark_id);
          if (cit != landmark_obs_count_.end() && cit->second >= mature_obs_thr_
              && wit != landmark_world_.end()) {
            // Transform refined world → previous-frame cam coords.
            const Eigen::Vector4d p_w(wit->second.x(), wit->second.y(),
                                       wit->second.z(), 1.0);
            const Eigen::Vector4d p_c = T_w_c_prev * p_w;
            if (p_c.z() > min_depth_m_) {
              p_prev = p_c.head<3>().cast<float>();
              used = true;
              ++n_mature;
            }
          }
        }
        if (!used) {
          if (!t.has_3d_prev) continue;
          p_prev = t.point_3d_prev;
        }
        p3d.push_back(p_prev);
        p2d_l.emplace_back(t.left_curr_x, t.left_curr_y);
        p2d_r.emplace_back(t.has_right_curr ? t.right_curr_x : nan_f,
                           t.has_right_curr ? t.right_curr_y : nan_f);
        idx_map.push_back(i);
      }
      n_3d_prev = (int)p3d.size();

      if (n_3d_prev >= 12 && !forced_loss) {
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<int> inliers;
        if (use_pnp_guess_ && have_world_pose_) T_pp = last_T_pp_;
        const bool ok = pose_estimator_->solve(
            p3d, p2d_l, p2d_r, K_, T_pp, inliers,
            /*use_guess=*/use_pnp_guess_ && have_world_pose_);
        pnp_ok = ok;
        if (ok) {
          n_pnp_in = (int)inliers.size();
          world_pose_ = world_pose_ * T_pp.inverse().cast<float>();
          T_w_c_      = world_pose_.cast<double>().inverse();
          last_T_pp_  = T_pp;
          have_world_pose_ = true;

          // Drop tracks rejected as PnP outliers.
          const int n_active_before = (int)tracks_.size();
          std::vector<bool> keep(n_active_before, true);
          for (int i : idx_map) keep[i] = false;
          for (int i : inliers) keep[idx_map[i]] = true;
          std::vector<StereoTrack> kept;
          kept.reserve(n_active_before);
          for (int i = 0; i < n_active_before; ++i)
            if (keep[i]) kept.push_back(tracks_[i]);
          tracks_ = std::move(kept);
          n_active = (int)tracks_.size();
        }
        const auto t1 = std::chrono::steady_clock::now();
        ms_pnp = std::chrono::duration<float, std::milli>(t1 - t0).count();
      }
    }

    // ---------------------------- Workstream R: tracking-loss dead-reckoning
    // If PnP did not produce a pose this frame (too few 3D points, or RANSAC
    // failed), propagate the pose with preintegrated IMU instead of letting it
    // freeze. Bridges short visual gaps (motion blur, low texture, occlusion);
    // re-anchors automatically once PnP recovers. Long gaps diverge (IMU-only),
    // so dr_max_s_ bounds how long we trust it before warning.
    const bool vision_lost = (!pnp_ok) || forced_loss;

    if (dr_enabled_ && vision_lost && enable_imu_ && imu_initialised_ &&
        T_BS_resolved_ && have_world_pose_ && last_frame_ts_ > 0.0) {
      const double dt = frame_ts - last_frame_ts_;
      if (dt > 1.0e-4 && dt < 0.5) {
        auto samples = drain_imu_window(last_frame_ts_, frame_ts);
        if (samples.size() >= 2) {
          slamko_vio::ImuPreintegration pi(bias_lin_, imu_noise_);
          pi.integrate_span(samples.data(), samples.size());
          Eigen::Matrix3d dR; Eigen::Vector3d dV, dP;
          pi.corrected(bias_lin_, dR, dV, dP);
          // Body pose in world: T_WB = T_WC * T_BS^{-1}  (world_pose_ = T_WC).
          const Eigen::Matrix4d T_WB_i =
              world_pose_.cast<double>() * T_BS_.inverse();
          const Eigen::Matrix3d R_WB_i = T_WB_i.block<3, 3>(0, 0);
          const Eigen::Vector3d p_WB_i = T_WB_i.block<3, 1>(0, 3);
          // Constant-velocity + gyro-rotation dead-reckoning (NOT full
          // strapdown). On this lean VI backend the velocity / accel-bias /
          // gravity-direction are jointly fit by BA (great poses, ATE 0.054)
          // but not individually consistent enough to double-integrate the
          // accelerometer open-loop — full Forster strapdown injects ~2.5 m/s²
          // spurious accel and diverges within ~0.2 s. The BA VELOCITY however
          // is accurate (|v|=0.50 vs GT 0.49 at onset), and the gyro (small
          // bias, no gravity coupling) gives reliable rotation. So we hold
          // velocity constant and integrate only the gyro. Bridges short gaps;
          // longer losses are the relocalizer's job. See docs/13.
          const Eigen::Matrix3d R_WB_j = R_WB_i * dR;            // gyro rotation
          const Eigen::Vector3d p_WB_j = p_WB_i + velocity_w_ * dt;  // const vel
          (void)dV; (void)dP;   // accelerometer intentionally NOT integrated
          Eigen::Matrix4d T_WB_j = Eigen::Matrix4d::Identity();
          T_WB_j.block<3, 3>(0, 0) = R_WB_j;
          T_WB_j.block<3, 1>(0, 3) = p_WB_j;
          const Eigen::Matrix4d T_WC_j = T_WB_j * T_BS_;  // body → camera
          world_pose_ = T_WC_j.cast<float>();
          T_w_c_      = T_WC_j.inverse();
          ++dr_frames_;
          if (!in_dead_reckoning_) {
            in_dead_reckoning_ = true;
            dr_start_ts_ = last_frame_ts_;
            RCLCPP_WARN(get_logger(),
                "tracking loss @ t=%.2fs%s — dead-reckoning on IMU "
                "(v_w=[%.3f %.3f %.3f] |v|=%.3f m/s, g_w=[%.2f %.2f %.2f])",
                t_rel, forced_loss ? " (forced)" : "",
                velocity_w_.x(), velocity_w_.y(), velocity_w_.z(),
                velocity_w_.norm(), ba_cfg_.gravity_w.x(),
                ba_cfg_.gravity_w.y(), ba_cfg_.gravity_w.z());
          } else if (frame_ts - dr_start_ts_ > dr_max_s_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "dead-reckoning %.1fs > dr_max_s — pose increasingly unreliable",
                frame_ts - dr_start_ts_);
          }
        }
      }
    } else if (in_dead_reckoning_ && !vision_lost) {
      RCLCPP_INFO(get_logger(),
          "tracking recovered @ t=%.2fs after %d dead-reckoned frames (%.2fs)",
          t_rel, dr_frames_, frame_ts - dr_start_ts_);
      in_dead_reckoning_ = false;
      dr_frames_ = 0;
    }

    // ---------------------------- Sprint 3: KF insertion + local BA refine
    float ms_ba = 0.f;
    int  n_ba_landmarks = 0;
    bool ba_solved = false;
    ++frames_since_last_kf_;
    bool kf_due = false;

    // Fix A (OV2SLAM-style): median pixel parallax since last KF. Triggers
    // KF only when scene has visually changed enough — handles slow motion
    // (V1_01) without inserting KFs at tiny baselines. Tried rotation-
    // compensated parallax (homography subtract); empirically WORSE on
    // V1_03/MH_03 because the depth-∞ assumption breaks for close points.
    double median_parallax_px = 0.0;
    if (have_last_kf_) {
      std::vector<double> parallaxes;
      parallaxes.reserve(tracks_.size());
      for (const auto& t : tracks_) {
        if (!t.has_at_last_kf) continue;
        const double du = (double)t.left_curr_x - (double)t.left_at_last_kf_x;
        const double dv = (double)t.left_curr_y - (double)t.left_at_last_kf_y;
        parallaxes.push_back(std::hypot(du, dv));
      }
      if (!parallaxes.empty()) {
        const std::size_t mid = parallaxes.size() / 2;
        std::nth_element(parallaxes.begin(), parallaxes.begin() + mid,
                          parallaxes.end());
        median_parallax_px = parallaxes[mid];
      }
    }

    if (!have_last_kf_) {
      kf_due = (n_pnp_in > 0) ? true : (n_stereo_match >= 12);
    } else {
      const Eigen::Matrix4d dT = T_w_c_at_last_kf_ * T_w_c_.inverse();
      const double dt_m = dT.block<3, 1>(0, 3).norm();
      const Eigen::AngleAxisd aa(Eigen::Matrix3d(dT.block<3, 3>(0, 0)));
      const double dr_rad = std::abs(aa.angle());
      const bool inliers_dropped =
          (first_kf_inliers_ > 0) &&
          (n_pnp_in < (int)(kf_inlier_drop_ratio_ * (float)first_kf_inliers_));
      // Parallax-triggered KF is the primary policy. Best mean ATE across
      // the EuRoC suite vs frame-interval or combined-OR triggers. Frame
      // safety net (30 frames) handles static-camera segments.
      const bool parallax_trigger = median_parallax_px >= kf_parallax_px_;
      const bool frame_safety     = frames_since_last_kf_ >= kf_max_frames_;
      kf_due = parallax_trigger || frame_safety ||
               (dt_m > kf_translation_thr_m_) ||
               (dr_rad > kf_rotation_thr_rad_) || inliers_dropped;
    }

    if (kf_due && n_stereo_match >= 12 && have_K_ && !forced_loss) {
      std::vector<std::uint32_t>      lids;
      std::vector<Eigen::Vector2d>    uvs_l, uvs_r;
      std::vector<Eigen::Vector3d>    wps;
      lids.reserve(n_stereo_match);
      uvs_l.reserve(n_stereo_match);
      uvs_r.reserve(n_stereo_match);
      wps.reserve(n_stereo_match);
      const double nan_d = std::numeric_limits<double>::quiet_NaN();
      const Eigen::Matrix4d T_c_w = T_w_c_.inverse();   // cam-to-world
      for (auto& t : tracks_) {
        if (!t.has_3d_curr) continue;
        if (t.landmark_id == 0) {
          t.landmark_id = next_lid_++;
          const Eigen::Vector3d p_cam = t.point_3d_curr.cast<double>();
          const Eigen::Vector4d p_w = T_c_w * p_cam.homogeneous();
          landmark_world_[t.landmark_id] = p_w.head<3>();
          // LMP v4: cache the appearance patch so re-acquisition can verify
          // snap-to-corner matches via NCC. Done only at first landmark
          // observation — the appearance is most reliable from the moment
          // we triangulated the 3D position.
          std::array<std::uint8_t, kLmpPatchPx> patch;
          if (lmp_extract_patch(msg_l, t.left_curr_x, t.left_curr_y, patch)) {
            landmark_patch_[t.landmark_id] = patch;
          }
        }
        lids.push_back(t.landmark_id);
        uvs_l.emplace_back((double)t.left_curr_x, (double)t.left_curr_y);
        uvs_r.emplace_back(t.has_right_curr ? (double)t.right_curr_x : nan_d,
                           t.has_right_curr ? (double)t.right_curr_y : nan_d);
        wps.push_back(landmark_world_[t.landmark_id]);
      }
      const double ts_now =
          msg_l->header.stamp.sec + msg_l->header.stamp.nanosec * 1e-9;
      const auto ba_t0 = std::chrono::steady_clock::now();

      // ---- Sprint 4 IMU integration block ---------------------------------
      // - The first KF is inserted visual-only (no preint).
      // - From the second KF on, preintegrate IMU samples in (last_kf_ts, ts_now]
      //   at the current bias linearisation, then call insert_keyframe_with_imu.
      // - Bootstrap velocity from the visual pose delta between consecutive
      //   KFs. BA then refines velocity + bias.
      bool used_imu_insert = false;
      if (enable_imu_ && have_last_kf_) {
        if (!T_BS_resolved_) {
          // Try to resolve cam→imu (we have left-rect optical → imu body).
          // EuRoC publisher names: euroc_imu and euroc_left_rect_optical.
          Eigen::Matrix4d T = try_resolve_T_BS(msg_l->header.frame_id,
                                               "euroc_imu");
          if (!T.isIdentity(1.0e-9)) {
            T_BS_           = T;
            ba_cfg_.T_BS    = T;
            T_BS_resolved_  = true;
            // Reset the LocalBA so the new config takes effect cleanly.
            local_ba_ = std::make_unique<slamko_vio::LocalBA>(ba_cfg_);
            have_last_kf_ = false;     // restart KF chain to fresh-init VI
            RCLCPP_INFO(get_logger(),
                "resolved T_BS (cam→imu): R [%.3f %.3f %.3f] t [%.3f %.3f %.3f]",
                T_BS_(0,0), T_BS_(1,1), T_BS_(2,2),
                T_BS_(0,3), T_BS_(1,3), T_BS_(2,3));
          }
        }
        if (T_BS_resolved_ && !gravity_calibrated_) {
          // One-shot gravity calibration on the first VI window: accel mean
          // in body ≈ -g_body. Transform to the (visual) world via T_BS and
          // current T_w_c (which is the visual estimate at this KF).
          std::vector<slamko_vio::ImuSample> warmup;
          {
            std::lock_guard<std::mutex> lk(imu_mutex_);
            for (const auto& s : imu_buffer_) {
              if ((int)warmup.size() >= imu_init_warmup_samples_) break;
              warmup.push_back(s);
            }
          }
          if ((int)warmup.size() >= imu_init_warmup_samples_) {
            Eigen::Vector3d a_sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d w_sum = Eigen::Vector3d::Zero();
            for (const auto& s : warmup) { a_sum += s.a; w_sum += s.w; }
            const Eigen::Vector3d a_mean = a_sum / (double)warmup.size();
            const Eigen::Vector3d w_mean = w_sum / (double)warmup.size();
            // R_body_to_world at this KF: T_w_b = T_BS * T_w_c, so
            // R_b_to_w = (T_BS_R * R_wc)^T.
            const Eigen::Matrix3d R_w_b = T_BS_.block<3, 3>(0, 0)
                                        * T_w_c_.block<3, 3>(0, 0);
            const Eigen::Matrix3d R_b_w = R_w_b.transpose();
            // OKVIS-style init: take gravity DIRECTION from the accel mean but
            // LOCK its MAGNITUDE to 9.81. The raw warmup mean has |a|!=9.81 when
            // the body accelerates during init (MH_01 starts in motion →
            // |raw|≈10.97, a 12% error that made the IMU factors fight the
            // stereo-scaled visual estimate). Stereo gives metric scale, so we
            // never estimate g magnitude — only its direction. See OKVIS2-X
            // ImuError.cpp:828 (g_W = imuParams.g * up).
            const Eigen::Vector3d g_raw = R_b_w * (-a_mean);
            ba_cfg_.gravity_w = kGravityMag * g_raw.normalized();
            // Gyro bias: estimated PROPERLY below from the visual rotation
            // chain (handled per-frame after PnP). The warmup-mean approach
            // failed on MH_01 (body in motion at start). Accel bias starts 0.
            bias_lin_.ba = Eigen::Vector3d::Zero();
            (void)w_mean;
            RCLCPP_INFO(get_logger(),
                "calibrated gravity_w = [%.3f %.3f %.3f] |g|=%.2f "
                "(raw |a_mean|=%.2f, locked to %.2f), "
                "bias_g_seed = [%.4f %.4f %.4f] rad/s",
                ba_cfg_.gravity_w.x(), ba_cfg_.gravity_w.y(),
                ba_cfg_.gravity_w.z(), ba_cfg_.gravity_w.norm(),
                a_mean.norm(), kGravityMag,
                bias_lin_.bg.x(), bias_lin_.bg.y(), bias_lin_.bg.z());
            gravity_calibrated_ = true;
            // Reset BA with calibrated gravity so the very first IMU factor
            // sees the right physics. Clears prior visual-only KFs.
            local_ba_ = std::make_unique<slamko_vio::LocalBA>(ba_cfg_);
            have_last_kf_      = false;
            imu_initialised_   = false;
            // Re-insert current frame as the new oldest KF below, after the
            // visual-only path. Skip the IMU branch this cycle.
          }
        }
        // ---- Gyro-bias init from visual rotation chain --------------------
        // For each consecutive KF pair, compare the visual body rotation
        // (R_wb_curr * R_wb_prev^T) over dt to the mean IMU gyro reading
        // over the same interval. bias_g = mean(ω_imu - ω_visual). After
        // N=15 samples lock the linearisation bias and let the IMU factor
        // start contributing.
        if (T_BS_resolved_ && gravity_calibrated_ && !bias_g_initialised_) {
          const Eigen::Matrix3d R_BS_R = T_BS_.block<3, 3>(0, 0);
          const Eigen::Matrix3d R_w_b = R_BS_R * T_w_c_.block<3, 3>(0, 0);
          if (has_R_w_b_prev_) {
            const double dt = ts_now - bias_init_prev_ts_;
            if (dt > 1.0e-3) {
              // Body-frame angular velocity from visual: in Forster's notation
              //   ω_B · dt = Log(R_W_B_i^T · R_W_B_{i+1})
              // We store R_w_b ≡ R_W_to_B = R_W_B^T (world-to-body), so the
              // body-frame angular rate is Log(R_w_b_prev · R_w_b_curr^T) / dt.
              const Eigen::Matrix3d dR = R_w_b_prev_ * R_w_b.transpose();
              const Eigen::AngleAxisd aa((Eigen::Matrix3d(dR)));
              const Eigen::Vector3d omega_visual =
                  aa.axis() * (aa.angle() / dt);
              // Mean IMU gyro over (prev_ts, ts_now].
              auto samples = drain_imu_window(bias_init_prev_ts_, ts_now);
              if ((int)samples.size() >= 2) {
                Eigen::Vector3d w_sum = Eigen::Vector3d::Zero();
                for (const auto& s : samples) w_sum += s.w;
                const Eigen::Vector3d omega_imu =
                    w_sum / (double)samples.size();
                bias_g_estimate_ += (omega_imu - omega_visual);
                ++bias_g_count_;
                if (bias_g_count_ >= bias_init_samples_) {
                  bias_lin_.bg = bias_g_estimate_ / (double)bias_g_count_;
                  bias_g_initialised_ = true;
                  RCLCPP_INFO(get_logger(),
                      "gyro bias init from visual chain: [%.5f %.5f %.5f] rad/s "
                      "(from %d KF pairs)",
                      bias_lin_.bg.x(), bias_lin_.bg.y(), bias_lin_.bg.z(),
                      bias_g_count_);
                }
              }
            }
          }
          R_w_b_prev_       = R_w_b;
          has_R_w_b_prev_   = true;
          bias_init_prev_ts_ = ts_now;
        }

        if (T_BS_resolved_ && gravity_calibrated_ && bias_g_initialised_) {
          // Pull IMU samples spanning the inter-KF interval.
          auto samples = drain_imu_window(last_kf_ts_, ts_now);
          if ((int)samples.size() >= 2) {
            // Sandwich the interval: insert bracketing virtual samples at
            // the exact endpoints by clamping timestamps. Forster integrate
            // uses (sample[i-1].w, sample[i-1].a) over (t[i] - t[i-1]).
            samples.front().t = std::max(samples.front().t, last_kf_ts_);
            samples.back().t  = std::min(samples.back().t,  ts_now);

            slamko_vio::ImuPreintegration pi(bias_lin_, imu_noise_);
            pi.integrate_span(samples.data(), samples.size());

            // Bootstrap velocity from visual delta if not yet initialised.
            if (!imu_initialised_) {
              const Eigen::Vector3d p_prev_w = T_w_c_at_last_kf_.inverse()
                                                  .block<3, 1>(0, 3);
              const Eigen::Vector3d p_curr_w = T_w_c_.inverse()
                                                  .block<3, 1>(0, 3);
              const double dt_v = std::max(1.0e-3, ts_now - last_kf_ts_);
              velocity_w_ = (p_curr_w - p_prev_w) / dt_v;
              imu_initialised_ = true;
            }

            local_ba_->insert_keyframe_with_imu(
                ts_now, T_w_c_, velocity_w_, bias_lin_, pi,
                K_, lids, uvs_l, uvs_r, wps);
            used_imu_insert = true;
          }
        }
      }
      if (!used_imu_insert) {
        local_ba_->insert_keyframe(ts_now, T_w_c_, K_, lids, uvs_l, uvs_r, wps);
      }
      // R2 maturity counter: each KF observation matures the landmark.
      for (std::uint32_t lid : lids) ++landmark_obs_count_[lid];
      if (local_ba_->solve()) {
        ba_solved = true;
        Eigen::Matrix4d T_refined;
        if (local_ba_->latest_pose(T_refined)) {
          T_w_c_ = T_refined;
          world_pose_ = T_w_c_.inverse().cast<float>();
        }
        // Pull back refined landmark world positions for IDs still in window.
        for (std::uint32_t lid : lids) {
          Eigen::Vector3d p_refined;
          if (local_ba_->landmark_world(lid, p_refined)) {
            landmark_world_[lid] = p_refined;
          }
        }
        // Pull back refined velocity + bias for next-interval preintegration.
        if (enable_imu_ && T_BS_resolved_) {
          Eigen::Vector3d v_ref;
          slamko_vio::ImuBias b_ref;
          if (local_ba_->latest_velocity(v_ref)) velocity_w_ = v_ref;
          if (local_ba_->latest_bias(b_ref))     bias_lin_   = b_ref;
        }
      }
      last_kf_ts_ = ts_now;
      n_ba_landmarks = local_ba_->landmark_count();
      first_kf_inliers_     = std::max(first_kf_inliers_, n_pnp_in);
      T_w_c_at_last_kf_     = T_w_c_;
      have_last_kf_         = true;
      frames_since_last_kf_ = 0;
      // Fix A: snapshot the left-cam pixel at this KF; parallax for next-KF
      // decision is computed against this.
      for (auto& t : tracks_) {
        t.left_at_last_kf_x = t.left_curr_x;
        t.left_at_last_kf_y = t.left_curr_y;
        t.has_at_last_kf    = true;
      }
      const auto ba_t1 = std::chrono::steady_clock::now();
      ms_ba = std::chrono::duration<float, std::milli>(ba_t1 - ba_t0).count();
    }

    // Commit this-frame triangulation → previous-frame anchor for next PnP.
    for (auto& t : tracks_) {
      if (t.has_3d_curr) {
        t.point_3d_prev = t.point_3d_curr;
        t.has_3d_prev   = true;
      } else {
        t.has_3d_prev   = false;
      }
    }

    cudaEventRecord(ev_stop_);
    cudaEventSynchronize(ev_stop_);
    float ms_total = 0.f;
    cudaEventElapsedTime(&ms_total, ev_start_, ev_stop_);

    publish_odometry(msg_l->header);
    publish_markers(msg_l->header);

    if (csv_out_.is_open()) {
      csv_out_ << frame_idx_ << "," << ms_total << "," << ms_klt << ","
               << ms_stereo << "," << ms_pnp << "," << ms_detect << ","
               << ms_ba << ","
               << n_prev << "," << n_active << "," << n_3d_prev << ","
               << n_pnp_in << "," << n_stereo_match << ","
               << n_new << "," << tracks_.size() << ","
               << n_ba_landmarks << "," << (ba_solved ? 1 : 0)
               << "," << n_lmp_attempt << "," << n_lmp_promote
               << "," << ms_lmp << "\n";
      if ((frame_idx_ % 20) == 0) csv_out_.flush();
    }

    tracker_->swap_pyramids();
    tracker_has_prev_ = true;
    last_frame_ts_ = frame_ts;   // every frame, for dead-reckoning dt
    ++frame_idx_;

    if ((frame_idx_ % 50) == 0) {
      const auto t = world_pose_.block<3,1>(0,3);
      RCLCPP_INFO(get_logger(),
                  "frame=%u  ms_total=%.2f ms_klt=%.2f ms_stereo=%.2f ms_pnp=%.2f  "
                  "active=%d 3d_prev=%d pnp_in=%d stereo=%d  pose=[%.2f,%.2f,%.2f]",
                  frame_idx_, ms_total, ms_klt, ms_stereo, ms_pnp,
                  n_active, n_3d_prev, n_pnp_in, n_stereo_match,
                  t.x(), t.y(), t.z());
    }
  }

  void publish_odometry(const std_msgs::msg::Header& hdr) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = hdr.stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id  = child_frame_;
    const Eigen::Vector3f t = world_pose_.block<3,1>(0,3);
    const Eigen::Matrix3f R = world_pose_.block<3,3>(0,0);
    const Eigen::Quaternionf q(R);
    msg.pose.pose.position.x = t.x();
    msg.pose.pose.position.y = t.y();
    msg.pose.pose.position.z = t.z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    pub_odom_->publish(msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = msg.header;
      tf.child_frame_id = child_frame_;
      tf.transform.translation.x = t.x();
      tf.transform.translation.y = t.y();
      tf.transform.translation.z = t.z();
      tf.transform.rotation = msg.pose.pose.orientation;
      tf_pub_->sendTransform(tf);
    }
  }

  void publish_markers(const std_msgs::msg::Header& hdr) {
    visualization_msgs::msg::MarkerArray kp_arr;
    visualization_msgs::msg::Marker m;
    m.header = hdr;
    m.ns = "slamko_vio_kps"; m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = 2.0;
    m.color.r = 0.f; m.color.g = 1.f; m.color.b = 0.f; m.color.a = 1.f;
    m.pose.orientation.w = 1.0;
    m.points.reserve(tracks_.size());
    for (const auto& t : tracks_) {
      geometry_msgs::msg::Point p;
      p.x = t.left_curr_x; p.y = t.left_curr_y; p.z = 0.0;
      m.points.push_back(p);
    }
    kp_arr.markers.push_back(m);
    pub_kp_->publish(kp_arr);
  }

  int image_width_, image_height_;
  int max_corners_, redetect_thr_;
  double dedup_radius_;
  int patch_size_, pyramid_lvls_;
  std::string timing_csv_, odom_frame_, child_frame_;
  std::string landmark_dump_path_;
  bool publish_tf_;

  slamko_vio::StereoIntrinsics K_{};
  bool have_K_ = false;
  float min_depth_m_ = 0.3f, max_depth_m_ = 15.f;
  bool  use_pnp_guess_ = true;
  bool             have_world_pose_ = false;
  Eigen::Matrix4f  last_T_pp_       = Eigen::Matrix4f::Identity();

  // Sprint 4 visual-inertial state.
  bool             enable_imu_      = true;
  slamko_vio::ImuNoise imu_noise_;
  int              imu_init_warmup_samples_ = 80;
  slamko_vio::LocalBA::Config ba_cfg_;
  std::mutex                              imu_mutex_;
  std::deque<slamko_vio::ImuSample>           imu_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool             T_BS_resolved_     = false;
  Eigen::Matrix4d  T_BS_              = Eigen::Matrix4d::Identity();
  slamko_vio::ImuBias  bias_lin_;       // current linearisation point
  Eigen::Vector3d  velocity_w_        = Eigen::Vector3d::Zero();
  bool             imu_initialised_   = false;
  bool             gravity_calibrated_ = false;
  double           last_kf_ts_        = 0.0;

  // Workstream R: IMU dead-reckoning during visual tracking loss. When PnP
  // can't run (n_3d_prev<12) or fails, we propagate the body nav state with
  // preintegrated IMU (Forster, world frame) so the published pose keeps
  // moving instead of freezing. Re-anchors automatically when PnP recovers.
  bool             dr_enabled_         = false;  // see param decl: needs gravity-dir fix
  double           dr_max_s_           = 1.0;    // warn/limit long DR
  double           last_frame_ts_      = 0.0;   // every frame (not just KF)
  bool             in_dead_reckoning_  = false;
  double           dr_start_ts_        = 0.0;
  int              dr_frames_          = 0;
  double           seq_t0_             = 0.0;
  bool             have_seq_t0_        = false;
  // Debug: force a visual-loss window [start,end] s (relative to seq start)
  // to test dead-reckoning in isolation. <0 disables.
  double           dr_force_loss_start_ = -1.0;
  double           dr_force_loss_end_   = -1.0;

  // Visual-rotation gyro bias init state.
  bool             bias_g_initialised_ = false;
  Eigen::Vector3d  bias_g_estimate_    = Eigen::Vector3d::Zero();
  int              bias_g_count_       = 0;
  int              bias_init_samples_  = 15;  // KF pairs to average
  Eigen::Matrix3d  R_w_b_prev_         = Eigen::Matrix3d::Identity();
  bool             has_R_w_b_prev_     = false;
  double           bias_init_prev_ts_  = 0.0;

  std::unique_ptr<slamko_vio::ShiTomasiDetector> detector_;
  std::unique_ptr<slamko_vio::KltTracker>        tracker_;
  std::unique_ptr<slamko_vio::StereoMatcher>     matcher_;
  std::unique_ptr<slamko_vio::PoseEstimator>     pose_estimator_;

  std::uint8_t*    d_left_     = nullptr;
  std::uint8_t*    d_right_    = nullptr;
  float*           d_prev_xy_  = nullptr;
  float*           d_curr_xy_  = nullptr;
  float*           d_right_xy_ = nullptr;
  std::int8_t*     d_status_   = nullptr;
  std::int8_t*     d_stereo_status_ = nullptr;
  cudaEvent_t      ev_start_{}, ev_stop_{};

  std::vector<StereoTrack> tracks_;
  std::uint32_t    next_id_ = 1;
  bool             tracker_has_prev_ = false;
  std::uint32_t    frame_idx_ = 0;
  std::ofstream    csv_out_;

  Eigen::Matrix4f world_pose_;

  // Sprint 3 — local BA state.
  std::unique_ptr<slamko_vio::LocalBA> local_ba_;
  Eigen::Matrix4d  T_w_c_  = Eigen::Matrix4d::Identity();  // world-to-cam, BA gauge
  std::unordered_map<std::uint32_t, Eigen::Vector3d> landmark_world_;
  std::unordered_map<std::uint32_t, int> landmark_obs_count_;  // R2 maturity counter
  // LMP v4: 11×11 grayscale patch stored at landmark first observation. Used
  // by NCC verification in the LMP re-acquisition step to reject false
  // snap-to-corner matches (the bug that regressed MH_05 / V1_01 in v3).
  // Constants declared above (kLmpPatchSide/Half/Px).
  std::unordered_map<std::uint32_t, std::array<std::uint8_t, kLmpPatchPx>> landmark_patch_;
  int              mature_obs_thr_ = 2;
  std::uint32_t    next_lid_  = 1;
  // Local map projection knobs / telemetry.
  bool             lmp_enabled_                = true;
  int              lmp_trigger_active_         = 800;
  float            lmp_snap_radius_px_         = 3.f;
  float            lmp_min_depth_m_            = 0.3f;
  float            lmp_max_depth_m_            = 25.f;
  int              lmp_max_added_per_frame_    = 400;
  int              lmp_image_border_px_        = 12;
  float            lmp_ncc_threshold_          = 0.70f;
  int              kf_interval_frames_   = 5;
  int              frames_since_last_kf_ = 0;
  float            kf_translation_thr_m_ = 0.15f;
  float            kf_rotation_thr_rad_  = 0.087f;
  float            kf_inlier_drop_ratio_ = 0.7f;
  float            kf_parallax_px_       = 15.0f;
  int              kf_max_frames_        = 30;
  int              first_kf_inliers_     = 0;
  Eigen::Matrix4d  T_w_c_at_last_kf_     = Eigen::Matrix4d::Identity();
  bool             have_last_kf_         = false;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_left_, sub_right_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> sub_lcam_, sub_rcam_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_kp_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_tracks_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KltVoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
