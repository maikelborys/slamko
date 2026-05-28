// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// vio_node — thin ROS 2 wrapper around the ROS-agnostic VioPipeline. It only:
//   - reads params into a VioConfig,
//   - syncs the stereo pair + camera_info and the IMU,
//   - wraps each image as an ImageView and calls pipeline.processStereo(),
//   - resolves cam→imu T_BS from TF once and hands it to the pipeline,
//   - publishes /slamko_vio/odometry + TF + keypoint markers from the pipeline.
// All algorithm + state lives in VioPipeline (bag/sim/real-portable, testable).

#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "slamko_core/estimation_frame.hpp"
#include "slamko_core/features.hpp"
#include "slamko_core/image_view.hpp"
#include "slamko_core/local_smoother.hpp"
#include "slamko_vio/vio_pipeline.hpp"
#include "slamko_vio/types.hpp"
#include "slamko_fusion/gtsam_local_smoother.hpp"        // node-only (composition root)
#include "slamko_fusion/gtsam_global_smoother.hpp"       // C.live: BA on weld
#include "slamko_loop/never_lost_supervisor.hpp"          // node-only (P2c)
#include "slamko_loop/xfeat_relocalizer.hpp"
#include "slamko_core/submap_io.hpp"                       // P4: cross-session map I/O

using slamko_vio::VioConfig;

class VioNode : public rclcpp::Node {
 public:
  VioNode() : Node("slamko_vio_node") {
    VioConfig cfg;
    auto P = [&](const std::string& n, auto def) { return declare_parameter(n, def); };
    cfg.image_width        = P("image_width", cfg.image_width);
    cfg.image_height       = P("image_height", cfg.image_height);
    cfg.max_corners        = P("max_corners", cfg.max_corners);
    cfg.redetect_threshold = P("redetect_threshold", cfg.redetect_threshold);
    cfg.dedup_radius_px    = P("dedup_radius_px", cfg.dedup_radius_px);
    cfg.patch_size         = P("patch_size", cfg.patch_size);
    cfg.pyramid_levels     = P("pyramid_levels", cfg.pyramid_levels);
    cfg.timing_csv_path    = P("timing_csv_path", cfg.timing_csv_path);
    cfg.odom_frame_id      = P("odom_frame_id", cfg.odom_frame_id);
    cfg.child_frame_id     = P("child_frame_id", cfg.child_frame_id);
    cfg.publish_tf         = P("publish_tf", cfg.publish_tf);
    cfg.landmark_dump_path = P("landmark_dump_path", cfg.landmark_dump_path);
    cfg.pose_dump_path     = P("pose_dump_path", cfg.pose_dump_path);
    cfg.grid_cols          = P("grid_cols", cfg.grid_cols);
    cfg.grid_rows          = P("grid_rows", cfg.grid_rows);
    cfg.k_per_cell         = P("k_per_cell", cfg.k_per_cell);
    cfg.stereo_patch_size  = P("stereo_patch_size", cfg.stereo_patch_size);
    cfg.min_disparity      = P("min_disparity", cfg.min_disparity);
    cfg.max_disparity      = P("max_disparity", cfg.max_disparity);
    cfg.stereo_ncc_thr     = P("stereo_ncc_thr", cfg.stereo_ncc_thr);
    cfg.stereo_border      = P("stereo_border", cfg.stereo_border);
    cfg.pnp_reproj_thr_px  = P("pnp_reproj_thr_px", cfg.pnp_reproj_thr_px);
    cfg.pnp_max_iters      = P("pnp_max_iters", cfg.pnp_max_iters);
    cfg.pnp_min_inliers    = P("pnp_min_inliers", cfg.pnp_min_inliers);
    cfg.pnp_refine_thr_px  = P("pnp_refine_thr_px", cfg.pnp_refine_thr_px);
    cfg.pnp_use_cuda       = P("pnp_use_cuda", cfg.pnp_use_cuda);
    cfg.pnp_cuda_max_points= P("pnp_cuda_max_points", cfg.pnp_cuda_max_points);
    cfg.pnp_lm_max_iters   = P("pnp_lm_max_iters", cfg.pnp_lm_max_iters);
    cfg.pnp_refine_second_pass = P("pnp_refine_second_pass", cfg.pnp_refine_second_pass);
    cfg.pnp_use_guess      = P("pnp_use_guess", cfg.pnp_use_guess);
    cfg.ba_window_size     = P("ba_window_size", cfg.ba_window_size);
    cfg.ba_huber_px        = P("ba_huber_px", cfg.ba_huber_px);
    cfg.ba_max_iters       = P("ba_max_iters", cfg.ba_max_iters);
    cfg.ba_function_tol    = P("ba_function_tol", cfg.ba_function_tol);
    cfg.ba_min_obs_per_landmark = P("ba_min_obs_per_landmark", cfg.ba_min_obs_per_landmark);
    cfg.enable_imu         = P("enable_imu", cfg.enable_imu);
    cfg.ba_use_inv_depth   = P("ba_use_inv_depth", cfg.ba_use_inv_depth);
    cfg.imu_bias_rw_gyro   = P("imu_bias_rw_gyro", cfg.imu_bias_rw_gyro);
    cfg.imu_bias_rw_accel  = P("imu_bias_rw_accel", cfg.imu_bias_rw_accel);
    cfg.imu_gyro_noise_density  = P("imu_gyro_noise_density", cfg.imu_gyro_noise_density);
    cfg.imu_accel_noise_density = P("imu_accel_noise_density", cfg.imu_accel_noise_density);
    cfg.imu_rate_hz        = P("imu_rate_hz", cfg.imu_rate_hz);
    cfg.imu_init_warmup_samples = P("imu_init_warmup_samples", cfg.imu_init_warmup_samples);
    cfg.dr_enabled         = P("dr_enabled", cfg.dr_enabled);
    cfg.dr_max_s           = P("dr_max_s", cfg.dr_max_s);
    cfg.dr_force_loss_start_s = P("dr_force_loss_start_s", cfg.dr_force_loss_start_s);
    cfg.dr_force_loss_end_s   = P("dr_force_loss_end_s", cfg.dr_force_loss_end_s);
    // Extra forced-loss windows as "start:end,start:end,..." (seconds, rel to seq
    // start) — induces several seals for the multi-submap merge validation.
    cfg.dr_force_loss_windows = parseLossWindows(
        P("dr_force_loss_windows", std::string{}));
    cfg.kf_translation_m   = P("kf_translation_m", cfg.kf_translation_m);
    cfg.kf_rotation_rad    = P("kf_rotation_rad", cfg.kf_rotation_rad);
    cfg.kf_inlier_drop     = P("kf_inlier_drop", cfg.kf_inlier_drop);
    cfg.kf_parallax_px     = P("kf_parallax_px", cfg.kf_parallax_px);
    cfg.kf_max_frames      = P("kf_max_frames", cfg.kf_max_frames);
    cfg.mature_obs_thr     = P("mature_obs_thr", cfg.mature_obs_thr);
    cfg.min_depth_m        = P("min_depth_m", cfg.min_depth_m);
    cfg.max_depth_m        = P("max_depth_m", cfg.max_depth_m);
    cfg.lmp_enabled        = P("lmp_enabled", cfg.lmp_enabled);
    cfg.lmp_trigger_active = P("lmp_trigger_active", cfg.lmp_trigger_active);
    cfg.lmp_snap_radius_px = P("lmp_snap_radius_px", cfg.lmp_snap_radius_px);
    cfg.lmp_min_depth_m    = P("lmp_min_depth_m", cfg.lmp_min_depth_m);
    cfg.lmp_max_depth_m    = P("lmp_max_depth_m", cfg.lmp_max_depth_m);
    cfg.lmp_max_added_per_frame = P("lmp_max_added_per_frame", cfg.lmp_max_added_per_frame);
    cfg.lmp_image_border_px= P("lmp_image_border_px", cfg.lmp_image_border_px);
    cfg.lmp_ncc_threshold  = P("lmp_ncc_threshold", cfg.lmp_ncc_threshold);
    cfg.backend            = P("backend", cfg.backend);
    cfg.feature_source     = P("feature_source", cfg.feature_source);
    cfg.xfeat_onnx_path    = P("xfeat_onnx_path", cfg.xfeat_onnx_path);
    cfg.xfeat_engine_path  = P("xfeat_engine_path", cfg.xfeat_engine_path);
    cfg.xfeat_keypoint_threshold = P("xfeat_keypoint_threshold", cfg.xfeat_keypoint_threshold);
    if (cfg.feature_source == "xfeat" && cfg.xfeat_onnx_path.empty()) {
      cfg.xfeat_onnx_path =
          ament_index_cpp::get_package_share_directory("slamko_vio") + "/models/xfeat.onnx";
    }
    // EigenPlaces global VPR descriptor (loop-closure retrieval). Default-enable when the
    // never-lost relocalizer runs (it's what makes loops actually close); the model ships
    // in share/models/. reloc_use_vpr can force it off.
    cfg.enable_vpr      = P("reloc_use_vpr", true);
    cfg.vpr_onnx_path   = P("vpr_onnx_path", cfg.vpr_onnx_path);
    cfg.vpr_engine_path = P("vpr_engine_path", cfg.vpr_engine_path);
    if (cfg.enable_vpr && cfg.vpr_onnx_path.empty()) {
      cfg.vpr_onnx_path =
          ament_index_cpp::get_package_share_directory("slamko_vio") + "/models/eigenplaces.onnx";
    }

    image_width_  = cfg.image_width;
    image_height_ = cfg.image_height;
    odom_frame_   = cfg.odom_frame_id;
    child_frame_  = cfg.child_frame_id;
    publish_tf_   = cfg.publish_tf;

    // Composition root for the Tier-2 backend. "ceres" (default) lets the
    // pipeline build its own CeresLocalSmoother (klt_vo LocalBA). "gtsam"
    // injects slamko_fusion's GtsamLocalSmoother — the node is the only place
    // that knows slamko_fusion, so the pipeline core stays decoupled (Hard
    // Rule #2). The pipeline drives whichever backend through the
    // slamko::LocalSmoother contract (setExtrinsics/setImuParams/setStereoCalib/
    // insertKeyframe).
    std::unique_ptr<slamko::LocalSmoother> backend;
    if (cfg.backend == "gtsam") {
      slamko_fusion::GtsamSmootherConfig gcfg;
      gcfg.use_imu = cfg.enable_imu;
      backend = std::make_unique<slamko_fusion::GtsamLocalSmoother>(gcfg);
      RCLCPP_INFO(get_logger(),
                  "Tier-2 backend: GTSAM fixed-lag smoother (use_imu=%d)",
                  (int)cfg.enable_imu);
    } else {
      if (cfg.backend != "ceres")
        RCLCPP_WARN(get_logger(), "unknown backend '%s'; using ceres",
                    cfg.backend.c_str());
      RCLCPP_INFO(get_logger(), "Tier-2 backend: ceres LocalBA");
    }
    pipeline_ = std::make_unique<slamko_vio::VioPipeline>(cfg, std::move(backend));
    use_imu_gate_ = cfg.enable_imu;  // gate frame processing on IMU coverage (determinism)

    if (!cfg.pose_dump_path.empty()) {
      pose_dump_.open(cfg.pose_dump_path);
      RCLCPP_INFO(get_logger(), "pose dump (TUM) -> %s", cfg.pose_dump_path.c_str());
    }
    // Never-lost map reconstruction: remember the dump paths; the per-frame epoch
    // file + per-submap sidecar are opened/written once neverlost_enabled_ is known.
    nl_landmark_dump_path_ = cfg.landmark_dump_path;
    nl_pose_dump_path_     = cfg.pose_dump_path;

    // P2c: the Tier-3 never-lost supervisor + XFeat relocalizer, driven in-process
    // from the VIO outputs. Built lazily once K + T_BS resolve (need intrinsics +
    // extrinsic). Logs seal/branch/weld; the weld re-anchors map→odom on revisit.
    neverlost_enabled_ = declare_parameter("enable_neverlost", false);
    // P2.5 (live): route the weld through the SE3 pose-graph backend (multi-submap
    // merge); weld-once bounds the duplicate-edge growth per episode.
    nl_use_pose_graph_ = declare_parameter("neverlost_use_pose_graph", false);
    nl_weld_once_      = declare_parameter("neverlost_weld_once", true);
    nl_continuous_reloc_ = declare_parameter("neverlost_continuous_reloc", false);
    nl_auto_seal_dist_m_ = declare_parameter("neverlost_auto_seal_distance_m", 0.0);
    nl_weld_ba_            = declare_parameter("neverlost_weld_ba", false);
    nl_weld_ba_max_iters_  = declare_parameter("neverlost_weld_ba_max_iters", 20);
    nl_weld_ba_pixel_sigma_= declare_parameter("neverlost_weld_ba_pixel_sigma", 1.0);
    // Relocalizer recall knobs (sweepable; defaults match XFeatRelocConfig).
    reloc_match_ratio_      = declare_parameter("reloc_match_ratio", 0.9);
    reloc_use_bow_          = declare_parameter("reloc_use_bow", true);
    reloc_bow_top_k_        = declare_parameter("reloc_bow_top_k", 25);
    reloc_mutual_check_     = declare_parameter("reloc_mutual_check", false);
    reloc_min_inlier_ratio_ = declare_parameter("reloc_min_inlier_ratio", 0.0);
    reloc_min_inliers_      = declare_parameter("reloc_min_inliers", 15);
    // LighterGlue verification: the viewpoint-robust matcher that replaces brute-force
    // NN in the verify stage (what closes a hard revisit XFeat-NN can't). Needs
    // slamko_loop built with -DSLAMKO_LOOP_WITH_TORCH; if the model can't load the
    // relocalizer silently falls back to brute force. Model ships in share/models/.
    reloc_use_lightglue_     = declare_parameter("reloc_use_lightglue", false);
    reloc_lightglue_model_   = declare_parameter("reloc_lightglue_model", std::string());
    reloc_lg_max_views_      = declare_parameter("reloc_lg_max_views", 4);
    // VPR candidate breadth — higher = more submaps verified per query (more recall,
    // more LightGlue compute). With real-KF LightGlue now strong, can push this past
    // the default 10 to surface submaps that VPR ranks below the top-10 on hard revisits.
    reloc_vpr_top_n_         = declare_parameter("reloc_vpr_top_n", 10);
    // P4: cross-session map persistence. prior_map_dir → load a prior Atlas at startup
    // (seed archive + reloc DB) so the live session localizes into it; map_save_dir →
    // dump the sealed Atlas at shutdown for the next session.
    nl_prior_map_dir_ = declare_parameter("prior_map_dir", std::string{});
    nl_map_save_dir_  = declare_parameter("map_save_dir", std::string{});
    if (neverlost_enabled_ && !nl_pose_dump_path_.empty())
      pose_epoch_.open(nl_pose_dump_path_ + ".epoch");  // per-frame "ts submap_id"

    using ImgSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
    using CamSub = message_filters::Subscriber<sensor_msgs::msg::CameraInfo>;
    // DETERMINISM (offline replay): RELIABLE + deep queue, NOT sensor_data (best-effort,
    // depth 5). When the single-threaded executor stalls on heavy reloc/LighterGlue GPU
    // work, a best-effort sub silently DROPS stereo frames (timing-dependent → ~80 cm
    // run-to-run trajectory divergence + ~13 lost frames). Reliable makes the player
    // (reliable pub) back-pressure/throttle instead → lossless, reproducible replay.
    const auto qos_img = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().get_rmw_qos_profile();
    sub_left_  = std::make_shared<ImgSub>(this, "left/image_rect_raw",  qos_img);
    sub_right_ = std::make_shared<ImgSub>(this, "right/image_rect_raw", qos_img);
    sub_lcam_  = std::make_shared<CamSub>(this, "left/camera_info",  qos_img);
    sub_rcam_  = std::make_shared<CamSub>(this, "right/camera_info", qos_img);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(100), *sub_left_, *sub_right_, *sub_lcam_, *sub_rcam_);
    sync_->registerCallback(std::bind(&VioNode::on_stereo, this,
        std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4));

    auto qos_imu = rclcpp::QoS(rclcpp::KeepLast(200)).reliable().durability_volatile();
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", qos_imu, std::bind(&VioNode::on_imu, this, std::placeholders::_1));

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/slamko_vio/odometry", 10);
    pub_kp_   = create_publisher<visualization_msgs::msg::MarkerArray>("/slamko_vio/keypoints", 10);
    tf_pub_   = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

  ~VioNode() override { writeSubmapSidecar(); }

 private:
  // At shutdown, write "<landmark_dump>.submaps": for each never-lost submap, the
  // landmark-id range it owns + its final (welded) anchor. The offline viz uses this
  // to place each submap's landmarks in the corrected MAP frame (map = anchor·odom),
  // making the merge visible instead of the raw drifted odom-frame cloud.
  void writeSubmapSidecar() {
    if (!neverlost_enabled_ || !supervisor_) return;

    // Viz sidecar (only if a landmark dump was requested).
    if (!nl_landmark_dump_path_.empty()) {
      std::ofstream f(nl_landmark_dump_path_ + ".submaps");
      if (f.is_open()) {
        f << "submap_id,id_lo,id_hi,a00,a01,a02,a03,a10,a11,a12,a13,a20,a21,a22,a23\n";
        auto rows = seal_idhi_;  // sealed submaps, in seal order
        rows.emplace_back(supervisor_->archive().activeId(), pipeline_->maxLandmarkId());
        std::uint64_t lo = 1;
        for (const auto& [sid, hi] : rows) {
          // okvis-arch-refactor P1: live trajectory is now PURE VIO. The supervisor's
          // anchor algebra was destroying ~7× of precision and creating boundary jumps
          // (mag1 max delta 478 cm, vertical 303 cm — pure VIO is 21 cm / 8 cm). The
          // .submaps sidecar is still written for downstream tooling (relocalizer DB
          // book-keeping, cross-session ids), but every anchor is forced to IDENTITY so
          // plot_slamko / check_neverlost don't re-corrupt the smooth VIO trajectory.
          // The supervisor still ATTEMPTS welds (for cross-session relocalization), but
          // its outputs never reach the trajectory dump or the TF map→odom.
          const Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
          f << sid << ',' << lo << ',' << hi;
          for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c) f << ',' << std::setprecision(9) << A(r, c);
          f << '\n';
          lo = hi + 1;
        }
        RCLCPP_INFO(get_logger(), "[neverlost] wrote %zu-submap map sidecar -> %s.submaps",
                    rows.size(), nl_landmark_dump_path_.c_str());
      }
    }

    // P4: persist the Atlas (sealed + the live active) for the next session to load.
    if (!nl_map_save_dir_.empty()) {
      std::vector<slamko::SubMap> maps = supervisor_->archive().sealed();
      maps.push_back(supervisor_->archive().active());
      if (slamko::saveSubMaps(maps, nl_map_save_dir_))
        RCLCPP_INFO(get_logger(), "[neverlost] saved %zu-submap Atlas -> %s",
                    maps.size(), nl_map_save_dir_.c_str());
      else
        RCLCPP_WARN(get_logger(), "[neverlost] FAILED to save Atlas to %s",
                    nl_map_save_dir_.c_str());
      // Calib + T_BS sidecar — the offline BA tool needs them (the .smap schema
      // is per-submap and doesn't carry rig calibration). One line, ASCII, stable:
      //   fx fy cx cy baseline tx ty tz qx qy qz qw
      // T_BS is cam->body (matches StereoCalib + the GTSAM body_T_cam convention).
      if (have_K_ && extrinsics_set_) {
        std::ofstream c(nl_map_save_dir_ + "/calib.txt");
        const Eigen::Quaterniond q(Eigen::Matrix3d(node_T_BS_.block<3,3>(0,0)));
        c << std::fixed << std::setprecision(9)
          << K_.fx << ' ' << K_.fy << ' ' << K_.cx << ' ' << K_.cy << ' '
          << K_.baseline_m << ' '
          << node_T_BS_(0,3) << ' ' << node_T_BS_(1,3) << ' ' << node_T_BS_(2,3) << ' '
          << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
      }
    }
  }

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>;

  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& m) {
    slamko_vio::ImuSample s;
    s.t = (double)m->header.stamp.sec + m->header.stamp.nanosec * 1e-9;
    s.a = Eigen::Vector3d(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
    s.w = Eigen::Vector3d(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
    pipeline_->addImu(s);
    // Advance the IMU frontier and release any buffered frame the IMU now covers.
    // DETERMINISM: rclcpp's executor does NOT guarantee a fixed imu-vs-stereo
    // callback order, so processing a frame on arrival would drain an INCOMPLETE
    // IMU window (late samples are then dropped by drain_imu_window) → divergent
    // preintegration → ~40-80% run-to-run ATE variance. Gating frame processing on
    // "IMU has advanced past the frame timestamp" makes the window complete and the
    // replay reproducible. See slamko_loop/docs memory slamko-vio-replay-nondeterministic.
    if (s.t > latest_imu_ts_) latest_imu_ts_ = s.t;
    drain_pending_frames();
  }

  bool parse_intrinsics(const sensor_msgs::msg::CameraInfo& cl,
                        const sensor_msgs::msg::CameraInfo& cr) {
    K_.fx = (float)cl.p[0]; K_.fy = (float)cl.p[5];
    K_.cx = (float)cl.p[2]; K_.cy = (float)cl.p[6];
    const float Tx_right = (float)cr.p[3];
    const float fx = (float)cr.p[0];
    K_.baseline_m = (fx > 1.0e-6f) ? -Tx_right / fx : 0.f;
    if (!(K_.baseline_m > 0.f)) K_.baseline_m = std::abs(K_.baseline_m);
    return K_.baseline_m > 0.f;
  }

  void try_resolve_extrinsics(const std::string& cam_frame) {
    if (extrinsics_set_ || !tf_buffer_) return;
    try {
      auto tf = tf_buffer_->lookupTransform("euroc_imu", cam_frame,
                                            tf2::TimePointZero, tf2::durationFromSec(0.0));
      const auto& q = tf.transform.rotation;
      const auto& p = tf.transform.translation;
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.block<3,3>(0,0) = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
      T(0,3) = p.x; T(1,3) = p.y; T(2,3) = p.z;
      if (!T.isIdentity(1.0e-9)) {
        pipeline_->setExtrinsics(T);
        node_T_BS_ = T;  // kept for the never-lost supervisor wiring (P2c)
        extrinsics_set_ = true;
        RCLCPP_INFO(get_logger(), "resolved T_BS (cam->imu) from TF");
      }
    } catch (...) { /* not yet available */ }
  }

  void on_stereo(const sensor_msgs::msg::Image::ConstSharedPtr& msg_l,
                 const sensor_msgs::msg::Image::ConstSharedPtr& msg_r,
                 const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_l,
                 const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_r) {
    if (msg_l->encoding != "mono8" || msg_r->encoding != "mono8") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Expected mono8 images");
      return;
    }
    if ((int)msg_l->width != image_width_ || (int)msg_l->height != image_height_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "image size mismatch");
      return;
    }
    if (!have_K_) { have_K_ = parse_intrinsics(*cam_l, *cam_r); if (!have_K_) return; }
    try_resolve_extrinsics(msg_l->header.frame_id);

    const double ts = (double)msg_l->header.stamp.sec + msg_l->header.stamp.nanosec * 1e-9;
    // Buffer the frame (hold the msg shared_ptrs so the pixel data stays alive) and
    // let it be processed by drain_pending_frames() once the IMU stream covers `ts`.
    // No-IMU mode (use_imu_gate_=false) processes immediately — no window to complete.
    pending_frames_.push_back(PendingFrame{msg_l, msg_r, ts});
    drain_pending_frames();
  }

  // Process buffered stereo frames whose timestamp the IMU stream has passed — the
  // ordering-independent gate that makes IMU preintegration deterministic. The very
  // last ≤1 frame after the final IMU sample is dropped (negligible for ATE).
  void drain_pending_frames() {
    while (!pending_frames_.empty() &&
           (!use_imu_gate_ || pending_frames_.front().ts <= latest_imu_ts_)) {
      const PendingFrame f = pending_frames_.front();
      pending_frames_.pop_front();
      const slamko::ImageView left(f.l->data.data(), (int)f.l->width, (int)f.l->height, (int)f.l->step);
      const slamko::ImageView right(f.r->data.data(), (int)f.r->width, (int)f.r->height, (int)f.r->step);
      pipeline_->processStereo(left, right, f.ts, K_);
      publish(f.l->header);
      if (neverlost_enabled_) driveSupervisor(f.ts);
    }
  }

  // Parse "s:e,s:e,..." (seconds, rel to seq start) into [start,end) windows; skips
  // malformed tokens. Empty string → no extra windows.
  static std::vector<std::pair<double, double>> parseLossWindows(const std::string& spec) {
    std::vector<std::pair<double, double>> out;
    std::stringstream ss(spec);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      const auto c = tok.find(':');
      if (c == std::string::npos) continue;
      try {
        const double s = std::stod(tok.substr(0, c));
        const double e = std::stod(tok.substr(c + 1));
        if (e > s) out.emplace_back(s, e);
      } catch (...) { /* skip malformed token */ }
    }
    return out;
  }

  // P2c: feed the live VIO outputs to the never-lost supervisor each frame and log
  // its recovery actions. Built lazily once K + T_BS are known.
  void driveSupervisor(double ts) {
    if (!have_K_ || !extrinsics_set_) return;
    if (!supervisor_) {
      slamko::XFeatRelocConfig rc;
      rc.fx = K_.fx; rc.fy = K_.fy; rc.cx = K_.cx; rc.cy = K_.cy;
      rc.body_T_cam = slamko::SE3(node_T_BS_);   // T_BS (cam→body)
      // Relocalization-recall tuning knobs (exposed as params for sweeping).
      rc.match_ratio      = (float)reloc_match_ratio_;
      rc.use_bow          = reloc_use_bow_;
      rc.bow_top_k        = reloc_bow_top_k_;
      rc.mutual_check     = reloc_mutual_check_;
      rc.min_inlier_ratio = reloc_min_inlier_ratio_;
      rc.min_inliers      = reloc_min_inliers_;
      // LighterGlue verifier (loads lighterglue.pt from share/models/ by default).
      rc.use_lightglue    = reloc_use_lightglue_;
      rc.lg_max_views     = reloc_lg_max_views_;
      rc.vpr_top_n        = reloc_vpr_top_n_;
      if (reloc_use_lightglue_) {
        rc.lightglue_model_path = reloc_lightglue_model_.empty()
            ? ament_index_cpp::get_package_share_directory("slamko_vio") +
                  "/models/lighterglue.pt"
            : reloc_lightglue_model_;
        RCLCPP_INFO(get_logger(), "[neverlost] LighterGlue verify ON (model=%s, views=%d)",
                    rc.lightglue_model_path.c_str(), reloc_lg_max_views_);
      }
      reloc_ = std::make_unique<slamko::XFeatRelocalizer>(rc);
      slamko::SupervisorConfig sc;
      sc.use_pose_graph       = nl_use_pose_graph_;
      sc.weld_once_per_target = nl_weld_once_;
      sc.continuous_reloc     = nl_continuous_reloc_;
      sc.auto_seal_distance_m = nl_auto_seal_dist_m_;
      // C.live V0: BA on weld over the active submap (intra-submap, refines KFs +
      // landmarks using visual + IMU factors when SMP5 windows are present). The
      // composition root injects the concrete GTSAM-backed smoother; slamko_loop
      // holds only the abstract slamko::GlobalSmoother* (Hard Rule #2 clean). The
      // supervisor silently skips BA when this is null — back-compat preserved.
      if (nl_weld_ba_) {
        global_smoother_ = std::make_unique<slamko_fusion::GtsamGlobalSmoother>();
        sc.global_smoother = global_smoother_.get();
        // slamko_vio::StereoIntrinsics → slamko::StereoCalib (float→double, baseline_m
        // → baseline). The two types coexist for legacy reasons (the audit flagged
        // it); for now bridge here.
        sc.ba_calib.fx       = K_.fx;
        sc.ba_calib.fy       = K_.fy;
        sc.ba_calib.cx       = K_.cx;
        sc.ba_calib.cy       = K_.cy;
        sc.ba_calib.baseline = K_.baseline_m;
        sc.ba_T_BS  = slamko::SE3(node_T_BS_);
        sc.ba_pixel_sigma = nl_weld_ba_pixel_sigma_;
        sc.ba_max_iters   = nl_weld_ba_max_iters_;
        RCLCPP_INFO(get_logger(),
                    "[neverlost] BA-on-weld ON (max_iters=%d pixel_sigma=%.2f)",
                    nl_weld_ba_max_iters_, nl_weld_ba_pixel_sigma_);
      }
      supervisor_ = std::make_unique<slamko::NeverLostSupervisor>(sc, reloc_.get());
      RCLCPP_INFO(get_logger(),
                  "[neverlost] supervisor + XFeat relocalizer up (pose_graph=%d weld_once=%d)",
                  (int)nl_use_pose_graph_, (int)nl_weld_once_);
      // Cross-session: load a prior Atlas → seed the archive (frozen sealed submaps)
      // and the relocalizer DB, so the live session welds into the prior map on
      // revisit. Live submap ids continue PAST the priors (set by seedPriorMap).
      if (!nl_prior_map_dir_.empty()) {
        std::vector<slamko::SubMap> priors;
        if (slamko::loadSubMaps(priors, nl_prior_map_dir_) && !priors.empty()) {
          for (const auto& p : priors) reloc_->addSubMap(p);
          supervisor_->seedPriorMap(priors);
          nl_first_live_id_ = supervisor_->archive().activeId();  // priors are < this
          RCLCPP_WARN(get_logger(),
                      "[neverlost] loaded %zu prior submaps from %s (live ids start at %lu)",
                      priors.size(), nl_prior_map_dir_.c_str(),
                      (unsigned long)nl_first_live_id_);
        } else {
          RCLCPP_WARN(get_logger(), "[neverlost] prior_map_dir set but no map loaded from %s",
                      nl_prior_map_dir_.c_str());
        }
      }
    }

    // odom body pose: T_WB = T_WC · T_BS⁻¹ (worldPose is camera-in-world).
    slamko::EstimationFrame ef;
    ef.timestamp = ts;
    ef.T_WB = slamko::SE3(Eigen::Matrix4d(
        pipeline_->worldPose().cast<double>() * node_T_BS_.inverse()));

    // Refresh the active submap content occasionally (buildSubMap is O(landmarks)).
    if (++nl_frame_ % 30 == 0) supervisor_->submitActiveSubMap(pipeline_->buildSubMap());

    // Query features from the current tracks (those carrying an XFeat descriptor).
    slamko::Features q;
    const auto& tr = pipeline_->tracks();
    int nd = 0;
    for (const auto& t : tr) if (t.has_desc) ++nd;
    if (nd > 0) {
      q.keypoints.resize(nd, 3);
      q.descriptors.resize(nd, 64);
      int r = 0;
      for (const auto& t : tr) {
        if (!t.has_desc) continue;
        q.keypoints(r, 0) = t.left_curr_x; q.keypoints(r, 1) = t.left_curr_y; q.keypoints(r, 2) = 1.f;
        for (int d = 0; d < 64; ++d) q.descriptors(r, d) = t.desc[d];
        ++r;
      }
      // VPR global descriptor for coarse candidate retrieval (XFeat local descriptors
      // can't recognize a revisited place — this is what closes loops).
      q.global_descriptor = pipeline_->currentGlobalDescriptor();
      supervisor_->submitQueryFeatures(q);
    }

    const slamko::HealthSignal h = pipeline_->health();
    const slamko::RecoveryAction a = supervisor_->step(h, ef, ts);

    if (a.sealed) {
      // Register the just-sealed submap so the relocalizer can match against it.
      const auto& sealed = supervisor_->archive().sealed();
      if (!sealed.empty()) reloc_->addSubMap(sealed.back());
      // Landmark-id seam: everything created so far belongs to the just-sealed
      // submap (ids are monotonic) — lets the offline map reconstruction place
      // each landmark in the corrected frame via its submap's welded anchor.
      seal_idhi_.emplace_back(a.sealed_id, pipeline_->maxLandmarkId());
      RCLCPP_WARN(get_logger(),
                  "[neverlost] SEAL submap %lu (%zu landmarks) + BRANCH %lu (odom_stale_gap=%.2fs)",
                  (unsigned long)a.sealed_id,
                  sealed.empty() ? 0 : sealed.back().landmarks.size(),
                  (unsigned long)a.branched_id, h.odom_stale_gap_s);
    }
    if (a.branched) {
      // Start a fresh VIO submap epoch so the new branch's buildSubMap() returns only
      // its OWN landmarks — sealed submaps stay disjoint (no cumulative duplication).
      pipeline_->beginSubmap();
    }
    if (a.welded) {
      const Eigen::Vector3d t = supervisor_->mapToOdom().translation();
      const bool cross_session = a.welded_to_id < nl_first_live_id_;
      RCLCPP_WARN(get_logger(),
                  "[neverlost] WELD to submap %lu%s (inliers-gated); map→odom t=[%.3f %.3f %.3f]",
                  (unsigned long)a.welded_to_id, cross_session ? " [CROSS-SESSION/prior map]" : "",
                  t.x(), t.y(), t.z());
    }
    if (supervisor_->state() != nl_last_state_) {
      RCLCPP_INFO(get_logger(), "[neverlost] state %d → %d",
                  (int)nl_last_state_, (int)supervisor_->state());
      nl_last_state_ = supervisor_->state();
    }
  }

  void publish(const std_msgs::msg::Header& hdr) {
    const Eigen::Matrix4f T = pipeline_->worldPose();
    const Eigen::Vector3f t = T.block<3,1>(0,3);
    const Eigen::Quaternionf q(Eigen::Matrix3f(T.block<3,3>(0,0)));

    // Offline TUM trajectory dump (in-process, bypasses rosbag2).
    if (pose_dump_.is_open()) {
      const double ts = (double)hdr.stamp.sec + hdr.stamp.nanosec * 1e-9;
      pose_dump_ << std::fixed << std::setprecision(9) << ts << ' '
                 << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
                 << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
      pose_dump_.flush();  // complete lines stay durable even if the run is killed
      // Lockstep epoch line (same frames as the TUM dump): which submap is active
      // now, so this pose can later be moved into the corrected map frame.
      if (pose_epoch_.is_open()) {
        const std::uint64_t sid = supervisor_ ? supervisor_->archive().activeId() : 0;
        pose_epoch_ << std::fixed << std::setprecision(9) << ts << ' ' << sid << '\n';
        pose_epoch_.flush();
      }
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = hdr.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = child_frame_;
    odom.pose.pose.position.x = t.x(); odom.pose.pose.position.y = t.y(); odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.x = q.x(); odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z(); odom.pose.pose.orientation.w = q.w();
    pub_odom_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = child_frame_;
      tf.transform.translation.x = t.x(); tf.transform.translation.y = t.y(); tf.transform.translation.z = t.z();
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_pub_->sendTransform(tf);
    }

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header = hdr; m.ns = "slamko_vio_kps"; m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = 2.0;
    m.color.g = 1.f; m.color.a = 1.f; m.pose.orientation.w = 1.0;
    for (const auto& tr : pipeline_->tracks()) {
      geometry_msgs::msg::Point p; p.x = tr.left_curr_x; p.y = tr.left_curr_y; p.z = 0.0;
      m.points.push_back(p);
    }
    arr.markers.push_back(m);
    pub_kp_->publish(arr);
  }

  int image_width_ = 752, image_height_ = 480;
  std::string odom_frame_, child_frame_;
  bool publish_tf_ = true;
  slamko_vio::StereoIntrinsics K_{};
  bool have_K_ = false;
  bool extrinsics_set_ = false;

  std::unique_ptr<slamko_vio::VioPipeline> pipeline_;
  std::ofstream pose_dump_;  // optional TUM trajectory export

  // P2c never-lost supervisor (node = composition root).
  bool neverlost_enabled_ = false;
  bool nl_use_pose_graph_ = false;
  bool nl_weld_once_      = true;
  bool nl_continuous_reloc_ = false;
  double nl_auto_seal_dist_m_ = 0.0;
  // C.live V0: BA-on-weld
  bool   nl_weld_ba_ = false;
  int    nl_weld_ba_max_iters_   = 20;
  double nl_weld_ba_pixel_sigma_ = 1.0;
  std::unique_ptr<slamko::GlobalSmoother> global_smoother_;
  double reloc_match_ratio_ = 0.9, reloc_min_inlier_ratio_ = 0.0;
  bool reloc_use_bow_ = true, reloc_mutual_check_ = false;
  int reloc_bow_top_k_ = 25, reloc_min_inliers_ = 15;
  bool reloc_use_lightglue_ = false;
  std::string reloc_lightglue_model_;
  int reloc_lg_max_views_ = 4;
  int reloc_vpr_top_n_     = 10;
  std::string nl_landmark_dump_path_, nl_pose_dump_path_;
  std::string nl_prior_map_dir_, nl_map_save_dir_;  // P4 cross-session map I/O
  std::uint64_t nl_first_live_id_ = 0;              // submap ids < this are prior-session
  std::ofstream pose_epoch_;  // per-frame "ts active_submap_id" (corrected-map viz)
  std::vector<std::pair<std::uint64_t, std::uint64_t>> seal_idhi_;  // (submap_id, max_lm_id@seal)
  Eigen::Matrix4d node_T_BS_ = Eigen::Matrix4d::Identity();
  std::unique_ptr<slamko::XFeatRelocalizer> reloc_;
  std::unique_ptr<slamko::NeverLostSupervisor> supervisor_;
  std::uint64_t nl_frame_ = 0;
  slamko::SupervisorState nl_last_state_ = slamko::SupervisorState::OK;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_left_, sub_right_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> sub_lcam_, sub_rcam_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  // Deterministic-replay frame gating (see on_imu): buffer stereo frames, release
  // them only once the IMU stream has advanced past their timestamp.
  struct PendingFrame {
    sensor_msgs::msg::Image::ConstSharedPtr l, r;
    double ts;
  };
  std::deque<PendingFrame> pending_frames_;
  double latest_imu_ts_ = -std::numeric_limits<double>::infinity();
  bool   use_imu_gate_  = true;  // false in no-IMU mode → process frames immediately
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_kp_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VioNode>());
  rclcpp::shutdown();
  return 0;
}
