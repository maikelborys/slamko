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
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

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

#include "slamko_core/image_view.hpp"
#include "slamko_core/local_smoother.hpp"
#include "slamko_vio/vio_pipeline.hpp"
#include "slamko_vio/types.hpp"
#include "slamko_fusion/gtsam_local_smoother.hpp"   // node-only (composition root)

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

    if (!cfg.pose_dump_path.empty()) {
      pose_dump_.open(cfg.pose_dump_path);
      RCLCPP_INFO(get_logger(), "pose dump (TUM) -> %s", cfg.pose_dump_path.c_str());
    }

    using ImgSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
    using CamSub = message_filters::Subscriber<sensor_msgs::msg::CameraInfo>;
    sub_left_  = std::make_shared<ImgSub>(this, "left/image_rect_raw",  rmw_qos_profile_sensor_data);
    sub_right_ = std::make_shared<ImgSub>(this, "right/image_rect_raw", rmw_qos_profile_sensor_data);
    sub_lcam_  = std::make_shared<CamSub>(this, "left/camera_info",  rmw_qos_profile_sensor_data);
    sub_rcam_  = std::make_shared<CamSub>(this, "right/camera_info", rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(20), *sub_left_, *sub_right_, *sub_lcam_, *sub_rcam_);
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

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>;

  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& m) {
    slamko_vio::ImuSample s;
    s.t = (double)m->header.stamp.sec + m->header.stamp.nanosec * 1e-9;
    s.a = Eigen::Vector3d(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
    s.w = Eigen::Vector3d(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
    pipeline_->addImu(s);
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

    const slamko::ImageView left(msg_l->data.data(), (int)msg_l->width, (int)msg_l->height, (int)msg_l->step);
    const slamko::ImageView right(msg_r->data.data(), (int)msg_r->width, (int)msg_r->height, (int)msg_r->step);
    const double ts = (double)msg_l->header.stamp.sec + msg_l->header.stamp.nanosec * 1e-9;

    pipeline_->processStereo(left, right, ts, K_);
    publish(msg_l->header);
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
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_left_, sub_right_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> sub_lcam_, sub_rcam_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
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
