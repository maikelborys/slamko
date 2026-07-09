// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// cuvslam_provider_node — ROS 2 shell around CuvslamProvider: D455/Gazebo stereo IR
// in, nav_msgs/Odometry out on the SAME contract the fusion node already consumes
// (`provider_fusion_node odom_topic:=/cuvslam/odometry`). No TF, no /map — the
// bridge/fusion own those. Health verdicts ride BOTH the odometry covariance
// (inflated on SUSPECT, Hard Rule #3) and ~/health (debug/eval).
//
// Bring-up (bag replay, brutal casa bag):
//   ros2 run slamko_vio cuvslam_provider_node --ros-args -p image_best_effort:=true
//   ros2 bag play /mnt/data/bags/bno_ab/CASA1_brutal1_Stereo60_RGB30_BNO_trim
//   ros2 topic echo /cuvslam/health   # [obs, inliers, residual, cost, cond, suspect, valid]
// First place to look when it misbehaves: this node logs "hard loss" (throttled) on
// nullopt; no odometry at all usually means the camera_info pair never arrived
// (rig is built lazily from the first synced camera_info).

#include <array>
#include <deque>
#include <memory>
#include <optional>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "slamko_vio/cuvslam_provider.hpp"

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

class CuvslamProviderNode : public rclcpp::Node {
 public:
  CuvslamProviderNode() : Node("cuvslam_provider") {
    left_topic_ = declare_parameter("left_topic", std::string("/camera/camera/infra1/image_rect_raw"));
    right_topic_ = declare_parameter("right_topic", std::string("/camera/camera/infra2/image_rect_raw"));
    left_info_topic_ = declare_parameter("left_info_topic", std::string("/camera/camera/infra1/camera_info"));
    right_info_topic_ = declare_parameter("right_info_topic", std::string("/camera/camera/infra2/camera_info"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));

    cfg_.async_sba = declare_parameter("async_sba", true);
    cfg_.health = declare_parameter("health", true);
    cfg_.min_inliers = static_cast<int>(declare_parameter("min_inliers", 10));
    cfg_.max_info_condition = declare_parameter("max_info_condition", 1e6);
    cfg_.cov_scale = declare_parameter("cov_scale", 80.0);
    cfg_.suspect_cov_mult = declare_parameter("suspect_cov_mult", 25.0);

    // Inertial (VIO) mode. imu_scale: bno_ab casa1-original bags have DOUBLED accel
    // (fix_imu_scale gotcha) -> 0.5 there; CASA1_*_BNO measured ~8.9 = fine at 1.0.
    cfg_.use_imu = declare_parameter("use_imu", false);
    cfg_.gyro_noise_density = declare_parameter("gyro_noise_density", cfg_.gyro_noise_density);
    cfg_.gyro_random_walk = declare_parameter("gyro_random_walk", cfg_.gyro_random_walk);
    cfg_.accel_noise_density = declare_parameter("accel_noise_density", cfg_.accel_noise_density);
    cfg_.accel_random_walk = declare_parameter("accel_random_walk", cfg_.accel_random_walk);
    cfg_.imu_frequency = declare_parameter("imu_frequency", cfg_.imu_frequency);
    imu_scale_ = declare_parameter("imu_scale", 1.0);
    imu_topic_ = declare_parameter("imu_topic", std::string("/camera/camera/imu"));

    const bool best_effort = declare_parameter("image_best_effort", true);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(30)).durability_volatile();
    if (best_effort) qos.best_effort();

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/cuvslam/odometry", 50);
    pub_health_ = create_publisher<std_msgs::msg::Float64MultiArray>("/cuvslam/health", 20);

    sub_left_info_ = create_subscription<CameraInfo>(
        left_info_topic_, qos, [this](CameraInfo::SharedPtr m) { left_info_ = m; tryInit(); });
    sub_right_info_ = create_subscription<CameraInfo>(
        right_info_topic_, qos, [this](CameraInfo::SharedPtr m) { right_info_ = m; tryInit(); });

    if (cfg_.use_imu) {
      // Buffer, don't feed directly: cuVSLAM requires a MONOTONIC interleave
      // (imu... < frame_t) but the sync queue delays image pairs while 200 Hz IMU
      // races ahead -> "Timestamps are non-monotonic" throw. onStereo drains the
      // buffer up to each frame's stamp before Track.
      auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(400)).durability_volatile().best_effort();
      sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
          imu_topic_, imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr m) {
            imu_buf_.push_back(std::move(m));
            while (imu_buf_.size() > 2000) imu_buf_.pop_front();
          });
    }

    sub_l_.subscribe(this, left_topic_, qos.get_rmw_qos_profile());
    sub_r_.subscribe(this, right_topic_, qos.get_rmw_qos_profile());
    sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(Policy(30), sub_l_, sub_r_);
    sync_->registerCallback(&CuvslamProviderNode::onStereo, this);

    RCLCPP_INFO(get_logger(), "cuvslam_provider up: %s + %s -> /cuvslam/odometry",
                left_topic_.c_str(), right_topic_.c_str());
  }

 private:
  using Policy = message_filters::sync_policies::ExactTime<Image, Image>;

  void tryInit() {
    if (provider_ || !left_info_ || !right_info_) return;
    cfg_.width = static_cast<int>(left_info_->width);
    cfg_.height = static_cast<int>(left_info_->height);
    cfg_.fx = left_info_->k[0];
    cfg_.fy = left_info_->k[4];
    cfg_.cx = left_info_->k[2];
    cfg_.cy = left_info_->k[5];
    cfg_.baseline_m = -right_info_->p[3] / right_info_->p[0];  // P[3] = -fx*B
    auto p = std::make_unique<slamko::CuvslamProvider>();
    if (!p->init(cfg_)) {
      RCLCPP_ERROR(get_logger(), "CuvslamProvider init failed (w=%d h=%d fx=%.1f B=%.4f)",
                   cfg_.width, cfg_.height, cfg_.fx, cfg_.baseline_m);
      return;
    }
    provider_ = std::move(p);
    RCLCPP_INFO(get_logger(), "rig ready: %dx%d fx=%.1f baseline=%.4f m gates: inliers>=%d cond<=%.0e",
                cfg_.width, cfg_.height, cfg_.fx, cfg_.baseline_m, cfg_.min_inliers,
                cfg_.max_info_condition);
  }

  void onStereo(const Image::ConstSharedPtr& l, const Image::ConstSharedPtr& r) {
    if (!provider_) return;
    if (l->encoding != "mono8" || r->encoding != "mono8") {
      RCLCPP_ERROR_ONCE(get_logger(), "expected mono8 images, got %s", l->encoding.c_str());
      return;
    }
    const double t = rclcpp::Time(l->header.stamp).seconds();
    while (!imu_buf_.empty()) {
      const auto& m = imu_buf_.front();
      const double ti = rclcpp::Time(m->header.stamp).seconds();
      if (ti > t) break;
      provider_->registerImu(ti, m->linear_acceleration.x * imu_scale_,
                             m->linear_acceleration.y * imu_scale_,
                             m->linear_acceleration.z * imu_scale_,
                             m->angular_velocity.x, m->angular_velocity.y,
                             m->angular_velocity.z);
      imu_buf_.pop_front();
    }
    auto sample = provider_->track(t, l->data.data(), r->data.data(),
                                   static_cast<int>(l->step));
    publishHealth(t);
    if (!sample) {  // hard loss: publish NOTHING — the fusion stale-gap owns the response
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "hard tracking loss at t=%.3f", t);
      return;
    }

    nav_msgs::msg::Odometry o;
    o.header.stamp = l->header.stamp;
    o.header.frame_id = odom_frame_;
    o.child_frame_id = base_frame_;
    const auto& T = sample->T_OB;
    const Eigen::Quaterniond q = T.so3().unit_quaternion();
    o.pose.pose.position.x = T.translation().x();
    o.pose.pose.position.y = T.translation().y();
    o.pose.pose.position.z = T.translation().z();
    o.pose.pose.orientation.w = q.w();
    o.pose.pose.orientation.x = q.x();
    o.pose.pose.orientation.y = q.y();
    o.pose.pose.orientation.z = q.z();
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) o.pose.covariance[i * 6 + j] = sample->cov(i, j);
    pub_odom_->publish(o);
  }

  void publishHealth(double t) {
    const auto& h = provider_->lastHealth();
    std_msgs::msg::Float64MultiArray m;
    m.data = {t, static_cast<double>(h.observations),
              static_cast<double>(h.inliers), h.mean_residual, h.final_cost,
              h.info_condition, h.suspect ? 1.0 : 0.0, h.valid ? 1.0 : 0.0};
    pub_health_->publish(m);
  }

  std::string left_topic_, right_topic_, left_info_topic_, right_info_topic_;
  std::string odom_frame_, base_frame_, imu_topic_;
  double imu_scale_ = 1.0;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buf_;
  slamko::CuvslamProviderConfig cfg_;
  std::unique_ptr<slamko::CuvslamProvider> provider_;
  CameraInfo::SharedPtr left_info_, right_info_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_left_info_, sub_right_info_;
  message_filters::Subscriber<Image> sub_l_, sub_r_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_health_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CuvslamProviderNode>());
  rclcpp::shutdown();
  return 0;
}
