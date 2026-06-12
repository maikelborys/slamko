// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// provider_fusion_node — the P-A loose fuser (MASTER_PLAN §2/§8): subscribes an
// external odometry provider's nav_msgs/Odometry stream (default OKVIS2-X
// /okvis/okvis_odometry, run with do_loop_closures=false), decimates it into
// keyframes via slamko_core::ProviderChain, and feeds RELATIVE keyframe edges +
// covariance into slamko_loop::PoseGraph — slamko's thin global graph.
//
// Output contract (the slamko side of the map->odom->base TF split):
//   map -> odom   : the global correction (pose-graph fused minus raw provider),
//                   SLEWED (bounded m/s + rad/s) so Nav2-style consumers never see
//                   a discontinuity. Identity until a global constraint (loop /
//                   reloc / GNSS) lands — in P-A there are none, which is exactly
//                   the gate: fused must track the provider.
//   odom -> base  : the raw provider pose, republished as TF (param-gated; in a
//                   production stack where the provider's bridge owns this, turn
//                   publish_odom_base_tf off).
//   ~/fused_odometry : the fused pose in map frame at provider rate (correction
//                   applied exactly, not slewed — the slew is TF-cosmetic only).
//
// Bench trail: traj_fused_path / traj_provider_path dump TUM lines per sample;
// scripts/bench_pa.sh compares them (gate: max divergence ~0 in P-A).
//
// Single-threaded executor on purpose: subscription + timer share one thread, so
// the chain/graph/correction state needs no locking. The pose-graph optimize()
// is NOT called on the hot path (no global constraints yet); when P-B/P-C add
// them, optimization moves to a worker (disposable-graph rule: the provider
// passthrough must never block on the solver).

#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/imgproc.hpp>

#include "slamko_core/odometry_provider.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"
#include "slamko_core/submap_io.hpp"
#include "slamko_loop/pose_graph.hpp"
#include "slamko_vio/feature/eigenplaces.h"

namespace {

slamko::SE3 fromPoseMsg(const geometry_msgs::msg::Pose& p) {
  const Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y,
                             p.orientation.z);
  return slamko::SE3(slamko::SO3(q.normalized()),
                     Eigen::Vector3d(p.position.x, p.position.y, p.position.z));
}

void toTransformMsg(const slamko::SE3& T, geometry_msgs::msg::Transform& out) {
  const Eigen::Quaterniond q = T.so3().unit_quaternion();
  const Eigen::Vector3d t = T.translation();
  out.translation.x = t.x();
  out.translation.y = t.y();
  out.translation.z = t.z();
  out.rotation.x = q.x();
  out.rotation.y = q.y();
  out.rotation.z = q.z();
  out.rotation.w = q.w();
}

}  // namespace

class ProviderFusionNode : public rclcpp::Node {
 public:
  ProviderFusionNode() : rclcpp::Node("provider_fusion_node") {
    map_frame_  = declare_parameter("map_frame", std::string("slamko_map"));
    odom_frame_ = declare_parameter("odom_frame", std::string("slamko_odom"));
    base_frame_ = declare_parameter("base_frame", std::string("slamko_base"));
    publish_tf_ = declare_parameter("publish_tf", true);
    publish_odom_base_tf_ = declare_parameter("publish_odom_base_tf", true);
    slew_trans_ = declare_parameter("slew_trans_mps", 0.5);
    slew_rot_   = declare_parameter("slew_rot_radps", 0.5);

    slamko::ProviderChainConfig ccfg;
    ccfg.kf_min_translation = declare_parameter("kf_min_translation", ccfg.kf_min_translation);
    ccfg.kf_min_rotation    = declare_parameter("kf_min_rotation", ccfg.kf_min_rotation);
    ccfg.kf_max_dt          = declare_parameter("kf_max_dt", ccfg.kf_max_dt);
    ccfg.default_sigma_t    = declare_parameter("default_sigma_t", ccfg.default_sigma_t);
    ccfg.default_sigma_r    = declare_parameter("default_sigma_r", ccfg.default_sigma_r);
    chain_ = slamko::ProviderChain(ccfg);

    const auto fused_path = declare_parameter("traj_fused_path", std::string(""));
    const auto provider_path = declare_parameter("traj_provider_path", std::string(""));
    if (!fused_path.empty()) fused_file_ = std::fopen(fused_path.c_str(), "w");
    if (!provider_path.empty()) provider_file_ = std::fopen(provider_path.c_str(), "w");

    pub_fused_ = create_publisher<nav_msgs::msg::Odometry>("~/fused_odometry", 50);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Provider QoS: default RELIABLE (OKVIS publishes reliable). best_effort
    // accepts both — use it for providers that publish best-effort (the same
    // D455-driver lesson as slamko_vio's imu_best_effort).
    const bool best_effort = declare_parameter("provider_best_effort", false);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).durability_volatile();
    if (best_effort) qos.best_effort(); else qos.reliable();
    const auto topic = declare_parameter("odom_topic", std::string("/okvis/okvis_odometry"));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        topic, qos,
        std::bind(&ProviderFusionNode::onOdometry, this, std::placeholders::_1));

    const double tf_rate = declare_parameter("tf_rate_hz", 30.0);
    tf_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(tf_rate, 1.0)),
        std::bind(&ProviderFusionNode::onTfTimer, this));

    // ------------- P-B: keyframe images -> EigenPlaces VPR -> sealed submaps.
    // image_topic empty = whole path disabled (P-A behavior). The provider chain
    // starts producing keyframes seconds after bag start (OKVIS init is fast) —
    // the 133 s data hole of the deprecated VIO cannot recur by construction.
    const auto image_topic = declare_parameter("image_topic", std::string(""));
    map_dir_ = declare_parameter("map_dir", std::string(""));
    kf_per_submap_ = declare_parameter("kf_per_submap", 50);
    image_tol_s_ = declare_parameter("image_tol_s", 0.06);
    image_buffer_s_ = declare_parameter("image_buffer_s", 0.6);
    if (!image_topic.empty()) {
      std::string onnx_default;
      try {
        onnx_default = ament_index_cpp::get_package_share_directory("slamko_vio") +
                       "/models/eigenplaces.onnx";
      } catch (...) {}
      EigenPlacesConfig vcfg;
      vcfg.onnx_file = declare_parameter("vpr_onnx_path", onnx_default);
      vcfg.engine_file =
          declare_parameter("vpr_engine_path", std::string("/tmp/slamko_vio_eigenplaces_512.engine"));
      vpr_ = std::make_shared<EigenPlaces>(vcfg);
      if (!vpr_->build()) {
        RCLCPP_ERROR(get_logger(), "EigenPlaces build failed (onnx=%s) — VPR disabled",
                     vcfg.onnx_file.c_str());
        vpr_.reset();
      }
      auto img_qos = rclcpp::QoS(rclcpp::KeepLast(30)).durability_volatile();
      if (declare_parameter("image_best_effort", true)) img_qos.best_effort();
      else img_qos.reliable();
      sub_image_ = create_subscription<sensor_msgs::msg::Image>(
          image_topic, img_qos,
          std::bind(&ProviderFusionNode::onImage, this, std::placeholders::_1));
      if (!map_dir_.empty()) std::filesystem::create_directories(map_dir_);
      RCLCPP_INFO(get_logger(), "VPR path on: image=%s map_dir=%s kf_per_submap=%d",
                  image_topic.c_str(), map_dir_.c_str(), kf_per_submap_);
    }

    RCLCPP_INFO(get_logger(), "provider_fusion_node up: topic=%s frames %s->%s->%s",
                topic.c_str(), map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

  ~ProviderFusionNode() override {
    // Seal the trailing partial submap so a bag-end map is complete.
    if (!pending_kfs_.empty() && !map_dir_.empty()) sealSubmap();
    if (fused_file_) std::fclose(fused_file_);
    if (provider_file_) std::fclose(provider_file_);
  }

 private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    slamko::ProviderSample s;
    s.t = rclcpp::Time(msg->header.stamp).seconds();
    s.T_OB = fromPoseMsg(msg->pose.pose);
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) s.cov(i, j) = msg->pose.covariance[i * 6 + j];

    const bool first = !chain_.hasKeyframe();
    const auto edge = chain_.feed(s);
    if (first) {
      // Keyframe 0 seeds the graph; map == odom at start (correction identity).
      graph_.addKeyframe(0, s.T_OB);
      graph_.setAnchor(0);
      T_map_odom_target_ = slamko::SE3();
      have_correction_ = true;
      onKeyframe(0, s.t, s.T_OB);
      RCLCPP_INFO(get_logger(), "first provider keyframe @t=%.3f", s.t);
    } else if (edge) {
      // Chain-compose in the fused (map) frame: re-uses pose(from) so a future
      // optimize() correction propagates to all later nodes automatically.
      const slamko::SE3 T_map_to = graph_.pose(edge->from) * edge->T_from_to;
      graph_.addKeyframe(edge->to, T_map_to);
      graph_.addEdge(edge->from, edge->to, edge->T_from_to, edge->information, false);
      // map->odom correction target from the latest keyframe pair.
      T_map_odom_target_ = T_map_to * chain_.lastKeyframe().T_OB.inverse();
      onKeyframe(edge->to, s.t, T_map_to);
      if (graph_.numNodes() % 100 == 0)
        RCLCPP_INFO(get_logger(), "pose-graph: %zu nodes / %zu edges",
                    graph_.numNodes(), graph_.numEdges());
    }

    // Fused pose at provider rate: exact correction (not the slewed TF one).
    const slamko::SE3 T_map_B = T_map_odom_target_ * s.T_OB;
    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = map_frame_;
    out.child_frame_id = base_frame_;
    const Eigen::Quaterniond q = T_map_B.so3().unit_quaternion();
    const Eigen::Vector3d t = T_map_B.translation();
    out.pose.pose.position.x = t.x();
    out.pose.pose.position.y = t.y();
    out.pose.pose.position.z = t.z();
    out.pose.pose.orientation.x = q.x();
    out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z();
    out.pose.pose.orientation.w = q.w();
    out.pose.covariance = msg->pose.covariance;
    out.twist = msg->twist;
    pub_fused_->publish(out);

    dumpTum(fused_file_, s.t, T_map_B);
    dumpTum(provider_file_, s.t, s.T_OB);

    last_sample_ = s;
    have_sample_ = true;
  }

  void onTfTimer() {
    if (!publish_tf_ || !have_sample_) return;
    // Slew the published map->odom toward the target: bounded step per tick so
    // downstream consumers (costmaps, planners) never see a pose jump.
    const double dt = 1.0 / 30.0;
    const slamko::SE3 delta = T_map_odom_pub_.inverse() * T_map_odom_target_;
    Eigen::Vector3d dt_t = delta.translation();
    Eigen::Vector3d dt_r = delta.so3().log();
    const double max_t = slew_trans_ * dt, max_r = slew_rot_ * dt;
    if (dt_t.norm() > max_t) dt_t *= max_t / dt_t.norm();
    if (dt_r.norm() > max_r) dt_r *= max_r / dt_r.norm();
    T_map_odom_pub_ = T_map_odom_pub_ * slamko::SE3(slamko::SO3::exp(dt_r), dt_t);

    const auto stamp = now();
    geometry_msgs::msg::TransformStamped tf_mo;
    tf_mo.header.stamp = stamp;
    tf_mo.header.frame_id = map_frame_;
    tf_mo.child_frame_id = odom_frame_;
    toTransformMsg(T_map_odom_pub_, tf_mo.transform);
    tf_broadcaster_->sendTransform(tf_mo);

    if (publish_odom_base_tf_) {
      geometry_msgs::msg::TransformStamped tf_ob;
      tf_ob.header.stamp = stamp;
      tf_ob.header.frame_id = odom_frame_;
      tf_ob.child_frame_id = base_frame_;
      toTransformMsg(last_sample_.T_OB, tf_ob.transform);
      tf_broadcaster_->sendTransform(tf_ob);
    }
  }

  // ------------------------------------------------ P-B: images + VPR + sealing
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    cv::Mat gray;
    if (msg->encoding == "mono8") {
      gray = cv::Mat(msg->height, msg->width, CV_8UC1,
                     const_cast<std::uint8_t*>(msg->data.data()), msg->step).clone();
    } else if (msg->encoding == "bgr8" || msg->encoding == "rgb8") {
      const cv::Mat c(msg->height, msg->width, CV_8UC3,
                      const_cast<std::uint8_t*>(msg->data.data()), msg->step);
      cv::cvtColor(c, gray, msg->encoding == "bgr8" ? cv::COLOR_BGR2GRAY : cv::COLOR_RGB2GRAY);
    } else {
      RCLCPP_WARN_ONCE(get_logger(), "unsupported image encoding '%s' — VPR starved",
                       msg->encoding.c_str());
      return;
    }
    img_buf_.emplace_back(t, std::move(gray));
    while (!img_buf_.empty() && img_buf_.back().first - img_buf_.front().first > image_buffer_s_)
      img_buf_.pop_front();
  }

  // Called once per new chain keyframe (single-threaded executor — no locking).
  // EigenPlaces runs HERE, at keyframe rate (~1-2 Hz), never per frame.
  void onKeyframe(std::uint64_t id, double t, const slamko::SE3& T_map) {
    if (!vpr_) return;
    // nearest buffered image
    double best_dt = 1e9;
    const cv::Mat* best = nullptr;
    for (const auto& [it, img] : img_buf_) {
      const double dt = std::abs(it - t);
      if (dt < best_dt) { best_dt = dt; best = &img; }
    }
    if (!best || best_dt > image_tol_s_) {
      ++kf_no_image_;
      return;
    }
    Eigen::VectorXf g;
    if (!vpr_->infer(*best, g)) return;
    pending_kfs_.push_back(KfRec{id, t, T_map, std::move(g)});
    if (!map_dir_.empty() && (int)pending_kfs_.size() >= kf_per_submap_) sealSubmap();
  }

  void sealSubmap() {
    slamko::SubMap sm;
    sm.id = next_submap_id_++;
    sm.anchor = pending_kfs_.front().T_map;  // local frame = first KF of the segment
    const slamko::SE3 anchor_inv = sm.anchor.inverse();
    sm.keyframes.reserve(pending_kfs_.size());
    sm.kf_obs.resize(pending_kfs_.size());
    Eigen::VectorXf mean = Eigen::VectorXf::Zero(pending_kfs_.front().g.size());
    for (std::size_t k = 0; k < pending_kfs_.size(); ++k) {
      const auto& r = pending_kfs_[k];
      sm.keyframes.push_back(slamko::KeyframePose{r.id, r.t, anchor_inv * r.T_map});
      sm.kf_obs[k].global_descriptor = r.g;
      mean += r.g;
    }
    mean.normalize();
    sm.global_descriptor = mean;  // submap-level representative (coarse stage)
    const std::string path = map_dir_ + "/submap_" + std::to_string(sm.id) + ".smap";
    if (!slamko::saveSubMap(sm, path)) {
      RCLCPP_ERROR(get_logger(), "saveSubMap failed: %s", path.c_str());
    } else {
      sealed_ids_.push_back(sm.id);
      std::ofstream mf(map_dir_ + "/submaps.manifest");
      for (auto sid : sealed_ids_) mf << sid << "\n";
      RCLCPP_INFO(get_logger(), "sealed submap %llu (%zu KF, VPR) -> %s%s",
                  (unsigned long long)sm.id, sm.keyframes.size(), path.c_str(),
                  kf_no_image_ ? (" [" + std::to_string(kf_no_image_) + " KF w/o image]").c_str()
                               : "");
    }
    pending_kfs_.clear();
  }

  static void dumpTum(std::FILE* f, double t, const slamko::SE3& T) {
    if (!f) return;
    const Eigen::Quaterniond q = T.so3().unit_quaternion();
    const Eigen::Vector3d p = T.translation();
    std::fprintf(f, "%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", t, p.x(), p.y(),
                 p.z(), q.x(), q.y(), q.z(), q.w());
  }

  std::string map_frame_, odom_frame_, base_frame_;
  bool publish_tf_ = true, publish_odom_base_tf_ = true;
  double slew_trans_ = 0.5, slew_rot_ = 0.5;

  slamko::ProviderChain chain_;
  slamko::PoseGraph graph_;
  slamko::SE3 T_map_odom_target_, T_map_odom_pub_;
  bool have_correction_ = false;
  slamko::ProviderSample last_sample_;
  bool have_sample_ = false;

  std::FILE* fused_file_ = nullptr;
  std::FILE* provider_file_ = nullptr;

  // P-B: image buffer + VPR + sealing
  struct KfRec {
    std::uint64_t id;
    double t;
    slamko::SE3 T_map;
    Eigen::VectorXf g;
  };
  std::deque<std::pair<double, cv::Mat>> img_buf_;
  EigenPlacesPtr vpr_;
  std::vector<KfRec> pending_kfs_;
  std::vector<std::uint64_t> sealed_ids_;
  std::uint64_t next_submap_id_ = 0;
  std::string map_dir_;
  int kf_per_submap_ = 50;
  int kf_no_image_ = 0;
  double image_tol_s_ = 0.06, image_buffer_s_ = 0.6;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_fused_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProviderFusionNode>());
  rclcpp::shutdown();
  return 0;
}
