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
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "slamko_core/odometry_provider.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_loop/pose_graph.hpp"

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

    RCLCPP_INFO(get_logger(), "provider_fusion_node up: topic=%s frames %s->%s->%s",
                topic.c_str(), map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

  ~ProviderFusionNode() override {
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
      RCLCPP_INFO(get_logger(), "first provider keyframe @t=%.3f", s.t);
    } else if (edge) {
      // Chain-compose in the fused (map) frame: re-uses pose(from) so a future
      // optimize() correction propagates to all later nodes automatically.
      const slamko::SE3 T_map_to = graph_.pose(edge->from) * edge->T_from_to;
      graph_.addKeyframe(edge->to, T_map_to);
      graph_.addEdge(edge->from, edge->to, edge->T_from_to, edge->information, false);
      // map->odom correction target from the latest keyframe pair.
      T_map_odom_target_ = T_map_to * chain_.lastKeyframe().T_OB.inverse();
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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
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
