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
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <array>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/imgproc.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include "slamko_core/features.hpp"
#include "slamko_core/loop_consensus.hpp"
#include "slamko_core/odometry_provider.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"
#include "slamko_core/submap_io.hpp"
#include "slamko_loop/pose_graph.hpp"
#include "slamko_loop/xfeat_relocalizer.hpp"
#include "slamko_vio/feature/eigenplaces.h"
#include "slamko_vio/feature/xfeat.h"

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
    const auto global_path = declare_parameter("traj_global_path", std::string(""));
    graph_path_ = declare_parameter("traj_graph_path", std::string(""));
    if (!fused_path.empty()) fused_file_ = std::fopen(fused_path.c_str(), "w");
    if (!provider_path.empty()) provider_file_ = std::fopen(provider_path.c_str(), "w");
    if (!global_path.empty()) global_file_ = std::fopen(global_path.c_str(), "w");

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
    // Never-lost branch (R-C): odom stale-gap that counts as tracking loss, and a
    // force-loss TEST window [start,end] (bag-relative s) that drops odom to simulate it.
    stale_thresh_ = declare_parameter("stale_gap_s", 0.5);
    // OKVIS pos-covariance trace above this = Marginal/Lost tracking (base ~3e-3,
    // x10 Marginal, x100 Lost) -> the segment's placement is soft (principled signal).
    cov_soft_thresh_ = declare_parameter("cov_soft_thresh", 0.01);
    force_loss_start_ = declare_parameter("force_loss_start", -1.0);
    force_loss_end_ = declare_parameter("force_loss_end", -1.0);
    image_tol_s_ = declare_parameter("image_tol_s", 0.06);
    // Provider odometry arrives with estimation latency (worse under GPU load —
    // the 0.6 s default starved 235/1300 KFs of their image on CASA1_Suave).
    image_buffer_s_ = declare_parameter("image_buffer_s", 2.5);
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

      // ---------- P-B step 2b: XFeat stereo landmarks + relocalization -> loop
      // edges. Needs the right image + both camera_infos; the relocalizer is
      // built lazily once the intrinsics arrive. XFeat ONNX is STATIC 752x480
      // (non-native input -> 0 keypoints) and the D455 IR is 848x480, so both
      // images are CENTER-CROPPED to 752 and the canonical camera model of
      // everything downstream (kp uv, PnP intrinsics, stored submap obs) is the
      // cropped one: cx' = cx - crop_x.
      reloc_enabled_ = declare_parameter("reloc", true);
      min_loop_gap_s_ = declare_parameter("min_loop_gap_s", 25.0);
      reloc_min_inliers_ = declare_parameter("reloc_min_inliers", 25);
      max_kf_landmarks_ = declare_parameter("max_kf_landmarks", 200);
      // ORB-SLAM-style structure-only map cleanup at seal (task #9): voxel size for
      // landmark dedup, and min observations to survive culling (the one-off rays).
      lm_dedup_voxel_ = declare_parameter("lm_dedup_voxel_m", 0.06);
      lm_min_obs_ = declare_parameter("lm_min_obs", 2);
      // Inter-map anchor edges (R1.1 hard / R1.3 soft): chain edge sigma between
      // consecutive submaps (good odometry) vs SOFT sigma when the segment crossed
      // a visual loss (dead-reckoning — approximate placement only). Hard = loop sigma.
      anchor_chain_sigma_t_ = declare_parameter("anchor_chain_sigma_t", 0.05);
      anchor_chain_sigma_r_ = declare_parameter("anchor_chain_sigma_r", 0.02);
      anchor_soft_sigma_t_ = declare_parameter("anchor_soft_sigma_t", 1.0);
      anchor_soft_sigma_r_ = declare_parameter("anchor_soft_sigma_r", 0.3);
      anchor_soft_lm_ = declare_parameter("anchor_soft_lm", 7000);
      loop_sigma_t_ = declare_parameter("loop_sigma_t", 0.10);
      loop_sigma_r_ = declare_parameter("loop_sigma_r", 0.05);
      max_loop_disagree_m_ = declare_parameter("max_loop_disagree_m", 30.0);
      slamko::LoopConsensusConfig gcfg;
      gcfg.tol_t = declare_parameter("pcm_tol_t", gcfg.tol_t);
      gcfg.tol_r = declare_parameter("pcm_tol_r", gcfg.tol_r);
      gcfg.required_consec = declare_parameter("pcm_consec", gcfg.required_consec);
      gcfg.cooldown_s = declare_parameter("loop_cooldown_s", gcfg.cooldown_s);
      pcm_consec_ = gcfg.required_consec;
      gate_ = slamko::LoopConsensusGate(gcfg);

      // Cross-session: load a prior map; matches into it re-anchor this session
      // into the prior's global frame (anchor-don't-weld — the session graph is
      // untouched, only T_global<-map is estimated).
      prior_min_inliers_ = declare_parameter("prior_min_inliers", 15);
      reanchor_jump_m_ = declare_parameter("reanchor_jump_m", 0.5);
      min_reloc_period_s_ = declare_parameter("min_reloc_period_s", 0.5);
      lg_model_path_ = declare_parameter(
          "lightglue_model_path",
          onnx_default.empty() ? std::string()
                               : onnx_default.substr(0, onnx_default.rfind('/')) +
                                     "/lighterglue.pt");
      const auto prior_dir = declare_parameter("prior_map_dir", std::string(""));
      if (!prior_dir.empty()) {
        if (slamko::loadSubMaps(prior_submaps_, prior_dir)) {
          for (const auto& sm : prior_submaps_) {
            prior_ids_.insert(sm.id);
            prior_anchor_[sm.id] = sm.anchor;
            next_submap_id_ = std::max(next_submap_id_, sm.id + 1);
          }
          RCLCPP_INFO(get_logger(), "prior map loaded: %zu submaps from %s",
                      prior_submaps_.size(), prior_dir.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "prior map FAILED to load: %s", prior_dir.c_str());
        }
      }
      // OKVIS rsD455 T_SC cam0 (body=S -> cam): identity rotation + this offset.
      const std::vector<double> btc = declare_parameter(
          "body_t_cam_xyz", std::vector<double>{-0.03022, 0.0074, 0.01602});
      body_T_cam_ = slamko::SE3(slamko::SO3(), Eigen::Vector3d(btc[0], btc[1], btc[2]));
      if (reloc_enabled_) {
        XFeatConfig xcfg;
        xcfg.onnx_file = declare_parameter(
            "xfeat_onnx_path",
            onnx_default.empty() ? std::string()
                                 : onnx_default.substr(0, onnx_default.rfind('/')) + "/xfeat.onnx");
        xcfg.engine_file =
            declare_parameter("xfeat_engine_path", std::string("/tmp/slamko_vio_xfeat_752.engine"));
        xcfg.max_keypoints = declare_parameter("xfeat_max_keypoints", 800);
        xfeat_ = std::make_shared<XFeat>(xcfg);
        if (!xfeat_->build()) {
          RCLCPP_ERROR(get_logger(), "XFeat build failed (onnx=%s) — reloc disabled",
                       xcfg.onnx_file.c_str());
          xfeat_.reset();
          reloc_enabled_ = false;
        }
        const auto right_topic =
            declare_parameter("right_image_topic",
                              std::string("/camera/camera/infra2/image_rect_raw"));
        sub_image_r_ = create_subscription<sensor_msgs::msg::Image>(
            right_topic, img_qos,
            std::bind(&ProviderFusionNode::onImageRight, this, std::placeholders::_1));
        auto info_qos = rclcpp::QoS(rclcpp::KeepLast(5)).durability_volatile().best_effort();
        sub_info_l_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            declare_parameter("left_info_topic",
                              std::string("/camera/camera/infra1/camera_info")),
            info_qos, [this](sensor_msgs::msg::CameraInfo::SharedPtr m) { onInfo(m, false); });
        sub_info_r_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            declare_parameter("right_info_topic",
                              std::string("/camera/camera/infra2/camera_info")),
            info_qos, [this](sensor_msgs::msg::CameraInfo::SharedPtr m) { onInfo(m, true); });
      }
    }

    RCLCPP_INFO(get_logger(), "provider_fusion_node up: topic=%s frames %s->%s->%s",
                topic.c_str(), map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

  ~ProviderFusionNode() override {
    // Seal the trailing partial submap so a bag-end map is complete.
    if (!pending_kfs_.empty() && !map_dir_.empty()) sealSubmap();
    // AUDIT FIX: export the OPTIMIZED graph trajectory (the honest shape — the
    // per-sample fused.tum is the causal online trail, never retro-corrected)
    // and refresh every sealed submap's anchor from the final graph so the
    // persisted map is internally consistent (kills the doubled walls the
    // optimization already knew how to fix).
    if (!graph_path_.empty()) {
      if (std::FILE* f = std::fopen(graph_path_.c_str(), "w")) {
        for (const auto& [id, T] : graph_.poses()) {
          const auto it = node_time_.find(id);
          if (it != node_time_.end()) dumpTum(f, it->second, T);
        }
        std::fclose(f);
      }
    }
    if (!map_dir_.empty()) {
      int refreshed = 0;
      for (auto sid : sealed_ids_) {
        const std::string path = map_dir_ + "/submap_" + std::to_string(sid) + ".smap";
        slamko::SubMap sm;
        if (!slamko::loadSubMap(sm, path)) continue;
        const auto it = submap_first_kf_.find(sid);
        if (it == submap_first_kf_.end() || !graph_.hasNode(it->second)) continue;
        sm.anchor = graph_.pose(it->second);
        if (slamko::saveSubMap(sm, path)) ++refreshed;
      }
      RCLCPP_INFO(get_logger(), "shutdown: refreshed %d/%zu sealed anchors from the graph",
                  refreshed, sealed_ids_.size());
    }
    if (fused_file_) std::fclose(fused_file_);
    if (provider_file_) std::fclose(provider_file_);
    if (global_file_) std::fclose(global_file_);
  }

 private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    slamko::ProviderSample s;
    s.t = rclcpp::Time(msg->header.stamp).seconds();
    s.T_OB = fromPoseMsg(msg->pose.pose);
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) s.cov(i, j) = msg->pose.covariance[i * 6 + j];

    // --- never-lost loss detection + branch (R-C) ---
    if (t0_ < 0.0) t0_ = s.t;
    const double rel = s.t - t0_;
    // force-loss TEST window: drop odom to simulate an OKVIS stall (-> stale-gap).
    if (force_loss_start_ >= 0.0 && rel >= force_loss_start_ && rel < force_loss_end_)
      return;
    // a stale-gap since the last accepted sample = tracking loss. Seal the current
    // submap (so the loss sits at a boundary = a branch) THEN flag the next chain
    // edge SOFT (its placement across the gap is dead-reckoned, not trustworthy).
    if (last_odom_t_ >= 0.0 && s.t - last_odom_t_ > stale_thresh_) {
      RCLCPP_WARN(get_logger(),
                  "TRACKING LOSS: odom stale-gap %.2fs @t=%.1f -> seal+branch (soft edge)",
                  s.t - last_odom_t_, rel);
      if (!map_dir_.empty() && pending_kfs_.size() >= 2) sealSubmap();
      loss_in_segment_ = true;
    }
    last_odom_t_ = s.t;
    // OKVIS-reported degraded tracking (covariance inflated, Marginal/Lost) also
    // marks the segment SOFT — the principled signal (not just a landmark proxy).
    if (s.cov(0, 0) + s.cov(1, 1) + s.cov(2, 2) > cov_soft_thresh_) loss_in_segment_ = true;

    const bool first = !chain_.hasKeyframe();
    const auto edge = chain_.feed(s);
    if (first) {
      // Keyframe 0 seeds the graph; map == odom at start (correction identity).
      graph_.addKeyframe(0, s.T_OB);
      graph_.setAnchor(0);
      node_time_[0] = s.t;
      T_map_odom_target_ = slamko::SE3();
      have_correction_ = true;
      onKeyframe(0, s.t, s.T_OB);
      RCLCPP_INFO(get_logger(), "first provider keyframe @t=%.3f", s.t);
    } else if (edge) {
      // Chain-compose in the fused (map) frame: re-uses pose(from) so a future
      // optimize() correction propagates to all later nodes automatically.
      const slamko::SE3 T_map_to = graph_.pose(edge->from) * edge->T_from_to;
      graph_.addKeyframe(edge->to, T_map_to);
      node_time_[edge->to] = s.t;
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
    // Pose in the PRIOR map's global frame — only meaningful once localized
    // (cross-session fusion trail; before that the session frame is floating).
    if (localized_) dumpTum(global_file_, s.t, T_global_map_ * T_map_B);

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

    // Cross-session re-anchor: prior-map global frame above the session map.
    if (localized_) {
      geometry_msgs::msg::TransformStamped tf_gm;
      tf_gm.header.stamp = stamp;
      tf_gm.header.frame_id = "slamko_global";
      tf_gm.child_frame_id = map_frame_;
      toTransformMsg(T_global_map_, tf_gm.transform);
      tf_broadcaster_->sendTransform(tf_gm);
    }
  }

  // ------------------------------------------------ P-B: images + VPR + sealing
  struct KfRec {
    std::uint64_t id;
    double t;
    slamko::SE3 T_map;
    Eigen::VectorXf g;
    // Stereo-triangulated XFeat landmarks (cropped-camera-model uv, camera-frame 3D).
    Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> lm_uv;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> lm_desc;
    std::vector<Eigen::Vector3d> lm_pcam;
  };

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

  void onImageRight(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->encoding != "mono8") return;
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    cv::Mat gray = cv::Mat(msg->height, msg->width, CV_8UC1,
                           const_cast<std::uint8_t*>(msg->data.data()), msg->step).clone();
    img_buf_r_.emplace_back(t, std::move(gray));
    while (!img_buf_r_.empty() &&
           img_buf_r_.back().first - img_buf_r_.front().first > image_buffer_s_)
      img_buf_r_.pop_front();
  }

  void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr m, bool right) {
    if (have_calib_ || !reloc_enabled_) return;
    if (!right) {
      fx_ = m->p[0]; fy_ = m->p[5]; cx_ = m->p[2]; cy_ = m->p[6];
      img_w_ = m->width;
      have_info_l_ = true;
    } else {
      // P[0,3] = -fx * baseline for the right camera of a rectified pair.
      baseline_ = -m->p[3] / m->p[0];
      have_info_r_ = true;
    }
    if (have_info_l_ && have_info_r_) {
      have_calib_ = true;
      crop_x_ = std::max(0, (int)img_w_ - 752) / 2;
      slamko::XFeatRelocConfig rcfg;
      rcfg.fx = fx_; rcfg.fy = fy_; rcfg.cx = cx_ - crop_x_; rcfg.cy = cy_;
      rcfg.body_T_cam = body_T_cam_;
      rcfg.min_inliers = reloc_min_inliers_;
      rcfg.use_bow = false;  // VPR per-KF ranking is the candidate stage (P-B verdict)
      // TWO relocalizers: same-session imagery always out-scores a prior map's
      // (different day/light/walk) in PnP inliers, so a single best-of-all
      // relocalizer NEVER surfaces the prior once own submaps exist (learned on
      // fusion_suave_on_escaleras: every match went to the just-aged session
      // submap). Querying session and prior separately gives each its own best;
      // both feed the same consensus gate.
      reloc_ = std::make_unique<slamko::XFeatRelocalizer>(rcfg);
      if (!prior_submaps_.empty()) {
        // Cross-bag verify needs the viewpoint-robust matcher: XFeat NN-brute
        // never reaches min_inliers across walks (proven: VPR cos 0.71 to the
        // right submap, zero NN-PnP candidates). LighterGlue + a lower inlier
        // bar (the consensus gate is the precision defense) fixes the verify.
        auto rcfg_p = rcfg;
        rcfg_p.min_inliers = prior_min_inliers_;
        rcfg_p.use_lightglue = true;
        rcfg_p.lightglue_model_path = lg_model_path_;
        reloc_prior_ = std::make_unique<slamko::XFeatRelocalizer>(rcfg_p);
        for (const auto& sm : prior_submaps_) reloc_prior_->addSubMap(sm);
        prior_submaps_.clear();  // registered; keep only ids/anchors
      }
      RCLCPP_INFO(get_logger(),
                  "reloc ready: fx=%.1f cx'=%.1f baseline=%.4f m crop_x=%d top-10 VPR + PnP"
                  " (%zu prior submaps registered)",
                  fx_, cx_ - crop_x_, baseline_, crop_x_, prior_ids_.size());
    }
  }

  // XFeat needs its STATIC 752x480 input. Wider sensors (848) are center-cropped
  // (crop_x_ > 0, canonical cx' = cx - crop_x); narrower ones (640) are RIGHT-
  // padded with black — padding on the right leaves the principal point alone
  // (cx' = cx), and keypoints landing in the pad are dropped. Output keypoints
  // are in the canonical (cropped/original) pixel frame.
  bool detect(const cv::Mat& full, slamko::Features& out) {
    constexpr int W = 752;
    cv::Mat view;
    if (full.cols >= W) {
      view = full(cv::Rect(crop_x_, 0, W, full.rows));
    } else {
      view = cv::Mat::zeros(full.rows, W, CV_8UC1);
      full.copyTo(view(cv::Rect(0, 0, full.cols, full.rows)));
    }
    Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic> f;
    if (!xfeat_->infer(view, f) || f.cols() == 0) return false;
    const float x_max = (full.cols >= W ? W : full.cols) - 2.0f;
    const int n = (int)f.cols();
    out.keypoints.resize(n, 3);
    out.descriptors.resize(n, 64);
    int k = 0;
    for (int j = 0; j < n; ++j) {
      if (f(1, j) > x_max) continue;  // pad region
      out.keypoints(k, 0) = f(1, j);
      out.keypoints(k, 1) = f(2, j);
      out.keypoints(k, 2) = f(0, j);
      out.descriptors.row(k) = f.block<64, 1>(3, j).transpose();
      ++k;
    }
    out.keypoints.conservativeResize(k, 3);
    out.descriptors.conservativeResize(k, 64);
    return k > 0;
  }

  // Called once per new chain keyframe (single-threaded executor — no locking).
  // EigenPlaces + XFeat run HERE, at keyframe rate (~1-5 Hz), never per frame.
  void onKeyframe(std::uint64_t id, double t, const slamko::SE3& T_map) {
    if (!vpr_) return;
    auto nearest = [](const std::deque<std::pair<double, cv::Mat>>& buf, double tq,
                      double tol) -> const std::pair<double, cv::Mat>* {
      double best_dt = 1e9;
      const std::pair<double, cv::Mat>* best = nullptr;
      for (const auto& e : buf)
        if (std::abs(e.first - tq) < best_dt) { best_dt = std::abs(e.first - tq); best = &e; }
      return (best && best_dt <= tol) ? best : nullptr;
    };
    const auto* left_e = nearest(img_buf_, t, image_tol_s_);
    if (!left_e) { ++kf_no_image_; return; }
    const cv::Mat* left = &left_e->second;
    Eigen::VectorXf g;
    if (!vpr_->infer(*left, g)) return;

    KfRec rec{id, t, T_map, std::move(g), {}, {}, {}};

    if (reloc_enabled_ && have_calib_ && xfeat_) {
      slamko::Features ql;
      if (detect(*left, ql)) {
        // Stereo: the right image must be the SAME hardware-synced frame as the
        // left (AUDIT FIX: matching both independently to the KF stamp could
        // pair left N with right N±1 — cm of motion parallax = ghost depth).
        const auto* right_e = nearest(img_buf_r_, left_e->first, 0.005);
        slamko::Features qr;
        if (right_e && detect(right_e->second, qr)) stereoLandmarks(ql, qr, rec);
        ql.global_descriptor = rec.g;
        // Reloc THROTTLE (the GPU-contention fix): attempting on EVERY chain KF
        // (5-9 Hz walking) starves a co-running provider — the verify stage
        // (NN brute / LighterGlue x top-10 candidates) is the hog, and the
        // consensus gate works fine at 2 Hz (3 consistent hits = 1.5 s).
        // Detection stays per-KF (cheap, and the map needs the landmarks).
        if (t - last_reloc_attempt_t_ >= min_reloc_period_s_) {
          last_reloc_attempt_t_ = t;
          tryRelocalize(id, t, ql);
        }
      }
    }

    pending_kfs_.push_back(std::move(rec));
    registerAgedSubmaps(t);
    if (!map_dir_.empty() && (int)pending_kfs_.size() >= kf_per_submap_) sealSubmap();
  }

  // Mutual-best NN stereo match (row tolerance 2 px, disparity 0.5..200 px) ->
  // triangulate in the CAMERA frame; keep the strongest max_kf_landmarks_.
  void stereoLandmarks(const slamko::Features& l, const slamko::Features& r, KfRec& rec) {
    if (r.size() == 0 || l.size() == 0 || baseline_ <= 0.0) return;
    const Eigen::MatrixXf sim = l.descriptors * r.descriptors.transpose();
    std::vector<std::tuple<float, int, int>> cand;  // score, li, ri
    for (int i = 0; i < l.size(); ++i) {
      int best_j = -1;
      float best = 0.80f;  // descriptor cosine floor
      for (int j = 0; j < r.size(); ++j) {
        if (std::abs(l.keypoints(i, 1) - r.keypoints(j, 1)) > 2.0f) continue;
        const float d = l.keypoints(i, 0) - r.keypoints(j, 0);
        if (d < 0.5f || d > 200.0f) continue;
        if (sim(i, j) > best) { best = sim(i, j); best_j = j; }
      }
      if (best_j < 0) continue;
      // mutual check along the same row
      bool mutual = true;
      for (int i2 = 0; i2 < l.size(); ++i2)
        if (i2 != i && sim(i2, best_j) > sim(i, best_j) &&
            std::abs(l.keypoints(i2, 1) - r.keypoints(best_j, 1)) <= 2.0f) { mutual = false; break; }
      if (mutual) cand.emplace_back(l.keypoints(i, 2), i, best_j);
    }
    std::sort(cand.rbegin(), cand.rend());
    if ((int)cand.size() > max_kf_landmarks_) cand.resize(max_kf_landmarks_);
    rec.lm_uv.resize(cand.size(), 2);
    rec.lm_desc.resize(cand.size(), 64);
    rec.lm_pcam.reserve(cand.size());
    int k = 0;
    const double cxp = cx_ - crop_x_;
    for (const auto& [score, i, j] : cand) {
      (void)score;
      const double d = l.keypoints(i, 0) - r.keypoints(j, 0);
      const double z = fx_ * baseline_ / d;
      if (z < 0.25 || z > 12.0) continue;
      rec.lm_uv(k, 0) = l.keypoints(i, 0);
      rec.lm_uv(k, 1) = l.keypoints(i, 1);
      rec.lm_desc.row(k) = l.descriptors.row(i);
      rec.lm_pcam.emplace_back((l.keypoints(i, 0) - cxp) * z / fx_,
                               (l.keypoints(i, 1) - cy_) * z / fy_, z);
      ++k;
    }
    rec.lm_uv.conservativeResize(k, 2);
    rec.lm_desc.conservativeResize(k, 64);
  }

  // Query the relocalizer against AGED sealed submaps; on a verified match add
  // a robust loop edge + re-optimize. Gates (P-B; reversibility lands in P-C):
  // PnP inliers >= reloc_min_inliers_ AND the matched submap is older than
  // min_loop_gap_s_ (adjacent-corridor matches are not loops).
  void tryRelocalize(std::uint64_t q_id, double t, const slamko::Features& query) {
    if (reloc_ && reloc_->numSubMaps() > 0)
      processRelocResult(reloc_->relocalize(query), q_id, t);
    if (reloc_prior_)
      processRelocResult(reloc_prior_->relocalize(query), q_id, t);
  }

  void processRelocResult(const slamko::RelocResult& r, std::uint64_t q_id, double t) {
    if (r.found)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "reloc attempt: kf %llu best=submap %llu inliers=%d",
                           (unsigned long long)q_id, (unsigned long long)r.submap_id,
                           r.num_inliers);
    const bool is_prior = r.found && prior_ids_.count(r.submap_id) > 0;
    if (!r.found ||
        r.num_inliers < (is_prior ? prior_min_inliers_ : reloc_min_inliers_))
      return;
    if (!is_prior) {
      // In-session: only AGED submaps count as loops (adjacent corridor isn't a
      // loop) + a loose teleport bound vs the graph (30 m default — NOT a tight
      // absolute gate; see LoopConsensusGate header for why those are wrong).
      const auto it = submap_last_t_.find(r.submap_id);
      if (it == submap_last_t_.end() || t - it->second < min_loop_gap_s_) return;
      const slamko::SE3 T_a_q_graph =
          graph_.pose(submap_first_kf_.at(r.submap_id)).inverse() * graph_.pose(q_id);
      if ((T_a_q_graph.inverse() * r.T_query_match).translation().norm() >
          max_loop_disagree_m_)
        return;
    }

    // Consensus gate (PCM-lite, slamko_core::LoopConsensusGate — unit-tested):
    // pcm_consec_ consecutive same-target matches, pairwise-consistent under the
    // provider's relative odometry. Drift-magnitude-agnostic by design.
    const bool accept = gate_.feed(slamko::LoopCandidate{
        r.submap_id, q_id, t, r.T_query_match, chain_.lastKeyframe().T_OB});
    if (!accept) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "loop candidate: kf %llu -> %ssubmap %llu inliers=%d streak=%d/%d",
                           (unsigned long long)q_id, is_prior ? "PRIOR " : "",
                           (unsigned long long)r.submap_id, r.num_inliers,
                           gate_.streak(r.submap_id), pcm_consec_);
      return;
    }

    if (is_prior) {
      // Cross-session: re-anchor this session into the prior map's global frame
      // (anchor-don't-weld — the session graph is untouched).
      //
      // Correction-consensus (the RTABmap Suave+Escaleras lesson, MULTISESSION_
      // FUSION.md): cross-FLOOR aliased matches are SELF-consistent (they pass
      // PCM) and showed up here as sudden 2.6-8 m T_global_map jumps. Small
      // refinements (< reanchor_jump_m) apply directly; a BIG correction only
      // applies when TWO consecutive accepted re-anchors agree on it (a real
      // drift correction repeats; an aliased floor jumps somewhere else next).
      const slamko::SE3 T_global_q = prior_anchor_.at(r.submap_id) * r.T_query_match;
      const slamko::SE3 T_new = T_global_q * graph_.pose(q_id).inverse();
      const char* tag = "LOCALIZED";
      if (!localized_) {
        T_global_map_ = T_new;
        localized_ = true;
      } else {
        const double jump = (T_global_map_.inverse() * T_new).translation().norm();
        if (jump < reanchor_jump_m_) {
          T_global_map_ = T_new;
          have_big_cand_ = false;
          tag = "RE-ANCHORED";
        } else if (have_big_cand_ &&
                   (T_big_cand_.inverse() * T_new).translation().norm() <
                       reanchor_jump_m_) {
          T_global_map_ = T_new;
          have_big_cand_ = false;
          tag = "BIG CORRECTION applied (2-vote)";
        } else {
          T_big_cand_ = T_new;
          have_big_cand_ = true;
          RCLCPP_WARN(get_logger(),
                      "re-anchor JUMP held (%.2f m, needs a 2nd agreeing vote): "
                      "kf %llu -> prior submap %llu inliers=%d t=[%.2f %.2f %.2f]",
                      jump, (unsigned long long)q_id,
                      (unsigned long long)r.submap_id, r.num_inliers,
                      T_new.translation().x(), T_new.translation().y(),
                      T_new.translation().z());
          return;
        }
      }
      const auto& gt = T_global_map_.translation();
      RCLCPP_INFO(get_logger(),
                  "%s in prior map: kf %llu -> prior submap %llu inliers=%d  "
                  "T_global_map t=[%.3f %.3f %.3f]",
                  tag, (unsigned long long)q_id, (unsigned long long)r.submap_id,
                  r.num_inliers, gt.x(), gt.y(), gt.z());
      return;
    }

    const std::uint64_t a = submap_first_kf_.at(r.submap_id);
    // Submap-local frame == its first KF's body frame (sealing convention), so
    // T_query_match IS the a->query relative measurement.
    graph_.addLoopEdge(a, q_id, r.T_query_match, loop_sigma_t_, loop_sigma_r_);
    const auto res = graph_.optimize();
    T_map_odom_target_ = graph_.pose(q_id) * chain_.lastKeyframe().T_OB.inverse();
    RCLCPP_INFO(get_logger(),
                "LOOP CLOSED: kf %llu -> submap %llu (kf %llu), inliers=%d | optimize: "
                "converged=%d iters=%d cost %.2e -> %.2e",
                (unsigned long long)q_id, (unsigned long long)r.submap_id,
                (unsigned long long)a, r.num_inliers, (int)res.converged,
                res.iterations, res.initial_cost, res.final_cost);
    ++loops_closed_;
    // R1.1 HARD inter-map edge: the building submap (its future id == next_submap_id_)
    // is verified-connected to target submap r.submap_id. Recorded for the anchor
    // graph + the multi-map viz (reversible: it is just an entry we can drop).
    anchor_edges_.push_back(AnchorEdge{next_submap_id_, r.submap_id, r.T_query_match,
                                       loop_sigma_t_, loop_sigma_r_, 2});
    dumpAnchorEdges();
  }

  // Persist the inter-map anchor edges (federation-of-islands connections) so the
  // multi-map viz (scripts/plot_multimap.py) can draw soft vs hard connections.
  void dumpAnchorEdges() {
    if (map_dir_.empty()) return;
    std::ofstream f(map_dir_ + "/anchor_edges.csv");
    f << "from,to,type,tx,ty,tz,qx,qy,qz,qw,sigma_t,sigma_r\n";
    for (const auto& e : anchor_edges_) {
      const auto p = e.T_from_to.translation();
      const auto q = e.T_from_to.so3().unit_quaternion();
      f << e.from << ',' << e.to << ',' << e.type << ',' << p.x() << ',' << p.y()
        << ',' << p.z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ','
        << q.w() << ',' << e.sigma_t << ',' << e.sigma_r << '\n';
    }
  }

  // Register sealed submaps into the relocalizer only once they are OLDER than
  // the loop gap — relocalize() then never sees "places" it just left.
  void registerAgedSubmaps(double now_t) {
    while (!sealed_unregistered_.empty() &&
           now_t - sealed_unregistered_.front().second > min_loop_gap_s_) {
      if (reloc_) reloc_->addSubMap(sealed_unregistered_.front().first);
      sealed_unregistered_.pop_front();
    }
  }

  void sealSubmap() {
    slamko::SubMap sm;
    sm.id = next_submap_id_++;
    // AUDIT FIX (2026-06-12): seal from the LIVE graph poses, never the cached
    // KfRec.T_map — when a loop optimizes mid-segment the cache mixes pre/post
    // correction poses inside one submap (smeared landmarks, corrupted
    // submap-local frame, poisoned future PnP against this submap).
    sm.anchor = graph_.pose(pending_kfs_.front().id);
    const slamko::SE3 anchor_inv = sm.anchor.inverse();
    // R1.3/R1.1 CHAIN anchor edge: connect the previous sealed submap to this one.
    // SOFT (high cov) if the segment crossed a visual loss (placement is
    // dead-reckoned, approximate — don't trust its geometry); else a tight ODOM
    // edge. The HARD verified edges come from welds (processRelocResult).
    if (have_prev_anchor_) {
      int seg_lms = 0;
      for (const auto& r : pending_kfs_) seg_lms += (int)r.lm_pcam.size();
      // SOFT if the segment was visually degraded — low landmark yield (the odom
      // across it is less trustworthy, OKVIS covariance was inflated) or images
      // missing → placement is approximate, don't trust its geometry. Else tight ODOM.
      const bool soft = loss_in_segment_ || seg_lms < anchor_soft_lm_ ||
                        (kf_no_image_ - seg_kf_no_image_) > 0;
      anchor_edges_.push_back(AnchorEdge{
          prev_anchor_id_, sm.id, prev_anchor_.inverse() * sm.anchor,
          soft ? anchor_soft_sigma_t_ : anchor_chain_sigma_t_,
          soft ? anchor_soft_sigma_r_ : anchor_chain_sigma_r_, soft ? 1 : 0});
      loss_in_segment_ = false;  // consumed by this chain edge
    }
    prev_anchor_ = sm.anchor;
    prev_anchor_id_ = sm.id;
    have_prev_anchor_ = true;
    seg_kf_no_image_ = kf_no_image_;
    sm.keyframes.reserve(pending_kfs_.size());
    sm.kf_obs.resize(pending_kfs_.size());
    Eigen::VectorXf mean = Eigen::VectorXf::Zero(pending_kfs_.front().g.size());

    // Phase 1 — gather the raw per-KF stereo landmarks in submap-local frame.
    // raw landmarks are created in (k outer, i inner) order; raw_ki keeps (k,i) so
    // we can pick a representative descriptor and rebuild per-KF observations.
    std::vector<Eigen::Vector3d> raw_p;
    std::vector<std::pair<int, int>> raw_ki;
    int total_lms = 0;
    for (const auto& r : pending_kfs_) total_lms += (int)r.lm_pcam.size();
    raw_p.reserve(total_lms);
    raw_ki.reserve(total_lms);
    for (std::size_t k = 0; k < pending_kfs_.size(); ++k) {
      const auto& r = pending_kfs_[k];
      const slamko::SE3 T_local_b = anchor_inv * graph_.pose(r.id);
      sm.keyframes.push_back(slamko::KeyframePose{r.id, r.t, T_local_b});
      sm.kf_obs[k].global_descriptor = r.g;
      mean += r.g;
      const slamko::SE3 T_local_cam = T_local_b * body_T_cam_;
      for (int i = 0; i < (int)r.lm_pcam.size(); ++i) {
        raw_p.push_back(T_local_cam * r.lm_pcam[i]);
        raw_ki.emplace_back((int)k, i);
      }
    }
    mean.normalize();
    sm.global_descriptor = mean;  // submap-level representative (coarse stage)

    // Phase 2 — ORB-SLAM-style data association as voxel dedup + culling
    // (structure-only, OKVIS poses FIXED; task #9 / PLAN_ROBUSTNESS_01). The SAME
    // physical feature triangulated from N keyframes lands in ONE voxel -> merge to
    // its centroid (multi-view refine) with obs count N. A far/uncertain "ray" has
    // its per-KF depth noise SPREAD across voxels -> 1 hit each -> culled by
    // lm_min_obs_. One pass dedups ~3-4x AND removes the rays. No BA: the metric
    // estimation stays in the provider; here we only de-duplicate structure.
    const double vox = lm_dedup_voxel_;
    auto vkey = [vox](const Eigen::Vector3d& p) -> std::int64_t {
      auto q = [vox](double x) { return (std::int64_t)std::llround(std::floor(x / vox)); };
      const std::int64_t a = q(p.x()) & 0x1FFFFF, b = q(p.y()) & 0x1FFFFF, c = q(p.z()) & 0x1FFFFF;
      return (a << 42) | (b << 21) | c;
    };
    std::unordered_map<std::int64_t, std::vector<int>> cells;
    for (int idx = 0; idx < (int)raw_p.size(); ++idx) cells[vkey(raw_p[idx])].push_back(idx);

    std::vector<int> raw_row(raw_p.size(), -1);   // raw idx -> merged row (-1 = culled)
    std::vector<std::pair<int, int>> rep;         // merged row -> representative (k,i)
    int row = 0;
    for (auto& kv : cells) {
      auto& idxs = kv.second;
      if ((int)idxs.size() < lm_min_obs_) continue;     // CULL: seen < lm_min_obs_ times
      Eigen::Vector3d c = Eigen::Vector3d::Zero();
      for (int ri : idxs) c += raw_p[ri];
      c /= (double)idxs.size();                          // centroid = multi-view refine
      sm.landmarks.push_back(slamko::MapLandmark{next_landmark_id_++, c, row});
      rep.push_back(raw_ki[idxs[0]]);
      for (int ri : idxs) raw_row[ri] = row;
      ++row;
    }
    sm.descriptors.resize(row, 64);
    for (int rr = 0; rr < row; ++rr)
      sm.descriptors.row(rr) = pending_kfs_[rep[rr].first].lm_desc.row(rep[rr].second);

    // Phase 3 — per-KF observations against the merged landmarks (re-walk raw order).
    int raw_idx = 0;
    for (std::size_t k = 0; k < pending_kfs_.size(); ++k) {
      const auto& r = pending_kfs_[k];
      const int m = (int)r.lm_pcam.size();
      auto& ids = sm.kf_obs[k].landmark_ids;
      std::vector<std::array<float, 2>> uvs;
      std::unordered_set<int> rows_seen;
      for (int i = 0; i < m; ++i, ++raw_idx) {
        const int mrow = raw_row[raw_idx];
        if (mrow < 0 || !rows_seen.insert(mrow).second) continue;
        ids.push_back(sm.landmarks[(std::size_t)mrow].id);
        uvs.push_back({r.lm_uv(i, 0), r.lm_uv(i, 1)});
      }
      sm.kf_obs[k].uv.resize((int)uvs.size(), 2);
      for (int j = 0; j < (int)uvs.size(); ++j) {
        sm.kf_obs[k].uv(j, 0) = uvs[j][0];
        sm.kf_obs[k].uv(j, 1) = uvs[j][1];
      }
    }
    const int raw_n = (int)raw_p.size();
    const std::string path = map_dir_.empty()
        ? std::string()
        : map_dir_ + "/submap_" + std::to_string(sm.id) + ".smap";
    if (!path.empty()) {
      if (!slamko::saveSubMap(sm, path)) {
        RCLCPP_ERROR(get_logger(), "saveSubMap failed: %s", path.c_str());
      } else {
        sealed_ids_.push_back(sm.id);
        std::ofstream mf(map_dir_ + "/submaps.manifest");
        for (auto sid : sealed_ids_) mf << sid << "\n";
      }
    }
    RCLCPP_INFO(get_logger(), "sealed submap %llu (%zu KF, %d->%zu lm %.1fx dedup, VPR)%s",
                (unsigned long long)sm.id, sm.keyframes.size(), raw_n, sm.landmarks.size(),
                sm.landmarks.empty() ? 1.0 : (double)raw_n / (double)sm.landmarks.size(),
                kf_no_image_ ? (" [" + std::to_string(kf_no_image_) + " KF w/o image]").c_str()
                             : "");
    // Defer relocalizer registration until the submap is older than the loop gap.
    submap_first_kf_[sm.id] = sm.keyframes.front().id;
    submap_last_t_[sm.id] = pending_kfs_.back().t;
    sealed_unregistered_.emplace_back(std::move(sm), pending_kfs_.back().t);
    dumpAnchorEdges();
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
  std::FILE* global_file_ = nullptr;
  std::string graph_path_;
  std::unordered_map<std::uint64_t, double> node_time_;

  // P-B: image buffer + VPR + sealing (KfRec declared above the methods)
  std::deque<std::pair<double, cv::Mat>> img_buf_, img_buf_r_;
  EigenPlacesPtr vpr_;
  std::shared_ptr<XFeat> xfeat_;
  std::unique_ptr<slamko::XFeatRelocalizer> reloc_;        // session submaps (in-session loops)
  std::unique_ptr<slamko::XFeatRelocalizer> reloc_prior_;  // prior map (cross-session re-anchor)
  std::vector<KfRec> pending_kfs_;
  std::vector<std::uint64_t> sealed_ids_;
  std::deque<std::pair<slamko::SubMap, double>> sealed_unregistered_;
  std::unordered_map<std::uint64_t, std::uint64_t> submap_first_kf_;
  std::unordered_map<std::uint64_t, double> submap_last_t_;
  std::uint64_t next_submap_id_ = 0, next_landmark_id_ = 0;
  std::string map_dir_;
  int kf_per_submap_ = 50;
  double stale_thresh_ = 0.5, force_loss_start_ = -1.0, force_loss_end_ = -1.0;
  double cov_soft_thresh_ = 0.01;  // OKVIS pos-cov trace -> degraded (Marginal/Lost)
  double t0_ = -1.0, last_odom_t_ = -1.0;
  bool loss_in_segment_ = false;   // a stale-gap occurred -> next chain edge is SOFT
  int kf_no_image_ = 0, loops_closed_ = 0;
  double image_tol_s_ = 0.06, image_buffer_s_ = 0.6;

  // P-B 2b: calibration (canonical CROPPED camera model) + reloc gates
  bool reloc_enabled_ = false, have_calib_ = false, have_info_l_ = false, have_info_r_ = false;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0;
  unsigned img_w_ = 0;
  int crop_x_ = 0;
  slamko::SE3 body_T_cam_;
  double min_loop_gap_s_ = 25.0, loop_sigma_t_ = 0.10, loop_sigma_r_ = 0.05;
  double max_loop_disagree_m_ = 30.0;
  int pcm_consec_ = 3;
  int reloc_min_inliers_ = 25, max_kf_landmarks_ = 200;
  double lm_dedup_voxel_ = 0.06;   // voxel [m] for landmark dedup at seal (task #9)
  int lm_min_obs_ = 2;             // min observations to survive culling (kills rays)
  // Inter-map anchor-graph edges (R1.1/R1.3) — the federation-of-islands connections.
  struct AnchorEdge {
    std::uint64_t from, to;
    slamko::SE3 T_from_to;
    double sigma_t, sigma_r;
    int type;  // 0=chain-odom, 1=chain-soft (DR across a loss), 2=hard (verified weld)
  };
  std::vector<AnchorEdge> anchor_edges_;
  slamko::SE3 prev_anchor_;
  std::uint64_t prev_anchor_id_ = 0;
  bool have_prev_anchor_ = false;
  int seg_kf_no_image_ = 0;
  double anchor_chain_sigma_t_ = 0.05, anchor_chain_sigma_r_ = 0.02;
  double anchor_soft_sigma_t_ = 1.0, anchor_soft_sigma_r_ = 0.3;
  int anchor_soft_lm_ = 7000;      // segment raw-landmark floor below which the chain edge is SOFT
  int prior_min_inliers_ = 15;
  double min_reloc_period_s_ = 0.5, last_reloc_attempt_t_ = -1e18;
  std::string lg_model_path_;
  slamko::LoopConsensusGate gate_;

  // Cross-session prior map (anchor-don't-weld re-anchoring)
  std::vector<slamko::SubMap> prior_submaps_;  // emptied after registration
  std::unordered_set<std::uint64_t> prior_ids_;
  std::unordered_map<std::uint64_t, slamko::SE3> prior_anchor_;
  slamko::SE3 T_global_map_;
  bool localized_ = false;
  // correction-consensus for big re-anchor jumps (cross-floor alias defense)
  double reanchor_jump_m_ = 0.5;
  slamko::SE3 T_big_cand_;
  bool have_big_cand_ = false;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_, sub_image_r_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_l_, sub_info_r_;
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
