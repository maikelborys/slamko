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
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
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
#include "slamko_loop/mappoint_store.hpp"
#include "slamko_loop/pose_graph.hpp"
#include "slamko_loop/xfeat_relocalizer.hpp"
#include "slamko_ros/viz_sink.hpp"
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

    // ----- R0.1 instrument: an INDEPENDENT dead-reckoning channel that GATES the
    // across-gap motion OKVIS reports. OKVIS internally bridges a tracking loss with
    // its own IMU (measured 2026-06-19: never resets, holds warm state) — so the
    // soft chain edge is already an IMU-bridged pose, NOT identity. The value of a
    // SECOND, independent DR is to GATE that bridge: if OKVIS's across-gap relative
    // motion disagrees with our gyro-integrated rotation + last-velocity coasting,
    // OKVIS's bridge is suspect (the R0 "never ingest garbage" gate). This pass only
    // MEASURES the disagreement (-> dr_gate.csv); the empirical distribution sets the
    // gate threshold (R0.1 -> R0). GYRO-ONLY on purpose: bno_ab camera-IMU accel is
    // DOUBLED on these bags, so we never integrate accel — coasting uses the OKVIS
    // body twist (its last trustworthy velocity), not raw accel.
    imu_topic_ = declare_parameter("imu_topic", std::string("/camera/camera/imu"));
    dr_gate_path_ = declare_parameter("dr_gate_path", std::string(""));
    if (!imu_topic_.empty()) {
      auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(200)).durability_volatile();
      if (declare_parameter("imu_best_effort", true)) imu_qos.best_effort();
      else imu_qos.reliable();
      sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
          imu_topic_, imu_qos,
          std::bind(&ProviderFusionNode::onImu, this, std::placeholders::_1));
      if (!dr_gate_path_.empty()) {
        dr_gate_file_ = std::fopen(dr_gate_path_.c_str(), "w");
        if (dr_gate_file_)
          std::fprintf(dr_gate_file_,
                       "rel_t,gap_s,d_rot_deg,d_trans_m,okvis_trans_m,coast_trans_m\n");
      }
      RCLCPP_INFO(get_logger(), "DR gate instrument on: imu=%s csv=%s",
                  imu_topic_.c_str(), dr_gate_path_.c_str());
    }

    // ----- COMPASS instrument (task #3, instrument-first like the DR gate). Raw magnetometer
    // (NOT the BNO fused orientation — it silently re-snaps ~180°), heading + FIELD-NORM GATE:
    // a reading is trusted ONLY when |B| sits in Earth's band (~25-65 uT). Indoors (rebar,
    // motors, electronics) the field is disturbed -> |B| out of band -> REJECTED, which is the
    // correct behaviour (the plan: gate the compass OFF where it's unreliable). This pass
    // measures availability + logs the gated heading; a yaw factor wiring is the next step.
    mag_topic_ = declare_parameter("mag_topic", std::string("/bno055/mag"));
    compass_csv_path_ = declare_parameter("compass_csv_path", std::string(""));
    mag_norm_min_ut_ = declare_parameter("mag_norm_min_ut", 25.0);
    mag_norm_max_ut_ = declare_parameter("mag_norm_max_ut", 65.0);
    if (!mag_topic_.empty()) {
      auto mag_qos = rclcpp::QoS(rclcpp::KeepLast(50)).durability_volatile();
      if (declare_parameter("mag_best_effort", true)) mag_qos.best_effort();
      else mag_qos.reliable();
      sub_mag_ = create_subscription<sensor_msgs::msg::MagneticField>(
          mag_topic_, mag_qos,
          std::bind(&ProviderFusionNode::onMag, this, std::placeholders::_1));
      if (!compass_csv_path_.empty()) {
        compass_file_ = std::fopen(compass_csv_path_.c_str(), "w");
        if (compass_file_)
          std::fprintf(compass_file_, "t,norm_ut,heading_deg,gated_ok\n");
      }
      RCLCPP_INFO(get_logger(), "compass instrument on: mag=%s gate=[%.0f,%.0f]uT csv=%s",
                  mag_topic_.c_str(), mag_norm_min_ut_, mag_norm_max_ut_,
                  compass_csv_path_.c_str());
    }

    // ----- BNO055 ABSOLUTE-YAW compass PRIOR (the bent-corridor / vision-loss yaw fix;
    // ported from RTABmap/COMPASS_RTABMAP.md). OFF by default — opt-in, never regresses an
    // existing run. Uses the BNO's fused orientation quaternion as a yaw-only graph prior,
    // gated by mag-cal + field-norm + innovation with adaptive sigma. The |B| field-norm
    // comes from onMag, so the mag topic must be subscribed (it is, above).
    compass_yaw_prior_ = declare_parameter("compass_yaw_prior", false);
    yaw_min_cal_mag_   = declare_parameter("yaw_min_cal_mag", 2);
    yaw_band_tol_ut_   = declare_parameter("yaw_band_tol_ut", 8.0);
    yaw_sigma_min_deg_ = declare_parameter("yaw_sigma_min_deg", 5.0);
    yaw_sigma_max_deg_ = declare_parameter("yaw_sigma_max_deg", 45.0);
    yaw_innov_gate_deg_ = declare_parameter("yaw_innov_gate_deg", 30.0);
    yaw_init_window_s_ = declare_parameter("yaw_init_window_s", 4.0);
    yaw_opt_every_     = declare_parameter("yaw_opt_every", 15);
    bno_imu_topic_     = declare_parameter("bno_imu_topic", std::string("/bno055/imu"));
    bno_calib_topic_   = declare_parameter("bno_calib_topic", std::string("/bno055/calib_status"));
    if (compass_yaw_prior_) {
      auto bq = rclcpp::QoS(rclcpp::KeepLast(200)).durability_volatile();
      if (declare_parameter("bno_best_effort", true)) bq.best_effort(); else bq.reliable();
      sub_bno_imu_ = create_subscription<sensor_msgs::msg::Imu>(
          bno_imu_topic_, bq,
          std::bind(&ProviderFusionNode::onBnoImu, this, std::placeholders::_1));
      sub_bno_calib_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
          bno_calib_topic_, bq,
          std::bind(&ProviderFusionNode::onBnoCalib, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(),
                  "compass YAW-PRIOR on: bno=%s calib=%s min_cal=%d band=±%.0fuT "
                  "sigma=[%.0f,%.0f]deg innov_gate=%.0fdeg opt_every=%d",
                  bno_imu_topic_.c_str(), bno_calib_topic_.c_str(), yaw_min_cal_mag_,
                  yaw_band_tol_ut_, yaw_sigma_min_deg_, yaw_sigma_max_deg_,
                  yaw_innov_gate_deg_, yaw_opt_every_);
    }

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
      // GAP-2 immortality (task #4 first brick): "don't re-map what you already see".
      // When the current segment is already EXPLAINED by an existing submap — i.e. we
      // stayed confidently re-anchored/loop-matched (>= dup_min_inliers) over >= dup_cover_frac
      // of its keyframes — sealing another submap just duplicates the map (10x replay
      // = 10x landmarks of the SAME room, even though reloc KNOWS we are there). So we
      // SUPPRESS the duplicate seal. A KF counts as "covered" if it arrives within
      // dup_cover_window_s of a confident match. dup_suppress=false restores old behavior.
      dup_suppress_ = declare_parameter("dup_suppress", true);
      dup_min_inliers_ = declare_parameter("dup_min_inliers", 60);
      dup_cover_window_s_ = declare_parameter("dup_cover_window_s", 4.0);
      dup_cover_frac_ = declare_parameter("dup_cover_frac", 0.6);
      // GAP-2 maturation (task #4 proper, V1): when a revisit segment is SUPPRESSED as a
      // duplicate of a PRIOR submap, don't throw its geometry away — REFINE the prior
      // submap's existing landmarks toward the multi-session consensus (cross-session
      // averaging, structure-only). The prior map MATURES each visit without growing.
      // V1 refines existing positions only (safe: descriptors/kf_obs untouched); adding
      // new descriptored landmarks to fix blind-spot recall is V2. Matured archive is
      // written to mature_out_dir (never clobbers the input prior unless pointed there).
      mature_enabled_ = declare_parameter("mature_enabled", true);
      mature_out_dir_ = declare_parameter("mature_out_dir", std::string(""));
      mature_voxel_ = declare_parameter("mature_voxel_m", 0.06);
      mature_alpha_ = declare_parameter("mature_alpha", 0.2);
      // GAP-2 CULL BACKSTOP (the immortality ceiling guarantee): a sealed submap whose
      // landmarks are >= cull_redundant_frac already in the global occupancy (existing
      // submaps) is geometrically redundant -> culled. Bounds the map by AREA regardless
      // of VPR recall (suppression alone leaks ~3.5 submaps/visit -> linear growth).
      cull_enabled_ = declare_parameter("cull_enabled", true);
      occ_refresh_ = declare_parameter("occ_refresh", true);  // P2: fuse revisits via loop-corrected occ
      cull_voxel_ = declare_parameter("cull_voxel_m", 0.15);  // drift-tolerant (only the redundancy
      // test coarsens; the stored-map dedup stays at lm_dedup_voxel_m=0.06).
      cull_redundant_frac_ = declare_parameter("cull_redundant_frac", 0.7);
      cull_min_lms_ = declare_parameter("cull_min_lms", 100);
      // Viewpoint-aware cull (recall fix): keep geometrically-redundant revisits that bring a NEW
      // viewing direction (low VPR coverage) as omni-directional reloc anchors; cull only same-view.
      cull_viewpoint_aware_ = declare_parameter("cull_viewpoint_aware", true);
      cull_vp_frac_ = declare_parameter("cull_vp_frac", 0.5);
      // Phase A (PLAN_PERSISTENT_MAPPOINTS_02): drift-TOLERANT cross-submap data
      // association by DESCRIPTOR — the persistent-MapPoint identity slamko lacked.
      // The voxel occ_ cull misses revisit duplicates when VIO drift > the voxel; this
      // matches the same physical point by XFeat cosine within a generous radius and
      // culls it regardless of drift. Opt-in (default OFF = today's behaviour exactly).
      mappoint_assoc_ = declare_parameter("mappoint_assoc", false);
      mp_store_ = slamko::MapPointStore(
          declare_parameter("mappoint_assoc_cell", 0.4),
          declare_parameter("mappoint_assoc_radius", 0.4),
          (float)declare_parameter("mappoint_assoc_cos", 0.82));
      // Phase B — MULTI-VIEW REFINE (requires mappoint_assoc). On a revisit re-observation
      // fold the new sighting into the MapPoint's running-mean position+descriptor (consensus)
      // and, at shutdown, back-propagate the consensus into the persisted submap landmarks +
      // dump mappoints.csv (id,x,y,z,n_obs = visit confidence). The revisit TIGHTENS the map.
      // Trajectory-neutral by construction (loose graph is poses-only; this only moves stored
      // landmark geometry, never a graph factor). Opt-in (default OFF = today's behaviour).
      mappoint_refine_ = declare_parameter("mappoint_refine", false);
      // Inter-map anchor edges (R1.1 hard / R1.3 soft): chain edge sigma between
      // consecutive submaps (good odometry) vs SOFT sigma when the segment crossed
      // a visual loss (dead-reckoning — approximate placement only). Hard = loop sigma.
      anchor_chain_sigma_t_ = declare_parameter("anchor_chain_sigma_t", 0.05);
      anchor_chain_sigma_r_ = declare_parameter("anchor_chain_sigma_r", 0.02);
      anchor_soft_sigma_t_ = declare_parameter("anchor_soft_sigma_t", 1.0);
      anchor_soft_sigma_r_ = declare_parameter("anchor_soft_sigma_r", 0.3);
      // TRUNK step 1 (#12, METHODOLOGY_01): when a stale-gap loss is detected, the
      // provider's covariance on the gap-spanning CHAIN edge is over-confident (OKVIS
      // IMU-bridged it but the geometry is dead-reckoned). Replace it with the DR-gate
      // disagreement (d_trans, d_rot) as the real across-gap uncertainty -> the optimizer
      // YIELDS there -> a later loop pulls the warped segment straight (the brutal-on-suave
      // warp fix). Opt-in (default OFF = today's stiff behaviour) for a clean A/B.
      dr_gate_soft_cov_ = declare_parameter("dr_gate_soft_cov", false);
      dr_gate_soft_floor_t_ = declare_parameter("dr_gate_soft_floor_t", 0.05);
      dr_gate_soft_floor_r_ = declare_parameter("dr_gate_soft_floor_r", 0.02);
      // ATLAS disjoint-islands (etapa 1b, METHODOLOGY_01): a tracking loss BREAKS the
      // chain into a NEW map component instead of soft-bridging it. The island floats
      // (per-component gauge, etapa 1a) until a feature match welds it back; no match =>
      // it honestly stays dangling. Supersedes the #12 soft-bridge when ON. Opt-in.
      atlas_break_on_loss_ = declare_parameter("atlas_break_on_loss", false);
      // Coalesce: don't spawn a NEW map until the active one has grown to a real size.
      // A burst of clustered losses (or a loss right at the end) would otherwise spawn
      // useless 10-landmark sliver maps; require the active component to be mature first.
      atlas_min_component_kfs_ = declare_parameter("atlas_min_component_kfs", 20);
      anchor_soft_lm_ = declare_parameter("anchor_soft_lm", 7000);
      // R0.2 ingestion gate: bar degraded-tracking submaps from being reloc match targets.
      gate_degraded_reloc_ = declare_parameter("gate_degraded_reloc", true);
      // DR-gate reject threshold: a stale-gap whose gyro-vs-OKVIS rotation disagreement exceeds
      // this (deg) means OKVIS's IMU-bridge is untrustworthy -> bar the submap as a reloc target.
      // R0.1 measured d_rot 1.3-4.2 deg for sound bridges up to 6 s, so 15 deg has wide margin.
      dr_gate_reject_deg_ = declare_parameter("dr_gate_reject_deg", 15.0);
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
      // Cross-session DRIFT correction (research RESEARCH_LIFELONG_FUSION_01): after
      // the first rigid re-anchor establishes the session->global frame, each further
      // confident prior match adds a UNARY PRIOR FACTOR on its keyframe (pulling it
      // toward the prior-implied pose) and re-optimizes — so the graph BENDS the
      // session onto the prior instead of a rigid re-base that leaves a doubled copy.
      xsession_prior_factor_ = declare_parameter("xsession_prior_factor", true);
      xsession_prior_sigma_t_ = declare_parameter("xsession_prior_sigma_t", 0.15);
      xsession_prior_sigma_r_ = declare_parameter("xsession_prior_sigma_r", 0.08);
      // Jumps below this apply as a prior factor (legit drift correction); bigger jumps
      // fall through to the 2-vote rigid-rebase alias defense (cross-FLOOR = 2.6-8 m).
      xsession_prior_jump_max_ = declare_parameter("xsession_prior_jump_max", 2.0);
      // E (proximity detection): radius [m] around the estimated global pose within which
      // prior submaps are geometrically verified independent of VPR retrieval. 0 = off.
      proximity_radius_ = declare_parameter("proximity_radius", 3.0);
      // 3-TIER candidate->soft->weld (RESEARCH_LIFELONG_FUSION_01 §arch; Kimera/maplab
      // precedent). A proximity match (E) is VPR-independent = weaker appearance evidence,
      // so it enters a CANDIDATE tier first (viz-only, NOT optimised) and is promoted to a
      // prior factor ONLY on a strong-inlier hit OR a 2nd consistent candidate. This
      // protects never-false-merge (Hard Rule: never ingest garbage) without losing E's
      // recall. false -> proximity matches promote immediately (the pre-3-tier behaviour).
      proximity_three_tier_ = declare_parameter("proximity_three_tier", true);
      // Inliers at/above which a single proximity candidate promotes straight to a factor.
      proximity_promote_inliers_ = declare_parameter("proximity_promote_inliers", 40);
      // Max translation [m] between two candidate corrections for them to "agree" (2nd vote).
      proximity_agree_m_ = declare_parameter("proximity_agree_m", 0.5);
      // Consistent votes to the SAME prior submap needed to promote a weak (inl<strong)
      // proximity match -> a weld. Accumulated per-submap (robust to interleaved candidates).
      proximity_votes_needed_ = declare_parameter("proximity_votes_needed", 2);
      // Looser tolerance for ACCUMULATING votes than for a single strong match: weak
      // cross-recording matches (inl 15-34) have noisy PnP (~0.5-1 m), so consecutive
      // votes to the SAME place scatter > proximity_agree_m. The submap IDENTITY repeating
      // is the real signal; this just rejects gross outliers. Precision stays (a false
      // place won't repeat to the same submap N times within this bound).
      proximity_vote_agree_m_ = declare_parameter("proximity_vote_agree_m", 2.0);
      min_reloc_period_s_ = declare_parameter("min_reloc_period_s", 0.5);
      lg_model_path_ = declare_parameter(
          "lightglue_model_path",
          onnx_default.empty() ? std::string()
                               : onnx_default.substr(0, onnx_default.rfind('/')) +
                                     "/lighterglue.pt");
      const auto prior_dir = declare_parameter("prior_map_dir", std::string(""));
      prior_dir_ = prior_dir;  // kept for shutdown maturation (re-load + refine + re-save)
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

    // ----- LIVE VIZ (Rerun). No-op unless the build is -DSLAMKO_WITH_RERUN AND a
    // `rerun` viewer is reachable. connect_grpc to a SEPARATELY-launched viewer (never
    // spawn() in-process — keeps the viewer's GPU/crash handling out of the estimator).
    viz_enable_ = declare_parameter("viz", false);
    viz_endpoint_ = declare_parameter("viz_endpoint", std::string(""));
    if (viz_enable_) {
      const bool on = viz_.init("slamko", viz_endpoint_);
      viz_.setSessionTransform(T_global_map_, /*localized=*/false);  // identity at boot
      RCLCPP_INFO(get_logger(), "live viz: %s (%s)",
                  on ? "STREAMING to rerun" : "requested but NOT connected "
                       "(build -DSLAMKO_WITH_RERUN + launch a `rerun` viewer)",
                  viz_endpoint_.empty() ? "default endpoint" : viz_endpoint_.c_str());
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
      int refreshed = 0, refined = 0;
      for (auto sid : sealed_ids_) {
        const std::string path = map_dir_ + "/submap_" + std::to_string(sid) + ".smap";
        slamko::SubMap sm;
        if (!slamko::loadSubMap(sm, path)) continue;
        const auto it = submap_first_kf_.find(sid);
        if (it == submap_first_kf_.end() || !graph_.hasNode(it->second)) continue;
        sm.anchor = graph_.pose(it->second);
        // Phase B — back-propagate the multi-view CONSENSUS into the persisted landmarks:
        // rewrite each from its refined MapPoint (global -> this submap's local via the
        // just-refreshed anchor). The revisit doesn't just dedup, it TIGHTENS the map.
        if (mappoint_refine_) {
          const slamko::SE3 to_local = (T_global_map_ * sm.anchor).inverse();
          for (auto& lm : sm.landmarks) {
            const Eigen::Vector3d* gp = mp_store_.position(lm.id);
            if (gp) { lm.position = to_local * (*gp); ++refined; }
          }
        }
        if (slamko::saveSubMap(sm, path)) ++refreshed;
      }
      RCLCPP_INFO(get_logger(), "shutdown: refreshed %d/%zu sealed anchors from the graph",
                  refreshed, sealed_ids_.size());
      // Phase B — dump the consensus MapPoint cloud with per-point visit confidence (n_obs).
      if (mappoint_refine_) {
        std::ofstream mpf(map_dir_ + "/mappoints.csv");
        mpf << "id,x,y,z,n_obs\n";
        for (const auto& m : mp_store_.points())
          mpf << m.id << "," << m.pos.x() << "," << m.pos.y() << "," << m.pos.z() << ","
              << m.n_obs << "\n";
        RCLCPP_INFO(get_logger(),
                    "Phase B: back-propagated consensus into %d landmarks; wrote %zu refined "
                    "MapPoints (mappoints.csv, n_obs = visit confidence)",
                    refined, mp_store_.size());
      }
    }
    // ETAPA 2: re-tag each submap by its FINAL connected component (after the automatic
    // feature-match welds), so the Atlas shows FUSED fragments as one map and a fragment
    // that never matched as dangling — not the frozen break-time tag. The merge already
    // happens automatically (a feature-match weld + the per-component gauge); this makes
    // it visible/measurable.
    if (!map_dir_.empty() && !sealed_ids_.empty()) {
      const auto roots = graph_.connectedComponents();
      std::unordered_map<std::uint64_t, int> root_to_comp;  // graph root -> small display id
      std::ofstream cf(map_dir_ + "/components.csv");
      cf << "submap_id,component\n";
      for (auto sid : sealed_ids_) {
        int comp = -1;
        const auto it = submap_first_kf_.find(sid);
        if (it != submap_first_kf_.end()) {
          const auto r = roots.find(it->second);
          if (r != roots.end())
            comp = root_to_comp.emplace(r->second, (int)root_to_comp.size()).first->second;
        }
        if (comp < 0) comp = submap_component_.count(sid) ? submap_component_[sid] : 0;
        cf << sid << "," << comp << "\n";
      }
      RCLCPP_INFO(get_logger(),
                  "ETAPA 2: %d break-component(s) -> %zu FUSED map(s) after feature-match "
                  "welds (components.csv re-tagged by final graph connectivity)",
                  component_id_ + 1, root_to_comp.size());
    }
    if (fused_file_) std::fclose(fused_file_);
    if (suppressed_dups_ > 0)
      RCLCPP_INFO(get_logger(),
                  "GAP-2 immortality: suppressed %d duplicate submap(s) of known ground "
                  "(map did not grow on revisited territory)", suppressed_dups_);
    if (culled_submaps_ > 0)
      RCLCPP_INFO(get_logger(),
                  "GAP-2 cull backstop: culled %d redundant submap(s) — map bounded by AREA, "
                  "not by visits", culled_submaps_);
    if (gated_reloc_targets_ > 0)
      RCLCPP_INFO(get_logger(),
                  "R0 seal-quality gate: barred %d degraded submap(s) from being reloc targets "
                  "(never ingest garbage as a match source)", gated_reloc_targets_);
    finalizeMaturation();
    if (provider_file_) std::fclose(provider_file_);
    if (global_file_) std::fclose(global_file_);
    if (dr_gate_file_) std::fclose(dr_gate_file_);
    if (mag_total_ > 0)
      RCLCPP_INFO(get_logger(),
                  "compass instrument: %ld/%ld readings passed the field-norm gate (%.0f%%) "
                  "— low %% indoors = field disturbed, compass correctly gated OFF",
                  mag_accepted_, mag_total_, 100.0 * (double)mag_accepted_ / (double)mag_total_);
    if (compass_file_) std::fclose(compass_file_);
  }

 private:
  // COMPASS instrument: field-norm-gated raw-magnetometer heading. |B| in Tesla -> uT; a
  // reading is trusted only if |B| sits in Earth's band. Heading from the horizontal
  // components (sensor frame; absolute alignment needs extrinsics + WMM declination later).
  void onMag(const sensor_msgs::msg::MagneticField::SharedPtr m) {
    const double t = rclcpp::Time(m->header.stamp).seconds();
    const double bx = m->magnetic_field.x, by = m->magnetic_field.y, bz = m->magnetic_field.z;
    const double norm_ut = std::sqrt(bx * bx + by * by + bz * bz) * 1e6;  // T -> uT
    bno_last_norm_ut_ = norm_ut;  // latest |B| for the compass yaw-prior field-norm gate
    const bool ok = norm_ut >= mag_norm_min_ut_ && norm_ut <= mag_norm_max_ut_;
    const double heading_deg = std::atan2(by, bx) * 180.0 / M_PI;
    ++mag_total_;
    if (ok) ++mag_accepted_;
    if (compass_file_) {
      std::fprintf(compass_file_, "%.3f,%.2f,%.2f,%d\n", t, norm_ut, heading_deg, ok ? 1 : 0);
      std::fflush(compass_file_);
    }
  }

  // ---- BNO055 ABSOLUTE-YAW compass prior (COMPASS_RTABMAP.md ported into slamko).
  // The BNO's fused orientation quaternion gives an absolute heading (magnetometer-
  // anchored). We use it as a YAW-ONLY prior in the pose graph — the one DOF VIO can't
  // observe — gated by mag calibration + field-norm + innovation, with adaptive sigma.
  // yaw about world-Z from a quaternion (pitch-safe except at ±90° pitch).
  static double yawFromQuat(const Eigen::Quaterniond& q) {
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }
  static double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  void onBnoImu(const sensor_msgs::msg::Imu::SharedPtr m) {
    if (!compass_yaw_prior_) return;
    const double t = rclcpp::Time(m->header.stamp).seconds();
    const Eigen::Quaterniond q(m->orientation.w, m->orientation.x, m->orientation.y,
                               m->orientation.z);
    if (q.norm() < 0.5) return;  // unpopulated orientation
    bno_yaw_buf_.emplace_back(t, yawFromQuat(q.normalized()));
    while (bno_yaw_buf_.size() > 600) bno_yaw_buf_.pop_front();
  }

  void onBnoCalib(const std_msgs::msg::UInt8MultiArray::SharedPtr m) {
    if (m->data.size() >= 4) bno_mag_cal_ = m->data[3];  // [sys,gyro,accel,MAG] 0-3
  }

  // BNO yaw nearest to time t (within 0.1 s), false if none.
  bool bnoYawAt(double t, double& yaw_out) const {
    double best_dt = 0.1;
    bool found = false;
    for (const auto& e : bno_yaw_buf_)
      if (std::abs(e.first - t) < best_dt) { best_dt = std::abs(e.first - t); yaw_out = e.second; found = true; }
    return found;
  }

  // Per-keyframe: maybe add a gated, adaptive-sigma yaw prior anchoring the OKVIS heading
  // to the BNO's absolute yaw. Calibrates the OKVIS-world↔magnetic-north offset over an
  // init window first. Re-optimizes every yaw_opt_every_ applied priors so the heading is
  // continuously straightened (the bent-corridor fix).
  void maybeAddYawPrior(std::uint64_t id, double t, const slamko::SE3& T_map) {
    if (!compass_yaw_prior_ || !graph_.hasNode(id)) return;
    double yaw_bno;
    if (!bnoYawAt(t, yaw_bno)) return;
    // Gate 1: magnetometer calibrated enough.
    if (bno_mag_cal_ < yaw_min_cal_mag_) { ++yaw_gated_cal_; return; }
    const double yaw_okvis = yawFromQuat(T_map.so3().unit_quaternion());

    if (!yaw_offset_locked_) {
      // INIT: learn the constant OKVIS-yaw↔BNO-yaw offset + nominal |B| over a window.
      if (yaw_init_t0_ < 0.0) yaw_init_t0_ = t;
      yaw_off_sin_ += std::sin(yaw_okvis - yaw_bno);
      yaw_off_cos_ += std::cos(yaw_okvis - yaw_bno);
      if (bno_last_norm_ut_ > 0.0) { yaw_nom_sum_ += bno_last_norm_ut_; ++yaw_nom_n_; }
      if (t - yaw_init_t0_ >= yaw_init_window_s_ && yaw_nom_n_ >= 10) {
        yaw_offset_ = std::atan2(yaw_off_sin_, yaw_off_cos_);
        yaw_nominal_ut_ = yaw_nom_sum_ / std::max(1, yaw_nom_n_);
        yaw_offset_locked_ = true;
        RCLCPP_INFO(get_logger(),
                    "compass yaw-prior LOCKED: offset=%.1fdeg nominal|B|=%.1fuT (cal=%d)",
                    yaw_offset_ * 180.0 / M_PI, yaw_nominal_ut_, bno_mag_cal_);
      }
      return;
    }

    // Gate 2: field-norm band (auto-learned nominal ± tol) — rejects ferrous spikes.
    const double anom = (yaw_nominal_ut_ > 0.0 && bno_last_norm_ut_ > 0.0)
                            ? std::abs(bno_last_norm_ut_ - yaw_nominal_ut_) / yaw_band_tol_ut_
                            : 1.0;
    if (anom > 1.0) { ++yaw_gated_band_; return; }
    // Gate 3: innovation (compass vs current VIO yaw) — anti-spike backstop.
    const double yaw_target = wrapPi(yaw_bno + yaw_offset_);
    const double innov = wrapPi(yaw_target - yaw_okvis);
    if (std::abs(innov) > yaw_innov_gate_deg_ * M_PI / 180.0) { ++yaw_gated_innov_; return; }

    // Adaptive sigma: tight in a clean field, loose where dirty (|B| anomaly / low cal /
    // big innovation). dirtiness in [0,1] lerps sigma_min -> sigma_max (geometric).
    const double dirt = std::min(1.0, std::max({anom,
        (3.0 - bno_mag_cal_) / 3.0,
        std::abs(innov) / (yaw_innov_gate_deg_ * M_PI / 180.0)}));
    const double smin = yaw_sigma_min_deg_ * M_PI / 180.0;
    const double smax = yaw_sigma_max_deg_ * M_PI / 180.0;
    const double sigma = smin * std::pow(smax / smin, dirt);
    graph_.addYawPrior(id, yaw_target, sigma, /*robust=*/true);
    ++yaw_priors_added_;
    if (yaw_priors_added_ % yaw_opt_every_ == 0 && graph_.numNodes() >= 5) {
      const auto res = graph_.optimize();
      T_map_odom_target_ = graph_.pose(id) * chain_.lastKeyframe().T_OB.inverse();
      RCLCPP_INFO(get_logger(),
                  "compass yaw-prior #%d @kf %llu: yaw_bno=%.0f yaw_vio=%.0f innov=%.1fdeg "
                  "sigma=%.0fdeg | re-opt cost %.2e->%.2e (gated cal/band/innov=%ld/%ld/%ld)",
                  yaw_priors_added_, (unsigned long long)id, yaw_target * 180.0 / M_PI,
                  yaw_okvis * 180.0 / M_PI, innov * 180.0 / M_PI, sigma * 180.0 / M_PI,
                  res.initial_cost, res.final_cost, yaw_gated_cal_, yaw_gated_band_,
                  yaw_gated_innov_);
    }
  }

  // Independent gyro-only orientation integration (world<-body). Drifts slowly
  // (gyro bias), but across a SHORT loss gap (<~3 s) the integrated rotation is
  // accurate — that is exactly the window we gate. Accel is NOT touched (doubled
  // on bno_ab); translation comes from coasting the OKVIS body twist.
  void onImu(const sensor_msgs::msg::Imu::SharedPtr m) {
    const double t = rclcpp::Time(m->header.stamp).seconds();
    if (last_imu_t_ < 0.0) { last_imu_t_ = t; return; }
    const double dt = t - last_imu_t_;
    last_imu_t_ = t;
    if (dt <= 0.0 || dt > 0.2) return;  // drop reorders / large holes (don't integrate junk)
    const Eigen::Vector3d w(m->angular_velocity.x, m->angular_velocity.y,
                            m->angular_velocity.z);
    dr_R_ = dr_R_ * slamko::SO3::exp(w * dt);
  }

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
      const double gap = s.t - last_odom_t_;
      // R0.1 GATE measurement: how far does OKVIS's across-gap motion (its own IMU
      // bridge) sit from an INDEPENDENT estimate? Rotation from our gyro integration;
      // translation from coasting the last trustworthy OKVIS body velocity. Big
      // disagreement => OKVIS's bridge is suspect => this segment is a gate candidate.
      const slamko::SE3 T_rel_okvis = last_odom_T_.inverse() * s.T_OB;
      const slamko::SO3 R_rel_dr = dr_R_at_last_odom_.inverse() * dr_R_;
      const Eigen::Vector3d t_coast = last_odom_v_ * gap;  // first-order coast, before-body frame
      const double d_rot_deg =
          (T_rel_okvis.so3().inverse() * R_rel_dr).log().norm() * 180.0 / M_PI;
      const double d_trans = (T_rel_okvis.translation() - t_coast).norm();
      RCLCPP_WARN(get_logger(),
                  "TRACKING LOSS: odom stale-gap %.2fs @t=%.1f -> seal+branch (soft edge) "
                  "| DR gate: d_rot=%.1fdeg d_trans=%.2fm (okvis=%.2fm coast=%.2fm)",
                  gap, rel, d_rot_deg, d_trans, T_rel_okvis.translation().norm(),
                  t_coast.norm());
      if (dr_gate_file_) {
        std::fprintf(dr_gate_file_, "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n", rel, gap,
                     d_rot_deg, d_trans, T_rel_okvis.translation().norm(), t_coast.norm());
        std::fflush(dr_gate_file_);
      }
      if (!map_dir_.empty() && pending_kfs_.size() >= 2) sealSubmap();
      loss_in_segment_ = true;
      // R0 DR-GATE as a real gate (not just instrument): bar the post-gap submap as a reloc
      // target ONLY if the independent gyro DR disagrees with OKVIS's across-gap motion beyond
      // dr_gate_reject_deg — i.e. OKVIS's IMU-bridge is suspect. A small d_rot means OKVIS
      // bridged the gap correctly (proven sound <=6 s), so its geometry is trustworthy -> keep
      // it as a valid reloc target. This turns the R0.1 measurement into the R0 decision.
      if (d_rot_deg > dr_gate_reject_deg_) seg_hard_loss_ = true;
      // #12: stage the DR-gate uncertainty for the NEXT chain edge (the one spanning
      // this gap). The disagreement IS the across-gap covariance — flooring keeps it
      // from going tighter than a normal odom edge.
      if (dr_gate_soft_cov_) {
        pending_soft_sigma_t_ = std::max(dr_gate_soft_floor_t_, d_trans);
        pending_soft_sigma_r_ = std::max(dr_gate_soft_floor_r_, d_rot_deg * M_PI / 180.0);
      }
      // ETAPA 1b: tracking was lost -> the post-gap keyframe starts a NEW disjoint map,
      // BUT only once the active map is mature (else clustered/end losses spawn slivers).
      if (atlas_break_on_loss_ && kfs_in_component_ >= atlas_min_component_kfs_)
        pending_break_ = true;
    }
    last_odom_t_ = s.t;
    // snapshot the last trustworthy state for the NEXT gap's DR comparison.
    last_odom_T_ = s.T_OB;
    last_odom_v_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                                   msg->twist.twist.linear.z);
    dr_R_at_last_odom_ = dr_R_;
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
      if (pending_break_) {
        // ETAPA 1b — ATLAS BREAK: tracking was lost, so do NOT bridge the dead-reckoned
        // gap. This keyframe starts a NEW disjoint map component that floats (per-component
        // gauge, etapa 1a) until a feature match welds it back. No chain edge added.
        ++component_id_;
        kfs_in_component_ = 0;   // the new map must mature before it can break again
        pending_break_ = false;
        pending_soft_sigma_t_ = pending_soft_sigma_r_ = 0.0;  // break supersedes #12 bridge
        RCLCPP_WARN(get_logger(),
                    "ATLAS BREAK: kf %llu starts new map component %d (loss -> disjoint island)",
                    (unsigned long long)edge->to, component_id_);
      } else if (pending_soft_sigma_t_ > 0.0) {
        Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
        const double it = 1.0 / (pending_soft_sigma_t_ * pending_soft_sigma_t_);
        const double ir = 1.0 / (pending_soft_sigma_r_ * pending_soft_sigma_r_);
        info.diagonal() << it, it, it, ir, ir, ir;
        graph_.addEdge(edge->from, edge->to, edge->T_from_to, info, false);
        RCLCPP_INFO(get_logger(),
                    "#12 DR-gate soft chain edge %llu->%llu: sigma_t=%.2fm sigma_r=%.1fdeg "
                    "(replaced provider-stiff cov)",
                    (unsigned long long)edge->from, (unsigned long long)edge->to,
                    pending_soft_sigma_t_, pending_soft_sigma_r_ * 180.0 / M_PI);
        pending_soft_sigma_t_ = pending_soft_sigma_r_ = 0.0;
      } else {
        graph_.addEdge(edge->from, edge->to, edge->T_from_to, edge->information, false);
      }
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
        for (const auto& sm : prior_submaps_) {
          reloc_prior_->addSubMap(sm);
          // Seed the cull-backstop occupancy: prior submaps are already in real-world
          // (prior-global) frame, so the session's new submaps that re-cover this ground
          // are recognized as redundant and culled from visit 1.
          for (const auto& lm : sm.landmarks)
            prior_occ_.insert(occKey(sm.anchor * lm.position));
          // LIVE VIZ: the prior map cloud (grey, world/prior) — the backdrop the session
          // aligns onto once cross-session-localized.
          if (viz_enable_ && viz_.enabled()) {
            std::vector<Eigen::Vector3d> pts;
            pts.reserve(sm.landmarks.size());
            for (const auto& lm : sm.landmarks) pts.push_back(sm.anchor * lm.position);
            viz_.logPriorCloud(sm.id, pts);
          }
        }
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
    ++kfs_in_component_;  // maturity of the active atlas component (gates break, etapa 1b)
    maybeAddYawPrior(id, t, T_map);  // compass yaw anchor (independent of the VPR path)
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
        // ---- LIVE VIZ window A (landmarks over video) + window B pose/camera/HUD.
        if (viz_enable_ && viz_.enabled()) {
          viz_.setTime(id, t);
          viz_.logImage(*left);
          // XFeat keypoints -> FULL-image pixels (wide sensors are centre-cropped by crop_x_).
          const float off = (left->cols >= 752) ? (float)crop_x_ : 0.0f;
          Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> uv(ql.keypoints.rows(), 2);
          for (int i = 0; i < ql.keypoints.rows(); ++i) {
            uv(i, 0) = ql.keypoints(i, 0) + off;
            uv(i, 1) = ql.keypoints(i, 1);
          }
          viz_.logKeypoints(uv, loss_in_segment_ ? slamko::VizTrack::Degraded
                                                  : slamko::VizTrack::Tracked);
          const bool certain = localized_ && viz_kf_since_match_ < 18;
          viz_.logPose(T_map, certain);
          viz_.logCamera(T_map * body_T_cam_, fx_, fy_, cx_, cy_, left->cols, left->rows);
          char hud[256];
          std::snprintf(hud, sizeof(hud),
                        "%s | kf %llu | submaps %zu | loops %d | x-prior %d | prox %d/%d | "
                        "%zu kpts%s",
                        localized_ ? "LOCALIZED" : (loss_in_segment_ ? "TRACK-LOSS" : "MAPPING"),
                        (unsigned long long)id, sealed_ids_.size(), loops_closed_,
                        xsession_priors_added_, prox_promoted_, prox_candidates_,
                        (size_t)ql.keypoints.rows(), loss_in_segment_ ? " [DR]" : "");
          viz_.logHud(hud);
          viz_.logScalar("loops", loops_closed_);
          viz_.logScalar("xsession_priors", xsession_priors_added_);
          viz_.logScalar("keypoints", ql.keypoints.rows());
          ++viz_kf_since_match_;
        }
      }
    }

    pending_kfs_.push_back(std::move(rec));
    // GAP-2 coverage tally: this KF is "covered" if a confident match to an existing
    // submap is still fresh (within dup_cover_window_s of the last one).
    ++seg_total_kfs_;
    if (covered_until_t_ >= 0.0 && t <= covered_until_t_) ++seg_covered_kfs_;
    registerAgedSubmaps(t);
    if (!map_dir_.empty() && (int)pending_kfs_.size() >= kf_per_submap_) {
      // "Don't re-map what you already see": if this whole segment is explained by an
      // existing submap (sustained confident re-anchor), don't persist a duplicate.
      const bool covered = dup_suppress_ && have_prev_anchor_ && seg_total_kfs_ > 0 &&
                           seg_covered_kfs_ >= dup_cover_frac_ * seg_total_kfs_;
      if (covered) {
        RCLCPP_INFO(get_logger(),
                    "SUPPRESSED duplicate submap: %d/%d KFs already covered by submap "
                    "%llu (immortal: map NOT grown, %d dups suppressed so far)",
                    seg_covered_kfs_, seg_total_kfs_,
                    (unsigned long long)covered_by_submap_, suppressed_dups_ + 1);
        ++suppressed_dups_;
        // GAP-2 maturation: instead of discarding this duplicate's geometry, buffer its
        // landmarks (in the PRIOR's global frame) keyed by the prior submap that covers
        // us — at shutdown they REFINE that prior submap's existing landmarks. Only for
        // PRIOR submaps (in-session maturation is V2). T_global_map_ maps session->prior.
        if (mature_enabled_ && prior_ids_.count(covered_by_submap_)) {
          auto& buf = mature_buf_[covered_by_submap_];
          for (const auto& r : pending_kfs_) {
            const slamko::SE3 T_g_cam = T_global_map_ * graph_.pose(r.id) * body_T_cam_;
            for (const auto& p : r.lm_pcam) buf.push_back(T_g_cam * p);
          }
        }
        pending_kfs_.clear();
        seg_total_kfs_ = seg_covered_kfs_ = 0;
      } else {
        sealSubmap();
      }
    }
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
    if (reloc_prior_) {
      processRelocResult(reloc_prior_->relocalize(query), q_id, t);
      // E — PROXIMITY DETECTION: once roughly localized, ALSO verify against prior submaps
      // NEAR our current estimated global pose (VPR-independent). This recovers the
      // recall-dead zone (jolts/motion-blur/opposite-heading) the cosine retrieval misses,
      // filling the cross-session match gap so the prior factors (A) can correct the drift
      // there. Where nothing matches, the region honestly stays dangling.
      if (localized_ && proximity_radius_ > 0.0 && graph_.hasNode(q_id)) {
        const slamko::SE3 T_q_global = T_global_map_ * graph_.pose(q_id);
        processRelocResult(
            reloc_prior_->relocalizeNear(query, T_q_global, proximity_radius_), q_id, t,
            /*from_proximity=*/true);
      }
    }
  }

  void processRelocResult(const slamko::RelocResult& r, std::uint64_t q_id, double t,
                          bool from_proximity = false) {
    if (r.found)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "reloc attempt: kf %llu best=submap %llu inliers=%d",
                           (unsigned long long)q_id, (unsigned long long)r.submap_id,
                           r.num_inliers);
    const bool is_prior = r.found && prior_ids_.count(r.submap_id) > 0;
    if (!r.found ||
        r.num_inliers < (is_prior ? prior_min_inliers_ : reloc_min_inliers_))
      return;

    // 3-TIER candidate->soft->weld gate for the PROXIMITY path (RESEARCH_LIFELONG_
    // FUSION_01 §arch). A proximity match is VPR-INDEPENDENT = weaker appearance
    // evidence, so it must not perturb the graph on a single hit (never-false-merge).
    // It enters a CANDIDATE tier (viz dashed-grey, NOT optimised) and is promoted to a
    // prior factor ONLY on strong inliers OR a 2nd candidate agreeing on the same
    // correction. Strong/agreeing -> fall through to the normal prior-factor path below.
    if (from_proximity && proximity_three_tier_ && is_prior && localized_ &&
        graph_.hasNode(q_id)) {
      const slamko::SE3 T_global_q = prior_anchor_.at(r.submap_id) * r.T_query_match;
      const slamko::SE3 target_session = T_global_map_.inverse() * T_global_q;
      // The implied session->global CORRECTION (where this whole map sits in the world).
      // THIS is the invariant consistent matches agree on — NOT the per-keyframe target
      // pose, which moves WITH the robot (the bug: votes from different moving keyframes
      // never agreed -> stuck at vote 1/2). Two matches that place the map at the same
      // world spot agree, regardless of which keyframe they came from.
      const slamko::SE3 T_gm_implied = T_global_q * graph_.pose(q_id).inverse();
      ++prox_candidates_;
      if (viz_enable_ && viz_.enabled()) {
        const Eigen::Vector3d pq = graph_.pose(q_id).translation();
        const Eigen::Vector3d pp =
            (T_global_map_.inverse() * prior_anchor_.at(r.submap_id)).translation();
        viz_cand_segs_.push_back({pp, pq});
        pushGraphEdgesToViz();
      }
      const bool strong = r.num_inliers >= proximity_promote_inliers_;
      // ROBUST 2nd-vote: accumulate votes PER prior submap (not a single overwriteable
      // slot — intervening candidates to OTHER submaps used to wipe it, so the 3 votes to
      // submap 1 on EuRoC never counted). A vote counts only if its implied target AGREES
      // with the running target (within proximity_agree_m) -> random false matches don't
      // agree, so they never accumulate (the precision defense survives). Promote when a
      // submap reaches proximity_votes_needed consistent votes: the cross-recording matches
      // EXIST at inl 15-36, they just need to be COUNTED instead of dropped below inl 40.
      auto& v = prox_votes_[r.submap_id];
      const bool consistent =
          v.count > 0 &&
          (v.corr.translation() - T_gm_implied.translation()).norm() < proximity_vote_agree_m_;
      if (consistent) {
        // running-mean the correction so small per-match noise keeps accumulating.
        const Eigen::Vector3d mean =
            (v.corr.translation() * v.count + T_gm_implied.translation()) / (v.count + 1);
        v.corr = slamko::SE3(T_gm_implied.so3(), mean);
        ++v.count;
      } else {
        v.count = 1;
        v.corr = T_gm_implied;
      }
      if (!strong && v.count < proximity_votes_needed_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "proximity CANDIDATE held: kf %llu -> prior submap %llu inl=%d "
                             "(vote %d/%d, or strong>=%d)",
                             (unsigned long long)q_id, (unsigned long long)r.submap_id,
                             r.num_inliers, v.count, proximity_votes_needed_,
                             proximity_promote_inliers_);
        return;
      }
      prox_votes_.erase(r.submap_id);  // promoted -> clear this submap's accumulated votes
      viz_cand_segs_.clear();  // promoted -> the candidate becomes a real (green) prior edge
      ++prox_promoted_;
    }
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
    // IN-SESSION loops only: cross-session matches legitimately jump between ADJACENT
    // prior submaps (the prior map is tiled), so a same-submap streak is the wrong
    // gate for them — they starve to 0-1 factors. Cross-session is guarded instead by
    // the inlier threshold (above) + the jump consensus + the robust Huber on the prior
    // factor. (Strict never-false-merge vs a DIFFERENT place still needs the other-place bag.)
    if (!is_prior) {
      const bool accept = gate_.feed(slamko::LoopCandidate{
          r.submap_id, q_id, t, r.T_query_match, chain_.lastKeyframe().T_OB});
      if (!accept) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "loop candidate: kf %llu -> submap %llu inliers=%d streak=%d/%d",
                             (unsigned long long)q_id, (unsigned long long)r.submap_id,
                             r.num_inliers, gate_.streak(r.submap_id), pcm_consec_);
        return;
      }
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
        refreshOcc();  // P2: T_global_map_ changed -> reproject session occupancy into global
        // Atlas coherence: pin the whole session subtree onto the prior frame so window B
        // shows the session sitting INSIDE the prior cloud (and re-anchors move it as one).
        viz_.setSessionTransform(T_global_map_, true);
        viz_kf_since_match_ = 0;
      } else if (xsession_prior_factor_) {
        // A (RESEARCH_LIFELONG_FUSION_01): T_global_map_ stays FROZEN after the first
        // re-anchor; each further confident match adds a per-keyframe PRIOR pulling q
        // toward the prior-implied pose, then re-optimizes -> the graph BENDS the session
        // onto the prior. (A rigid re-base has 6 DOF and cannot absorb path-length-growing
        // internal drift -> the doubled/offset copy.) Big jumps (cross-floor alias, 2.6-8 m)
        // still fall through to the 2-vote rigid alias defense.
        const slamko::SE3 target_session = T_global_map_.inverse() * T_global_q;
        const double jump =
            (graph_.pose(q_id).inverse() * target_session).translation().norm();
        if (jump < xsession_prior_jump_max_) {
          graph_.addPriorFactor(q_id, target_session, xsession_prior_sigma_t_,
                                xsession_prior_sigma_r_, /*robust=*/true);
          const auto res = graph_.optimize();
          T_map_odom_target_ = graph_.pose(q_id) * chain_.lastKeyframe().T_OB.inverse();
          refreshOcc();  // P2: prior factor corrected the session -> realign occupancy
          ++xsession_priors_added_;
          have_big_cand_ = false;
          RCLCPP_INFO(get_logger(),
              "X-SESSION prior #%d: kf %llu -> prior submap %llu inl=%d jump=%.2fm "
              "cost %.2e->%.2e",
              xsession_priors_added_, (unsigned long long)q_id,
              (unsigned long long)r.submap_id, r.num_inliers, jump,
              res.initial_cost, res.final_cost);
          if (viz_enable_ && viz_.enabled()) {
            const Eigen::Vector3d pq = graph_.pose(q_id).translation();
            const Eigen::Vector3d pp =
                (T_global_map_.inverse() * prior_anchor_.at(r.submap_id)).translation();
            viz_prior_segs_.push_back({pp, pq});
            viz_kf_since_match_ = 0;
            pushGraphEdgesToViz();
          }
          markCoverage(node_time_.count(q_id) ? node_time_[q_id] : 0.0, r.submap_id,
                       r.num_inliers);
          return;
        }
        if (have_big_cand_ &&
            (T_big_cand_.inverse() * T_new).translation().norm() < reanchor_jump_m_) {
          T_global_map_ = T_new; have_big_cand_ = false;
          tag = "BIG CORRECTION applied (2-vote)";
        } else {
          T_big_cand_ = T_new; have_big_cand_ = true;
          RCLCPP_WARN(get_logger(),
              "x-session JUMP held (%.2f m, needs 2nd vote): kf %llu -> prior submap %llu inl=%d",
              jump, (unsigned long long)q_id, (unsigned long long)r.submap_id, r.num_inliers);
          return;
        }
      } else {
        // legacy rigid re-anchor (xsession_prior_factor disabled)
        const double jump = (T_global_map_.inverse() * T_new).translation().norm();
        if (jump < reanchor_jump_m_) {
          T_global_map_ = T_new; have_big_cand_ = false; tag = "RE-ANCHORED";
        } else if (have_big_cand_ &&
                   (T_big_cand_.inverse() * T_new).translation().norm() < reanchor_jump_m_) {
          T_global_map_ = T_new; have_big_cand_ = false;
          tag = "BIG CORRECTION applied (2-vote)";
        } else {
          T_big_cand_ = T_new; have_big_cand_ = true;
          RCLCPP_WARN(get_logger(),
                      "re-anchor JUMP held (%.2f m, needs a 2nd agreeing vote): "
                      "kf %llu -> prior submap %llu inliers=%d",
                      jump, (unsigned long long)q_id,
                      (unsigned long long)r.submap_id, r.num_inliers);
          return;
        }
      }
      const auto& gt = T_global_map_.translation();
      RCLCPP_INFO(get_logger(),
                  "%s in prior map: kf %llu -> prior submap %llu inliers=%d  "
                  "T_global_map t=[%.3f %.3f %.3f]",
                  tag, (unsigned long long)q_id, (unsigned long long)r.submap_id,
                  r.num_inliers, gt.x(), gt.y(), gt.z());
      markCoverage(node_time_.count(q_id) ? node_time_[q_id] : 0.0, r.submap_id,
                   r.num_inliers);
      return;
    }

    const std::uint64_t a = submap_first_kf_.at(r.submap_id);
    // Submap-local frame == its first KF's body frame (sealing convention), so
    // T_query_match IS the a->query relative measurement.
    graph_.addLoopEdge(a, q_id, r.T_query_match, loop_sigma_t_, loop_sigma_r_);
    const auto res = graph_.optimize();
    T_map_odom_target_ = graph_.pose(q_id) * chain_.lastKeyframe().T_OB.inverse();
    refreshOcc();  // P2: the loop removed drift -> realign occupancy -> revisits now fuse
    RCLCPP_INFO(get_logger(),
                "LOOP CLOSED: kf %llu -> submap %llu (kf %llu), inliers=%d assoc=%zu | optimize: "
                "converged=%d iters=%d cost %.2e -> %.2e",
                (unsigned long long)q_id, (unsigned long long)r.submap_id,
                (unsigned long long)a, r.num_inliers, r.matches.size(), (int)res.converged,
                res.iterations, res.initial_cost, res.final_cost);
    ++loops_closed_;
    // P1 proof artifact: dump the re-associated landmark ids (the SAME points re-observed
    // on this revisit) so scripts/render_map_png.py can HIGHLIGHT them. q_id,submap_id,lm_id.
    if (!map_dir_.empty() && !r.matches.empty()) {
      std::ofstream af(map_dir_ + "/loop_assoc.csv", std::ios::app);
      for (const auto& m : r.matches)
        af << q_id << ',' << r.submap_id << ',' << m.second << '\n';
    }
    markCoverage(node_time_.count(q_id) ? node_time_[q_id] : 0.0, r.submap_id,
                 r.num_inliers);
    // R1.1 HARD inter-map edge: the building submap (its future id == next_submap_id_)
    // is verified-connected to target submap r.submap_id. Recorded for the anchor
    // graph + the multi-map viz (reversible: it is just an entry we can drop).
    anchor_edges_.push_back(AnchorEdge{next_submap_id_, r.submap_id, r.T_query_match,
                                       loop_sigma_t_, loop_sigma_r_, 2});
    dumpAnchorEdges();
    // LIVE VIZ: the "sees another keyframe" link q<->matched (red) + redraw the graph.
    if (viz_enable_ && viz_.enabled()) {
      viz_loop_segs_.push_back({graph_.pose(a).translation(), graph_.pose(q_id).translation()});
      viz_kf_since_match_ = 0;
      pushGraphEdgesToViz();
    }
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

  // World position (session frame) of a sealed submap's anchor = its first KF's live
  // graph pose. False if not yet in the graph (e.g. the still-building submap).
  bool anchorPos(std::uint64_t sm, Eigen::Vector3d& out) {
    auto it = submap_first_kf_.find(sm);
    if (it == submap_first_kf_.end() || !graph_.hasNode(it->second)) return false;
    out = graph_.pose(it->second).translation();
    return true;
  }

  // Re-log the typed pose-graph edges to the viz (replace-per-class). Chain/Soft come
  // from anchor_edges_ (submap-anchor endpoints); Loop/Prior/Candidate are the
  // accumulated kf<->target links. The trust ladder is drawn by COLOUR + a per-class
  // entity layer (research: never a z-offset) so soft/candidate links read as uncertain
  // without contaminating the trusted map.
  void pushGraphEdgesToViz() {
    if (!viz_enable_ || !viz_.enabled()) return;
    std::vector<std::array<Eigen::Vector3d, 2>> chain, soft, loop;
    Eigen::Vector3d a, b;
    for (const auto& e : anchor_edges_) {
      if (!anchorPos(e.from, a) || !anchorPos(e.to, b)) continue;
      if (e.type == 0) chain.push_back({a, b});
      else if (e.type == 1) soft.push_back({a, b});
      else loop.push_back({a, b});
    }
    for (const auto& s : viz_loop_segs_) loop.push_back(s);
    viz_.setEdges(slamko::VizEdge::Chain, chain);
    viz_.setEdges(slamko::VizEdge::Soft, soft);
    viz_.setEdges(slamko::VizEdge::Loop, loop);
    viz_.setEdges(slamko::VizEdge::Prior, viz_prior_segs_);
    viz_.setEdges(slamko::VizEdge::Candidate, viz_cand_segs_);
  }

  // Register sealed submaps into the relocalizer only once they are OLDER than
  // the loop gap — relocalize() then never sees "places" it just left.
  void registerAgedSubmaps(double now_t) {
    while (!sealed_unregistered_.empty() &&
           now_t - sealed_unregistered_.front().second > min_loop_gap_s_) {
      const auto& sm = sealed_unregistered_.front().first;
      // R0.2 SEAL-QUALITY GATE: a submap sealed during degraded tracking has untrustworthy
      // geometry -> keep it in the map (chain/viz) but DON'T register it as a reloc target,
      // so a future query can never false-match against garbage and corrupt the graph (I2).
      if (reloc_ && !degraded_submaps_.count(sm.id)) {
        reloc_->addSubMap(sm);
      } else if (degraded_submaps_.count(sm.id)) {
        ++gated_reloc_targets_;
        RCLCPP_INFO(get_logger(),
                    "R0 gate: submap %llu sealed degraded -> NOT a reloc target "
                    "(no garbage match source; %d gated so far)",
                    (unsigned long long)sm.id, gated_reloc_targets_);
      }
      sealed_unregistered_.pop_front();
    }
  }

  // GAP-2: a confident match to an EXISTING submap proves we are on known ground.
  // Extend the "covered" window so the keyframes around here count as duplicates of
  // submap `sm` (used to suppress a redundant seal). Only confident matches count.
  void markCoverage(double t, std::uint64_t sm, int inliers) {
    if (inliers < dup_min_inliers_) return;
    covered_until_t_ = t + dup_cover_window_s_;
    covered_by_submap_ = sm;
  }

  // Voxel key of a real-world point for the cull-backstop occupancy set (coarser than the
  // dedup voxel — we test "is this space already mapped", robust to small pose differences).
  std::int64_t occKey(const Eigen::Vector3d& p) const {
    const double v = cull_voxel_;
    auto q = [v](double x) { return (std::int64_t)std::llround(std::floor(x / v)); };
    const std::int64_t a = q(p.x()) & 0x1FFFFF, b = q(p.y()) & 0x1FFFFF, c = q(p.z()) & 0x1FFFFF;
    return (a << 42) | (b << 21) | c;
  }

  // P2 FUSION: rebuild the session occupancy from the loop-CORRECTED submap anchors. A loop
  // closure removes the accumulated drift from the graph poses; refreshing occ_ to the
  // corrected positions means a subsequent revisit (now aligned) lands in the SAME voxels as
  // the original → its duplicate landmarks are culled → the map FUSES instead of doubling.
  // Prior occupancy (prior_occ_, global + fixed) is untouched. occ_refresh=false = old behaviour.
  void refreshOcc() {
    if (!cull_enabled_ || !occ_refresh_ || kept_lm_local_.empty()) return;
    occ_.clear();
    for (const auto& kv : kept_lm_local_) {
      auto it = submap_first_kf_.find(kv.first);
      if (it == submap_first_kf_.end() || !graph_.hasNode(it->second)) continue;
      const slamko::SE3 anchor = graph_.pose(it->second);
      for (const auto& lm : kv.second) occ_.insert(occKey(T_global_map_ * anchor * lm));
    }
  }

  // GAP-2 maturation (shutdown): reload the prior archive and REFINE each covered prior
  // submap's existing landmarks toward this session's revisit observations (cross-session
  // averaging, structure-only — positions nudged by mature_alpha toward the multi-view
  // consensus; descriptors/kf_obs untouched, map size UNCHANGED). The lifelong map
  // improves where it was revisited, without growing. Matured archive -> mature_out_dir.
  void finalizeMaturation() {
    if (!mature_enabled_ || mature_buf_.empty() || prior_dir_.empty()) return;
    std::vector<slamko::SubMap> pm;
    if (!slamko::loadSubMaps(pm, prior_dir_)) {
      RCLCPP_WARN(get_logger(), "maturation: could not reload prior from %s",
                  prior_dir_.c_str());
      return;
    }
    const double vox = mature_voxel_;
    auto vkey = [vox](const Eigen::Vector3d& p) -> std::int64_t {
      auto q = [vox](double x) { return (std::int64_t)std::llround(std::floor(x / vox)); };
      const std::int64_t a = q(p.x()) & 0x1FFFFF, b = q(p.y()) & 0x1FFFFF,
                         c = q(p.z()) & 0x1FFFFF;
      return (a << 42) | (b << 21) | c;
    };
    int total_refined = 0, matured = 0;
    for (auto& X : pm) {
      auto it = mature_buf_.find(X.id);
      if (it == mature_buf_.end() || X.landmarks.empty()) continue;
      std::unordered_map<std::int64_t, std::vector<int>> cells;
      for (int i = 0; i < (int)X.landmarks.size(); ++i)
        cells[vkey(X.landmarks[i].position)].push_back(i);
      const slamko::SE3 Xinv = X.anchor.inverse();
      int refined = 0;
      for (const auto& g : it->second) {
        const Eigen::Vector3d pl = Xinv * g;  // prior-global -> X-local
        auto ci = cells.find(vkey(pl));
        if (ci == cells.end()) continue;
        int best = -1;
        double bd = vox * vox;
        for (int idx : ci->second) {
          const double d = (X.landmarks[idx].position - pl).squaredNorm();
          if (d < bd) { bd = d; best = idx; }
        }
        if (best < 0) continue;
        X.landmarks[best].position += mature_alpha_ * (pl - X.landmarks[best].position);
        ++refined;
      }
      if (refined > 0) { total_refined += refined; ++matured; }
    }
    const std::string out =
        mature_out_dir_.empty() ? (prior_dir_ + "_matured") : mature_out_dir_;
    std::filesystem::create_directories(out);
    if (slamko::saveSubMaps(pm, out))
      RCLCPP_INFO(get_logger(),
                  "GAP-2 maturation: refined %d landmarks across %d prior submap(s) "
                  "(map size UNCHANGED) -> %s",
                  total_refined, matured, out.c_str());
  }

  void sealSubmap() {
    slamko::SubMap sm;
    sm.id = next_submap_id_++;
    // CULL backstop rollback snapshot — restored verbatim if this submap turns out to be
    // geometrically redundant (see the cull check after the landmarks are built).
    const slamko::SE3 cull_saved_prev_anchor = prev_anchor_;
    const std::uint64_t cull_saved_prev_id = prev_anchor_id_;
    const bool cull_saved_have_prev = have_prev_anchor_;
    const bool cull_saved_loss = loss_in_segment_;
    const int cull_saved_seg_kf_no_img = seg_kf_no_image_;
    const std::size_t cull_saved_edges = anchor_edges_.size();
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
    // DEGRADED-segment signal (R0 ingestion gate): the segment crossed a tracking loss,
    // yielded too few landmarks, or missed images -> its odom is dead-reckoned and its
    // landmark geometry is untrustworthy. Used both for the SOFT chain edge AND (R0.2) to
    // BAR this submap from becoming a relocalization target — a garbage match source would
    // false-anchor the session and propagate corruption through the graph (the I2 violation).
    // Reloc-target bar fires ONLY on a TRUE stale-gap loss (dead-reckoned poses = wrong
    // geometry) — NOT on the covariance-marginal trigger (still-usable geometry, valid
    // reloc target). Conflating them barred clean-marginal submaps and killed loop closure.
    const bool seg_was_hard_loss = seg_hard_loss_;   // capture before the reset below
    if (gate_degraded_reloc_ && seg_hard_loss_) degraded_submaps_.insert(sm.id);
    seg_hard_loss_ = false;
    if (have_prev_anchor_) {
      // B (RESEARCH_LIFELONG_FUSION_01): the CHAIN edge is SOFT (high cov, the optimizer
      // yields here) ONLY on a TRUE odometry stale-gap — when the placement is genuinely
      // dead-reckoned. NOT on missing-VPR-images or low-landmark-count: OKVIS's relative
      // pose is still accurate there, so the chain geometry stays STIFF. (Using `degraded`
      // mislabeled every segment with a dropped image as soft -> the whole chain went loose
      // = 0 odom / all-soft edges, and the map couldn't hold OKVIS's metric shape.) The
      // reloc-TARGET bar above still uses the broader degraded signal — that's correct.
      const bool soft = seg_was_hard_loss;
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
    int row = 0, lm_culled_occ = 0;
    // Phase A: descriptor-duplicates removed from the STORED map this seal. Kept SEPARATE
    // from lm_culled_occ so the CULL BACKSTOP + occ_ stay byte-identical to baseline ->
    // Phase A dedups the map WITHOUT moving the graph/trajectory (provably no-regress).
    int culled_mp_this = 0;
    std::vector<std::int64_t> mp_culled_ock;   // occ_ keys to insert IFF this submap is kept
    // VIEWPOINT-AWARE cull (the recall fix): only cull a geometrically-redundant revisit if its
    // VIEWPOINT is already in the map — i.e. its keyframes matched known places by VPR
    // (seg_covered_kfs_ high = same viewing direction). A revisit from a NEW direction (e.g.
    // opposite-facing: same 3D points, but low VPR cosine -> NOT covered) is KEPT, so its
    // descriptors become a reloc anchor for that direction. Recall failures are a viewpoint-
    // COVERAGE gap (measured: same-heading revisits already match at floor 0.426; opposite-facing
    // can't be matched by any descriptor/dense-matcher). Distinct viewing directions per place are
    // finite -> the map stays bounded while becoming omni-directional. cull_viewpoint_aware=false
    // restores pure-geometric culling.
    const bool viewpoint_known = !cull_viewpoint_aware_ ||
        (seg_total_kfs_ > 0 && seg_covered_kfs_ >= cull_vp_frac_ * seg_total_kfs_);
    const bool occ_cull = cull_enabled_ && (!occ_.empty() || !prior_occ_.empty()) && viewpoint_known;
    for (auto& kv : cells) {
      auto& idxs = kv.second;
      if ((int)idxs.size() < lm_min_obs_) continue;     // CULL: seen < lm_min_obs_ times
      Eigen::Vector3d c = Eigen::Vector3d::Zero();
      for (int ri : idxs) c += raw_p[ri];
      c /= (double)idxs.size();                          // centroid = multi-view refine
      // ORB-SLAM data association ACROSS submaps (the immortality bound): if this physical
      // point already lives in the global occupancy (an existing submap mapped it), it's a
      // duplicate of known ground -> CULL it, keep only genuinely-NEW points. Per-LANDMARK
      // (not per-submap) so a PARTIAL revisit keeps its new sliver + drops the redundant bulk
      // -> the map grows only with new content -> bounded by AREA, not by visits.
      const Eigen::Vector3d gpos = T_global_map_ * sm.anchor * c;
      const std::int64_t ock = occKey(gpos);
      if (occ_cull && (occ_.count(ock) || prior_occ_.count(ock))) { ++lm_culled_occ; continue; }
      // Phase A — DRIFT-TOLERANT data association by descriptor (the voxel cull above only
      // fires when the revisit lands in the SAME voxel; under drift it doesn't). If this
      // point's XFeat descriptor matches an existing MapPoint within the search radius, it
      // is the SAME physical point the revisit re-observed -> cull the duplicate (record the
      // observation), regardless of the few-cm drift that fooled the voxel test.
      const auto& ki0 = raw_ki[idxs[0]];
      if (mappoint_assoc_) {
        const auto drow = pending_kfs_[ki0.first].lm_desc.row(ki0.second);
        const std::int64_t mp = mp_store_.associate(gpos, drow);
        if (mp >= 0) {
          // Drift-tolerant duplicate: drop from the stored map, but treat it EXACTLY as a
          // kept point for the backstop denominator + occ_ (so the graph is unaffected).
          // Phase B: fold this re-observation into the MapPoint consensus (multi-view refine);
          // otherwise just count it (Phase A pure dedup).
          if (mappoint_refine_) mp_store_.refine(mp, gpos, drow);
          else mp_store_.addObservation(mp);
          ++culled_mp_; ++culled_mp_this;
          mp_culled_ock.push_back(ock);
          continue;
        }
      }
      sm.landmarks.push_back(slamko::MapLandmark{next_landmark_id_, c, row});
      if (mappoint_assoc_)
        mp_store_.add(next_landmark_id_, gpos, pending_kfs_[ki0.first].lm_desc.row(ki0.second));
      ++next_landmark_id_;
      rep.push_back(ki0);
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

    // ---- CULL BACKSTOP (immortality ceiling, ORB-SLAM KeyFrameCulling at submap level).
    // If most of this submap's landmarks fall in real-world voxels ALREADY occupied by
    // existing submaps, it duplicates known ground -> CULL (roll the seal back, persist
    // nothing). Unlike dup-suppression (appearance/VPR-gated -> misses blind spots, leaks
    // ~3.5 submaps/visit -> linear growth), this is GEOMETRIC: it bounds the map by AREA
    // regardless of recall, so the map plateaus instead of inflating over revisits.
    // + culled_mp_this so the denominator equals baseline's (those points WOULD be in
    // sm.landmarks without Phase A) -> the backstop fires identically -> trajectory-neutral.
    const int dedup_total = lm_culled_occ + (int)sm.landmarks.size() + culled_mp_this;
    if (cull_enabled_ && dedup_total >= cull_min_lms_ &&
        lm_culled_occ >= cull_redundant_frac_ * dedup_total) {
      // >= cull_redundant_frac of the deduped landmarks were already mapped -> this segment
      // is a (near-)pure revisit -> drop the whole submap (the per-landmark cull above already
      // kept the few new points; below cull_min_lms_ of them isn't worth its own submap).
      RCLCPP_INFO(get_logger(),
                  "CULLED redundant submap (%d/%d lm already mapped, only %zu new) — immortal: "
                  "map bounded by AREA, %d culled so far",
                  lm_culled_occ, dedup_total, sm.landmarks.size(), culled_submaps_ + 1);
      ++culled_submaps_;
      next_submap_id_ = sm.id;                        // give the id back
      prev_anchor_ = cull_saved_prev_anchor;          // undo every seal mutation
      prev_anchor_id_ = cull_saved_prev_id;
      have_prev_anchor_ = cull_saved_have_prev;
      loss_in_segment_ = cull_saved_loss;
      seg_kf_no_image_ = cull_saved_seg_kf_no_img;
      anchor_edges_.resize(cull_saved_edges);
      pending_kfs_.clear();
      seg_total_kfs_ = seg_covered_kfs_ = 0;
      return;
    }
    if (lm_culled_occ > 0)
      RCLCPP_INFO(get_logger(),
                  "data-association: kept %zu NEW lm, culled %d already-mapped (partial revisit)",
                  sm.landmarks.size(), lm_culled_occ);
    // KEEP: this submap's genuinely-new landmarks -> mark their voxels occupied so future
    // revisits of this ground are recognized as duplicates and culled per-landmark.
    for (const auto& lm : sm.landmarks)
      occ_.insert(occKey(T_global_map_ * sm.anchor * lm.position));
    // Phase A: the descriptor-duplicates dropped from the stored map STILL mark their
    // voxels occupied (baseline would have kept them) -> occ_ identical to baseline.
    for (std::int64_t k : mp_culled_ock) occ_.insert(k);
    // P2: cache this KEPT submap's local landmarks so occ_ can be refreshed from the
    // loop-corrected anchor (graph_.pose(first_kf)) after a loop removes drift.
    {
      auto& cache = kept_lm_local_[sm.id];
      cache.reserve(sm.landmarks.size());
      for (const auto& lm : sm.landmarks) cache.push_back(lm.position);
    }

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
        // ETAPA 1b/1c: tag this submap with its atlas component (which disjoint island it
        // belongs to) + dump a sidecar so the renderer can colour fragments separately.
        submap_component_[sm.id] = component_id_;
        std::ofstream cf(map_dir_ + "/components.csv");
        cf << "submap_id,component\n";
        for (auto sid : sealed_ids_) cf << sid << "," << submap_component_[sid] << "\n";
      }
    }
    RCLCPP_INFO(get_logger(), "sealed submap %llu (%zu KF, %d->%zu lm %.1fx dedup, VPR)%s",
                (unsigned long long)sm.id, sm.keyframes.size(), raw_n, sm.landmarks.size(),
                sm.landmarks.empty() ? 1.0 : (double)raw_n / (double)sm.landmarks.size(),
                kf_no_image_ ? (" [" + std::to_string(kf_no_image_) + " KF w/o image]").c_str()
                             : "");
    if (mappoint_assoc_)
      RCLCPP_INFO(get_logger(),
                  "MapPoint store: %zu points, %d drift-tolerant duplicates culled (cumulative)",
                  mp_store_.size(), culled_mp_);
    // Defer relocalizer registration until the submap is older than the loop gap.
    submap_first_kf_[sm.id] = sm.keyframes.front().id;
    submap_last_t_[sm.id] = pending_kfs_.back().t;
    // LIVE VIZ window B: this submap's landmark cloud (session frame) + the typed edges.
    // A submap sealed across a true loss / during degraded tracking renders DANGLING
    // (distinct hue) — honest about which geometry is trustworthy.
    if (viz_enable_ && viz_.enabled()) {
      viz_.setTime(sm.keyframes.back().id, pending_kfs_.back().t);
      std::vector<Eigen::Vector3d> pts;
      pts.reserve(sm.landmarks.size());
      for (const auto& lm : sm.landmarks) pts.push_back(sm.anchor * lm.position);
      viz_.logSubmapCloud(sm.id, pts, seg_was_hard_loss || degraded_submaps_.count(sm.id) > 0);
      pushGraphEdgesToViz();
    }
    sealed_unregistered_.emplace_back(std::move(sm), pending_kfs_.back().t);
    dumpAnchorEdges();
    pending_kfs_.clear();
    seg_total_kfs_ = seg_covered_kfs_ = 0;  // GAP-2: new segment starts fresh
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
  // R0.1 independent DR-gate channel (gyro-only orientation + coasting translation).
  std::string imu_topic_, dr_gate_path_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  // Compass instrument (task #3): field-norm-gated raw-mag heading.
  std::string mag_topic_, compass_csv_path_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_mag_;
  std::FILE* compass_file_ = nullptr;
  double mag_norm_min_ut_ = 25.0, mag_norm_max_ut_ = 65.0;
  long mag_total_ = 0, mag_accepted_ = 0;
  // BNO055 absolute-yaw compass PRIOR (ported from COMPASS_RTABMAP.md). Anchors the one
  // unobservable VIO DOF (yaw), gated by mag-cal + field-norm + innovation, adaptive sigma.
  bool compass_yaw_prior_ = false;
  std::string bno_imu_topic_, bno_calib_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_bno_imu_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_bno_calib_;
  std::deque<std::pair<double, double>> bno_yaw_buf_;  // (t, yaw_bno[rad])
  double bno_last_norm_ut_ = -1.0;
  int bno_mag_cal_ = 0;
  bool yaw_offset_locked_ = false;
  double yaw_offset_ = 0.0, yaw_off_sin_ = 0.0, yaw_off_cos_ = 0.0;
  double yaw_init_window_s_ = 4.0, yaw_init_t0_ = -1.0;
  double yaw_nom_sum_ = 0.0, yaw_nominal_ut_ = -1.0;
  int yaw_nom_n_ = 0, yaw_min_cal_mag_ = 2;
  double yaw_band_tol_ut_ = 8.0;
  double yaw_sigma_min_deg_ = 5.0, yaw_sigma_max_deg_ = 45.0, yaw_innov_gate_deg_ = 30.0;
  int yaw_opt_every_ = 15, yaw_priors_added_ = 0;
  long yaw_gated_cal_ = 0, yaw_gated_band_ = 0, yaw_gated_innov_ = 0;
  std::FILE* dr_gate_file_ = nullptr;
  slamko::SO3 dr_R_;                 // integrated world<-body orientation (gyro)
  slamko::SO3 dr_R_at_last_odom_;    // its value at the last accepted odom sample
  slamko::SE3 last_odom_T_;          // last accepted OKVIS pose (pre-gap snapshot)
  Eigen::Vector3d last_odom_v_ = Eigen::Vector3d::Zero();  // last OKVIS body velocity
  double last_imu_t_ = -1.0;
  bool loss_in_segment_ = false;   // a stale-gap occurred -> next chain edge is SOFT
  bool seg_hard_loss_ = false;     // a TRUE stale-gap loss this segment -> bar reloc target (R0.2)
  // GAP-2 "don't re-map what you already see" (task #4 first brick).
  bool dup_suppress_ = true;
  int dup_min_inliers_ = 60, seg_total_kfs_ = 0, seg_covered_kfs_ = 0, suppressed_dups_ = 0;
  double dup_cover_window_s_ = 4.0, dup_cover_frac_ = 0.6, covered_until_t_ = -1.0;
  std::uint64_t covered_by_submap_ = 0;
  // GAP-2 maturation: refine prior submaps with revisit obs at shutdown (task #4 proper).
  bool mature_enabled_ = true;
  double mature_voxel_ = 0.06, mature_alpha_ = 0.2;
  std::string mature_out_dir_, prior_dir_;
  std::unordered_map<std::uint64_t, std::vector<Eigen::Vector3d>> mature_buf_;
  // GAP-2 cull backstop: real-world voxel occupancy of all KEPT submaps (+ prior seed).
  bool cull_enabled_ = true;
  bool occ_refresh_ = true;   // P2: refresh occupancy from loop-corrected anchors (fuse revisits)
  double cull_voxel_ = 0.10, cull_redundant_frac_ = 0.7;
  int cull_min_lms_ = 100, culled_submaps_ = 0;
  bool cull_viewpoint_aware_ = true;
  double cull_vp_frac_ = 0.5;
  // Phase A persistent-MapPoint identity (PLAN_PERSISTENT_MAPPOINTS_02): global store of
  // points keyed by descriptor for DRIFT-TOLERANT cross-submap association at seal.
  bool mappoint_assoc_ = false;
  bool mappoint_refine_ = false;  // Phase B: fold revisits into the consensus + back-prop
  slamko::MapPointStore mp_store_;
  int culled_mp_ = 0;   // duplicates the voxel cull MISSED, caught by descriptor match
  std::unordered_set<std::int64_t> occ_;
  // P2 (fusion / immortal-bounding): the PRIOR map's occupancy (already global, fixed —
  // never refreshed) kept apart from the session occ_ so a cross-session revisit of
  // prior-mapped ground is culled, not duplicated. kept_lm_local_ caches each KEPT session
  // submap's local landmarks so occ_ can be REFRESHED from the loop-corrected anchors
  // (after a loop removes drift, the revisit aligns → its duplicates fall in occupied
  // voxels → fused). This is what makes the self-correcting map ABSORB revisits.
  std::unordered_set<std::int64_t> prior_occ_;
  std::unordered_map<std::uint64_t, std::vector<Eigen::Vector3d>> kept_lm_local_;
  // R0.2 seal-quality gate: submaps sealed during degraded tracking are barred as reloc targets.
  bool gate_degraded_reloc_ = true;
  double dr_gate_reject_deg_ = 15.0;
  int gated_reloc_targets_ = 0;
  std::unordered_set<std::uint64_t> degraded_submaps_;
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
  // #12 DR-gate soft chain-edge covariance (trunk step 1).
  bool dr_gate_soft_cov_ = false;
  double dr_gate_soft_floor_t_ = 0.05, dr_gate_soft_floor_r_ = 0.02;
  double pending_soft_sigma_t_ = 0.0, pending_soft_sigma_r_ = 0.0;
  // ATLAS disjoint islands (etapa 1b): break the chain on loss -> new map component.
  bool atlas_break_on_loss_ = false;
  bool pending_break_ = false;
  int component_id_ = 0;
  int atlas_min_component_kfs_ = 20;
  int kfs_in_component_ = 0;   // keyframes in the ACTIVE component (reset on break)
  std::unordered_map<std::uint64_t, int> submap_component_;  // submap id -> atlas component
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
  // cross-session drift correction via per-keyframe prior factors (A: fix doubling)
  bool xsession_prior_factor_ = true;
  double xsession_prior_sigma_t_ = 0.15, xsession_prior_sigma_r_ = 0.08;
  double xsession_prior_jump_max_ = 2.0;
  int xsession_priors_added_ = 0;
  double proximity_radius_ = 3.0;   // E: proximity-detection radius [m] (0 = off)
  // 3-tier candidate->soft->weld for the proximity path (never-false-merge defense).
  bool proximity_three_tier_ = true;
  int proximity_promote_inliers_ = 40;
  double proximity_agree_m_ = 0.5;
  // ROBUST accumulative 2nd-vote: votes per prior submap (not a single overwriteable slot).
  struct ProxVote { int count = 0; slamko::SE3 corr; };  // corr = implied session->global
  std::unordered_map<std::uint64_t, ProxVote> prox_votes_;
  int proximity_votes_needed_ = 2;
  double proximity_vote_agree_m_ = 2.0;
  int prox_candidates_ = 0, prox_promoted_ = 0;
  // Accumulated viz edge segments by class (session frame). Chain/Soft are rebuilt from
  // anchor_edges_ each push; these three accumulate kf<->target links as matches fire.
  std::vector<std::array<Eigen::Vector3d, 2>> viz_cand_segs_;   // proximity, pre-promotion (grey)
  std::vector<std::array<Eigen::Vector3d, 2>> viz_loop_segs_;   // same-session loop (red)
  std::vector<std::array<Eigen::Vector3d, 2>> viz_prior_segs_;  // cross-session prior (green)

  // ----- live viz (Rerun); no-op unless built with SLAMKO_WITH_RERUN + a viewer.
  slamko::VizSink viz_;
  bool viz_enable_ = false;
  std::string viz_endpoint_;
  int viz_kf_since_match_ = 1000;     // keyframes since the last verified match (certainty)

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
