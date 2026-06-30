// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// live_driver.hpp — the LIVE policy over VolumetricMapper. The ROS node feeds it
// raw events (a new keyframe + its depth; a fresh snapshot of corrected world
// poses); the driver decides WHAT to re-integrate and WHEN to seal — keeping the
// node a thin shell and the policy pure-C++ unit-testable (no rclpy, no GPU).
//
// THE LOAD-BEARING POLICY (why this is decoupled from the pose-graph):
//   - addKeyframe(): forward-fuse a new kf immediately at its current world pose.
//   - applyCorrection(): given the LATEST corrected world poses (the live
//     pose-graph, transformed to the map frame), DIFF them against the pose each
//     kf was last fused at and window-re-integrate ONLY the ones that moved past a
//     threshold. This needs no hook into the optimizer — a loop closure simply
//     shows up as a set of moved poses on the next snapshot. Decoupled by design.
//   - enforceBudget(): bound the per-kf depth store (the 751 MB lever) by SEALING
//     the oldest frames outside the active (recent) window once the store exceeds
//     a byte budget. Sealing trades a frame's re-poseability for memory.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "slamko_core/volumetric_map.hpp"
#include "slamko_tsdf/volumetric_mapper.hpp"

namespace slamko {

struct LiveParams {
  // A keyframe whose corrected world pose shifted more than EITHER threshold since
  // it was last fused counts as "moved" and triggers a windowed re-integration.
  double move_threshold_m = 0.05;
  double move_threshold_rad = 0.02;  // ~1.1°
  // Depth-store bound. 0 = unbounded (keep every frame re-poseable). Otherwise
  // seal oldest frames outside `keep_recent` until the store fits the budget.
  std::size_t store_budget_bytes = 0;
  std::size_t keep_recent = 40;  // never seal the most recent N keyframes
};

class VolumetricLiveDriver {
 public:
  VolumetricLiveDriver(std::unique_ptr<VolumetricBackend> backend,
                       VolumetricParams vp, LiveParams lp);

  // A new keyframe arrived: forward-fuse its depth now at `world_pose` (T_map_body)
  // and remember the pose for future correction diffs. Returns false if invalid.
  bool addKeyframe(DepthFrame frame, const SE3& world_pose);

  // The pose-graph was (re-)optimized: `world_poses` = the CURRENT corrected
  // T_map_body for every keyframe node (the node composes T_global_map ∘ graph
  // pose before calling). Frames whose pose moved past threshold are window-
  // re-integrated; all held poses are refreshed. Returns #frames re-fused.
  std::size_t applyCorrection(
      const std::unordered_map<std::uint64_t, SE3>& world_poses);

  // Seal oldest re-poseable frames (outside the keep_recent window) until the
  // store fits store_budget_bytes. No-op when budget == 0. Returns #sealed.
  std::size_t enforceBudget();

  CostmapSlice costmap(const CostmapParams& cp) { return mapper_.exportCostmap(cp); }
  void exportMesh(const std::string& ply) { mapper_.exportMesh(ply); }
  // Dense geometric loop channel read side (depth-ICP-to-TSDF) — forwards to the backend ESDF.
  bool queryDistanceField(const std::vector<Eigen::Vector3d>& pts_map,
                          std::vector<float>& dist, std::vector<float>& weight) const {
    return mapper_.queryDistanceField(pts_map, dist, weight);
  }
  // DYNAMIC LOCAL reactive costmap (per-frame @ live pose, decays-to-free, bounded window).
  void integrateLocal(const DepthFrame& frame, const SE3& T_map_body) {
    mapper_.integrateLocal(frame, T_map_body);
  }
  CostmapSlice localCostmap(const CostmapParams& cp, const Eigen::Vector3d& center, double radius_m) {
    return mapper_.exportLocalCostmap(cp, center, radius_m);
  }

  bool backendAvailable() const { return mapper_.backendAvailable(); }
  std::size_t numKeyframes() const { return kf_order_.size(); }
  std::size_t numLiveFrames() const { return mapper_.numLiveFrames(); }
  std::size_t storeBytes() const { return mapper_.storeBytes(); }
  const LiveParams& liveParams() const { return lp_; }

 private:
  // Did the corrected pose move enough vs where the kf was last fused?
  bool moved(const SE3& last, const SE3& now) const;

  VolumetricMapper mapper_;
  LiveParams lp_;
  // Pose each held kf was LAST integrated at (the diff baseline for corrections).
  std::unordered_map<std::uint64_t, SE3> integrated_pose_;
  // Insertion order — defines "recent" (keep) vs "old" (sealable) for the budget.
  std::vector<std::uint64_t> kf_order_;
};

}  // namespace slamko
