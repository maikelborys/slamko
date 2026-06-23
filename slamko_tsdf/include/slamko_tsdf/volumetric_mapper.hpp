// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// volumetric_mapper.hpp — owns the per-keyframe depth SOURCE + a VolumetricBackend
// and produces the bend. `reintegrate()` IS the deformation: drop the backend's
// fused geometry, then fuse every stored frame at its keyframe's CURRENT corrected
// world pose. Offline-first calls it ONCE after the graph is final → perfect
// smooth bend by construction (PLVS OnMapChange / RTAB on-demand, run once).
//
// LIVE path (this layer is the engine the slamko_ros volumetric node drives):
//   - integrateLive(frame, pose): forward incremental fusion as keyframes arrive —
//     no reset, the map grows live. The frame is KEPT (re-poseable) with its world
//     pose + footprint for a later bend.
//   - reintegrateWindow(moved_kfs, poses): the touched-window bend. A loop closure
//     moves a few keyframes; we clear ONLY the region they touch (old ∪ new
//     footprint) and re-fuse the stored frames overlapping it at corrected poses —
//     O(frames touching the window), not O(whole map). Falls back to a full
//     reintegrate when most of the map moved (cheaper than clearing a huge region).
//   - sealFrame / store bounding: the per-kf depth store is the memory cost (the
//     751 MB problem). sealFrame(kf) drops a stable frame's depth payload — its
//     geometry stays baked but it is no longer re-poseable. The POLICY (which
//     frames to seal) lives in the ROS node; the MECHANISM lives here.
//
// This is the slamko-NATIVE piece: unlike nvblox_submap (rigid per-submap anchor,
// RTAB-node-coupled, zero re-integration → piecewise), we re-integrate the
// per-keyframe source at corrected poses → smooth. See docs/PLAN_SLAMKO_TSDF_01.md.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "slamko_core/volumetric_map.hpp"

namespace slamko {

class VolumetricMapper {
 public:
  VolumetricMapper(std::unique_ptr<VolumetricBackend> backend,
                   VolumetricParams params);

  // Store a per-keyframe depth source (kept re-poseable for the bend). OFFLINE
  // path: pose is supplied later at reintegrate() time, so none is stored here.
  void addFrame(DepthFrame frame);
  std::size_t numFrames() const { return frames_.size(); }

  // The (offline) bend: reset the backend, then fuse every stored frame at its
  // keyframe's corrected world (map-frame) BODY pose. `kf_world_pose` maps
  // keyframe id → T_map_body. Frames whose keyframe is absent from the map (e.g.
  // a dangling island not yet anchored) are skipped. Returns the count actually
  // integrated. Also records each frame's integrated pose + footprint so the LIVE
  // window path can continue from a full build.
  std::size_t reintegrate(
      const std::unordered_map<std::uint64_t, SE3>& kf_world_pose);

  // ---- LIVE path -----------------------------------------------------------

  // Forward incremental fusion: fuse this frame NOW at `kf_world_pose` and keep
  // it (re-poseable, with footprint) for a later window bend. No reset — the map
  // grows. Returns false if the frame is invalid (not stored, not fused).
  bool integrateLive(DepthFrame frame, const SE3& kf_world_pose);

  // The touched-window bend. `moved_kfs` = keyframes whose pose changed in this
  // correction; `kf_world_pose` = the NEW corrected poses (must contain the moved
  // ones). Clears the region the moved frames touch (old ∪ new footprint) and
  // re-fuses every stored, unsealed frame overlapping it at its corrected pose.
  // Falls back to a full reintegrate() when the moved fraction exceeds
  // `window_full_fraction` (clearing most of the map is wasteful). Returns the
  // number of frames re-integrated.
  std::size_t reintegrateWindow(
      const std::vector<std::uint64_t>& moved_kfs,
      const std::unordered_map<std::uint64_t, SE3>& kf_world_pose);

  // ---- depth-store bounding (the 751 MB lever) -----------------------------

  // Drop a stable frame's depth payload: its geometry stays baked in the backend
  // but it is no longer re-poseable (window bends skip it). Returns true if a
  // live, unsealed frame with this kf_id was sealed. The seal POLICY lives in the
  // ROS node (seal frames outside the active/loop window).
  bool sealFrame(std::uint64_t kf_id);

  // Bytes held by the re-poseable depth store (sum of unsealed frames' depth).
  std::size_t storeBytes() const;
  // Frames still re-poseable (stored, unsealed, valid).
  std::size_t numLiveFrames() const;

  CostmapSlice exportCostmap(const CostmapParams& params);
  void exportMesh(const std::string& ply_path);

  bool backendAvailable() const { return backend_ && backend_->available(); }
  const VolumetricParams& params() const { return params_; }

 private:
  // A stored depth source + where it was last fused (for re-pose / windowing).
  struct StoredFrame {
    DepthFrame frame;
    SE3 world_pose;      // T_map_body it was last integrated at
    Aabb footprint;      // world region its depth touches at world_pose
    bool sealed = false; // payload dropped → geometry frozen, not re-poseable
    bool fused = false;  // has been integrated into the backend at least once
  };

  // World AABB of a frame's depth frustum at `T_map_body` (camera origin + the
  // four image corners ray-cast to the max integration distance).
  Aabb footprintOf(const DepthFrame& f, const SE3& T_map_body) const;

  std::unique_ptr<VolumetricBackend> backend_;
  VolumetricParams params_;
  std::vector<StoredFrame> frames_;
};

}  // namespace slamko
