// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// volumetric_mapper.hpp — owns the per-keyframe depth SOURCE + a VolumetricBackend
// and produces the bend. `reintegrate()` IS the deformation: drop the backend's
// fused geometry, then fuse every stored frame at its keyframe's CURRENT corrected
// world pose. Offline-first calls it ONCE after the graph is final → perfect
// smooth bend by construction (PLVS OnMapChange / RTAB on-demand, run once). The
// same call is the live hook later (re-integrate the touched window on a loop).
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

  // Store a per-keyframe depth source (kept re-poseable for the bend).
  void addFrame(DepthFrame frame);
  std::size_t numFrames() const { return frames_.size(); }

  // The bend: reset the backend, then fuse every stored frame at its keyframe's
  // corrected world (map-frame) BODY pose. `kf_world_pose` maps keyframe id →
  // T_map_body. Frames whose keyframe is absent from the map (e.g. a dangling
  // island not yet anchored) are skipped. Returns the count actually integrated.
  std::size_t reintegrate(
      const std::unordered_map<std::uint64_t, SE3>& kf_world_pose);

  CostmapSlice exportCostmap(const CostmapParams& params);
  void exportMesh(const std::string& ply_path);

  bool backendAvailable() const { return backend_ && backend_->available(); }
  const VolumetricParams& params() const { return params_; }

 private:
  std::unique_ptr<VolumetricBackend> backend_;
  VolumetricParams params_;
  std::vector<DepthFrame> frames_;
};

}  // namespace slamko
