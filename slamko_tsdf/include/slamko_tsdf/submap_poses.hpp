// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// submap_poses.hpp — the slamko-side glue that turns a loaded submap archive into
// the `kf_world_pose` map VolumetricMapper::reintegrate() consumes. A keyframe's
// CORRECTED world pose is `submap.anchor ∘ kf.T_WB` (the anchor carries the
// loop-closure / cross-session correction; T_WB is submap-local). Re-reading this
// after the graph moves is exactly what makes the dense map bend.

#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"

namespace slamko {

// keyframe id → corrected world (map-frame) BODY pose, over a whole archive.
// Later submaps win on a duplicate id (a re-sealed kf carries the newer anchor).
inline std::unordered_map<std::uint64_t, SE3> keyframeWorldPoses(
    const std::vector<SubMap>& submaps) {
  std::unordered_map<std::uint64_t, SE3> poses;
  for (const auto& sm : submaps) {
    for (const auto& kf : sm.keyframes) {
      poses[kf.id] = sm.anchor * kf.T_WB;  // submap-local → global
    }
  }
  return poses;
}

}  // namespace slamko
