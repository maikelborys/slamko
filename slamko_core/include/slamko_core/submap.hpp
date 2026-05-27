// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// SubMap — the unit of the global map and the "archive-don't-discard" object
// (MASTER_PLAN §5/§6). Self-contained and serializable: keyframe poses,
// landmarks, a small descriptor index for place recognition, an optional dense
// payload, and an anchor pose that places it in the global frame. The
// custom_data hatch (GLIM) lets it carry sensor-specific extras. Lives in
// slamko_core until slamko_mapping is split out (P4).

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "slamko_core/custom_data.hpp"
#include "slamko_core/features.hpp"
#include "slamko_core/se3.hpp"

namespace slamko {

struct KeyframePose {
  std::uint64_t id = 0;
  double timestamp = 0.0;
  SE3 T_WB;  // in this submap's local frame (compose with anchor for global)
};

struct MapLandmark {
  std::uint64_t id = 0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // submap-local
  int descriptor_row = -1;  // index into the submap descriptor block, or -1
};

struct SubMap {
  std::uint64_t id = 0;
  SE3 anchor;  // submap-local -> global; the only cross-submap coupling

  std::vector<KeyframePose> keyframes;
  std::vector<MapLandmark> landmarks;

  // Compact descriptor index for relocalization (N×D, rows referenced by
  // MapLandmark::descriptor_row). Empty for descriptor-less runs.
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> descriptors;

  // Global place-recognition (VPR) descriptor — one L2-normalized vector (EigenPlaces,
  // 512-D) of this submap's appearance, for coarse loop-closure RETRIEVAL (cosine-NN);
  // the local `descriptors` above then geometrically verify. Empty when no VPR front-end
  // ran. XFeat local descriptors carry no place signal — see docs/PLAN_VPR_RELOC.md.
  Eigen::VectorXf global_descriptor;

  CustomData custom_data;  // optional dense payload (occupancy slab), etc.
};

}  // namespace slamko
