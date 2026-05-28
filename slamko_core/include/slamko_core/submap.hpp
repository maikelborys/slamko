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

// Per-keyframe 2D OBSERVATIONS of landmarks (left image, optional right). One block
// per entry in `SubMap.keyframes` (aligned 1:1). Each row k refers to landmark
// `landmark_ids[k]` (id must exist in `SubMap.landmarks`) observed at pixel `uv.row(k)`
// in this keyframe's left image (and `uv_right.row(k)` for stereo, when present).
//
// Why this exists (docs/PLAN_BA_GLOBAL.md): the OKVIS-class loop closure (cm-aligned)
// is global BA over reprojection factors — each (KF, landmark, uv) is one factor. The
// flat `landmarks[]` cloud + KF poses alone cannot reconstruct the factors. The same
// data also enables real two-image LightGlue matching (query frame ↔ a stored keyframe's
// real 2D features, in-distribution) — the hard-revisit recall fix. Stored, not
// recomputed, so the cross-session map carries the BA substrate.
struct KeyframeObservations {
  std::vector<std::uint64_t> landmark_ids;  // N
  // N×2 left-image pixel coordinates (row k matches landmark_ids[k]).
  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> uv;
  // N×2 right-image pixel coordinates for stereo observations; empty (rows()==0) when
  // the observation is monocular only — that's a legal Features, the BA backend then
  // builds a mono reprojection factor instead of a stereo one.
  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> uv_right;

  // Per-keyframe global place-recognition (VPR) descriptor (EigenPlaces, 512-D,
  // L2-normalized). Empty (size()==0) when no VPR front-end ran for this KF. SubMap-
  // level `global_descriptor` stays as the submap's representative; this gives the
  // relocalizer FINER-GRAIN retrieval — on hard revisits the right KF inside a 10-m
  // submap is what carries the place signal, not a single aggregated vector. See
  // docs/PLAN_BA_GLOBAL.md "VPR retrieval: keep EigenPlaces, change granularity first".
  Eigen::VectorXf global_descriptor;

  int size() const { return static_cast<int>(landmark_ids.size()); }
  bool hasStereo() const { return uv_right.rows() == uv.rows() && uv_right.rows() > 0; }
  bool hasGlobalDescriptor() const { return global_descriptor.size() > 0; }
};

struct SubMap {
  std::uint64_t id = 0;
  SE3 anchor;  // submap-local -> global; the only cross-submap coupling

  std::vector<KeyframePose> keyframes;
  std::vector<MapLandmark> landmarks;

  // Compact descriptor index for relocalization (N×D, rows referenced by
  // MapLandmark::descriptor_row). Empty for descriptor-less runs.
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> descriptors;

  // Per-keyframe 2D landmark observations, aligned 1:1 with `keyframes` when present.
  // Empty (size()==0) for legacy / non-BA-capable maps; SMP1/SMP2 loads leave it empty.
  // See KeyframeObservations + docs/PLAN_BA_GLOBAL.md.
  std::vector<KeyframeObservations> kf_obs;

  // Global place-recognition (VPR) descriptor — one L2-normalized vector (EigenPlaces,
  // 512-D) of this submap's appearance, for coarse loop-closure RETRIEVAL (cosine-NN);
  // the local `descriptors` above then geometrically verify. Empty when no VPR front-end
  // ran. XFeat local descriptors carry no place signal — see docs/PLAN_VPR_RELOC.md.
  Eigen::VectorXf global_descriptor;

  CustomData custom_data;  // optional dense payload (occupancy slab), etc.
};

}  // namespace slamko
