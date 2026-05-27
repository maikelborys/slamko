// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// XFeatRelocalizer — the P2b place-recognition + geometric-verification backend
// (implements slamko_core::Relocalizer). It localizes a query frame against
// archived submaps using the XFeat descriptors the VIO already attaches to
// landmarks (SubMap's N×64 index) — NO new model: brute-force NN match (Lowe
// ratio) → 2D-3D correspondences → PnP-RANSAC (core P3P) → the query CAMERA pose
// in the matched submap's local frame, converted to the BODY frame via the
// cam↔body extrinsic.
//
// It returns RelocResult.T_query_match = query BODY pose in sealed-local
// (T_submaplocal_body). The never-lost supervisor composes that with the live
// odom to get the weld constraint (T_active_sealed = T_query_match · T_WB⁻¹ — the
// OKVIS2-X T_AB formula) and feeds it to the lazy-anchor gate. Keeping the
// extrinsic here (the relocalizer is camera-aware) lets the supervisor stay in
// the body frame. Depends on slamko_core only (Hard Rule #2) — no OpenCV.

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "slamko_core/features.hpp"
#include "slamko_core/relocalizer.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"

namespace slamko {

struct XFeatRelocConfig {
  double fx = 0, fy = 0, cx = 0, cy = 0;  // rectified pinhole intrinsics
  SE3    body_T_cam;                      // T_BS (cam→body); identity = cam ≡ body
  float  match_ratio = 0.8f;              // Lowe second-nearest ratio
  int    ransac_iters = 200;
  double ransac_thresh_px = 3.0;          // reprojection inlier threshold
  int    min_inliers = 15;                // accept a relocalization above this
  unsigned seed = 1u;                     // RANSAC RNG seed (deterministic)
  // Brute-force NN match is O(N_query · N_db · D); the cumulative submap can hold
  // tens of thousands of landmarks. Stride-subsample a registered submap to at
  // most this many descriptors so a relocalize() call stays cheap (a real system
  // would use a vocabulary/inverted index — that's the scalable swap).
  int    max_db_landmarks = 3000;
};

class XFeatRelocalizer : public Relocalizer {
 public:
  explicit XFeatRelocalizer(const XFeatRelocConfig& cfg) : cfg_(cfg) {}

  std::string name() const override { return "xfeat"; }

  // Register an archived submap: cache its descriptor rows + the matching
  // submap-local 3D positions (only landmarks that carry a descriptor).
  void addSubMap(const SubMap& submap) override;

  // Match query → each submap, PnP-RANSAC verify, return the best (most inliers).
  RelocResult relocalize(const Features& query) const override;

  std::size_t numSubMaps() const { return db_.size(); }

 private:
  struct Entry {
    std::uint64_t id = 0;
    SE3 anchor;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> desc;  // M×D
    std::vector<Eigen::Vector3d> pos;  // M submap-local 3D, aligned with desc rows
  };

  XFeatRelocConfig    cfg_;
  std::vector<Entry>  db_;
};

}  // namespace slamko
