// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// MapPointStore — persistent point identity for slamko (the ORB-SLAM3 abstraction
// slamko was missing). A global store of MapPoints (3D position in the global frame
// + a representative XFeat descriptor); the seal path queries it to RE-ASSOCIATE a
// revisit's landmarks to the points that already exist.
//
// Why this exists (docs/PLAN_PERSISTENT_MAPPOINTS_02.md). slamko's cross-submap dedup
// was VOXEL-occupancy only (provider_fusion_node occ_): a revisit point is culled iff
// its global voxel is already occupied. Under VIO drift > the voxel size the revisit's
// points fall in DIFFERENT voxels -> the cull misses -> a 2nd offset submap is sealed =
// the map DOUBLING. ORB-SLAM3 never relies on voxel coincidence; it re-associates the
// same physical point by DESCRIPTOR (drift-tolerant) and fuses. This store gives slamko
// that: associate by position-NEIGHBOURHOOD (generous radius, absorbs drift) AND
// descriptor COSINE (the discriminator). It does NOT do metric estimation — the geometry
// still comes from the provider; this is identity + data association only.
//
// Header-only, no deps beyond Eigen — included by provider_fusion_node and unit-testable
// in isolation. Descriptors are XFeat 64-D, L2-normalised (so cosine = dot product).

#pragma once

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace slamko {

struct MapPoint {
  std::uint64_t id = 0;
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();   // global frame
  Eigen::Matrix<float, 1, 64, Eigen::RowMajor> desc;  // representative (L2-normalised)
  int n_obs = 1;
};

class MapPointStore {
 public:
  // cell    : voxel-hash cell size for the spatial index [m] (>= radius so a query's
  //           neighbourhood is covered by the 3x3x3 cell stencil).
  // radius  : max metric distance for an association [m] (absorbs VIO drift).
  // cos     : min descriptor cosine for an association ([-1,1]; XFeat same-point ~0.85+).
  explicit MapPointStore(double cell = 0.4, double radius = 0.4, float cos = 0.82f)
      : cell_(cell), radius2_(radius * radius), cos_(cos) {}

  std::size_t size() const { return points_.size(); }
  const std::vector<MapPoint>& points() const { return points_; }

  // Return the id of the EXISTING MapPoint this (position, descriptor) re-observes, or
  // -1 if none. Searches the 3x3x3 cell stencil around p; among MapPoints within radius,
  // picks the highest descriptor cosine >= cos_. desc must be an L2-normalised 64-D row
  // (templated so an XFeat lm_desc.row(i) block binds without a copy).
  template <typename Desc>
  std::int64_t associate(const Eigen::Vector3d& p, const Desc& desc) const {
    std::int64_t best = -1;
    float best_cos = cos_;
    const std::int64_t cx = cell(p.x()), cy = cell(p.y()), cz = cell(p.z());
    for (std::int64_t dx = -1; dx <= 1; ++dx)
      for (std::int64_t dy = -1; dy <= 1; ++dy)
        for (std::int64_t dz = -1; dz <= 1; ++dz) {
          auto it = cells_.find(key(cx + dx, cy + dy, cz + dz));
          if (it == cells_.end()) continue;
          for (int idx : it->second) {
            const MapPoint& mp = points_[(std::size_t)idx];
            if ((mp.pos - p).squaredNorm() > radius2_) continue;
            const float c = mp.desc.dot(desc);  // both L2-normalised -> cosine
            if (c > best_cos) { best_cos = c; best = (std::int64_t)mp.id; }
          }
        }
    return best;
  }

  // Register a genuinely-new MapPoint. id is the caller's global landmark id (kept in
  // sync with the node's next_landmark_id_ so submap landmarks and MapPoints share ids).
  template <typename Desc>
  void add(std::uint64_t id, const Eigen::Vector3d& p, const Desc& desc) {
    const int idx = (int)points_.size();
    points_.push_back(MapPoint{id, p, desc, 1});
    cells_[key(cell(p.x()), cell(p.y()), cell(p.z()))].push_back(idx);
    id_index_[id] = idx;
  }

  // Record one more observation of an existing MapPoint (Phase B refine hook; for now
  // just bumps n_obs — the position refinement lands in Phase B so Phase A stays a pure
  // association/cull with zero geometry change).
  void addObservation(std::int64_t id) {
    auto it = id_index_.find((std::uint64_t)id);
    if (it != id_index_.end()) ++points_[(std::size_t)it->second].n_obs;
  }

 private:
  std::int64_t cell(double x) const { return (std::int64_t)std::floor(x / cell_); }
  static std::uint64_t key(std::int64_t a, std::int64_t b, std::int64_t c) {
    const std::uint64_t ua = (std::uint64_t)(a & 0x1FFFFF);
    const std::uint64_t ub = (std::uint64_t)(b & 0x1FFFFF);
    const std::uint64_t uc = (std::uint64_t)(c & 0x1FFFFF);
    return (ua << 42) | (ub << 21) | uc;
  }

  double cell_;
  double radius2_;
  float cos_;
  std::vector<MapPoint> points_;
  std::unordered_map<std::uint64_t, std::vector<int>> cells_;  // cell key -> point indices
  std::unordered_map<std::uint64_t, int> id_index_;           // MapPoint id -> index
};

}  // namespace slamko
