// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// volumetric_map.hpp — the `slamko_tsdf` contract: a dense volumetric map that
// BENDS with the pose-graph and exports a 2D costmap for global A→B planning.
//
// WHY THIS EXISTS / THE LOAD-BEARING DECISION
// -------------------------------------------
// A fused TSDF is *fuse-and-forget*: once a depth frame is integrated, its
// contribution blends into neighbouring voxels and is no longer re-poseable. A
// single monolithic volume therefore does NOT bend on loop closure (the same
// lesson as the lifelong-fusion work: a rigid SE3 re-base can't absorb
// path-growing drift). So the bend is NOT a backend feature — it is an
// architecture property of THIS layer:
//
//   keep the per-keyframe DEPTH SOURCE (re-poseable) → the TSDF is a DERIVED
//   product, re-integrated from the keyframe's CORRECTED pose. A loop closure
//   that moves the keyframe moves its contribution for free.
//
// This is PLVS `OnMapChange` (re-integrate per keyframe on a graph change) and
// RTAB-Map's "ensamblado on-demand" (nothing stored in world coords; the global
// map is assembled from graph poses). Offline-first makes it nearly free: run
// the re-integration ONCE after the graph is final → perfect smooth bend by
// construction (see docs/PLAN_SLAMKO_TSDF_01.md).
//
// The backend (nvblox first; voxblox / GPU-SGM / open3d swappable) only knows
// "fuse this frame at this world pose" and "give me the slice". The bend lives
// ABOVE it, in VolumetricMapper (slamko_tsdf). This header is the only coupling
// the contract exchanges (Hard Rule #2: packages depend on slamko_core only).
//
// Frame conventions: poses are BODY-in-map (T_map_body); the backend applies the
// static camera extrinsic T_body_cam itself. Depth is metric [m], row-major
// float32, 0 = no return. Honest-unknowns is a HARD invariant: never fill space
// that was never observed (no generative surface — it hallucinates free-space →
// unsafe navigation).

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "slamko_core/se3.hpp"

namespace slamko {

// Pinhole intrinsics of the depth/integration camera. (StereoCalib carries a
// baseline this layer doesn't need; a bare pinhole keeps the contract minimal.)
struct CameraIntrinsics {
  double fx = 0, fy = 0, cx = 0, cy = 0;
  int width = 0, height = 0;
};

// One depth observation tied to a keyframe — the RE-POSEABLE SOURCE. The world
// pose is deliberately NOT stored here: it is supplied at integration time from
// the (corrected) pose-graph, so the bend is free. `depth` is width*height
// metric depth [m], row-major, 0 = invalid/no-return.
struct DepthFrame {
  std::uint64_t kf_id = 0;     // keyframe this depth was observed from
  int width = 0, height = 0;
  std::vector<float> depth;    // width*height, metres, 0 = invalid
  CameraIntrinsics K;          // depth-camera intrinsics
  SE3 T_body_cam;              // static extrinsic: camera frame in body frame

  bool valid() const {
    return width > 0 && height > 0 &&
           depth.size() == static_cast<std::size_t>(width) * height;
  }
};

// A 2D ground-projected slice in the MAP frame. When `is_occupancy` is false,
// cells hold signed distance [m] to the nearest obstacle (an ESDF: ≥0 free,
// <0 inside obstacles, `unknown_value` where never observed). When true, cells
// hold occupancy {-1 unknown, 0 free, +100 occupied} (Nav2 OccupancyGrid).
// origin = map-frame XY of cell (row=0,col=0); +col → +x, +row → +y.
struct CostmapSlice {
  int width = 0, height = 0;
  double resolution = 0.05;            // metres per cell
  double origin_x = 0.0, origin_y = 0.0;
  double slice_height = 0.0;           // z of the slice in map frame [m]
  bool is_occupancy = false;
  float unknown_value = 1e6f;          // sentinel for never-observed (ESDF mode)
  std::vector<float> data;             // width*height, row-major

  bool empty() const { return data.empty(); }
};

// Backend integration / quality knobs. Honest-unknowns is enforced by NEVER
// enabling decay and NEVER filling unobserved space (no generative surface).
struct VolumetricParams {
  double voxel_size_m = 0.05;
  double max_integration_distance_m = 8.0;  // ignore depth beyond (RTAB casa ref)
  double max_weight = 100.0;                // high → clean static map, no decay
  double min_integration_distance_m = 0.3;  // ignore too-close (stereo garbage)
};

// Costmap export knobs.
struct CostmapParams {
  double slice_height_m = 0.10;        // height above the floor to slice
  double resolution_m = 0.05;
  bool occupancy = false;              // false = ESDF distance, true = occupancy
  double occupied_distance_m = 0.15;   // ESDF < this ⇒ occupied (occupancy mode)
};

// The volumetric backend contract — nvblox is one impl; voxblox / GPU-SGM /
// open3d are swappable behind it. Stateful: integrate-many, then export. The
// backend is pose-agnostic about HOW poses were obtained (live vs corrected);
// VolumetricMapper owns the re-integration policy that produces the bend.
class VolumetricBackend {
 public:
  virtual ~VolumetricBackend() = default;

  // Fuse one depth frame at a world (map-frame) BODY pose. The backend composes
  // T_body_cam internally. Cells never observed stay unknown (honest-unknowns).
  virtual void integrate(const DepthFrame& frame, const SE3& T_map_body) = 0;

  // Drop all fused geometry — for a full re-integration on a graph change.
  virtual void reset() = 0;

  // Update the distance field and export the 2D ground slice in the map frame.
  virtual CostmapSlice exportCostmap(const CostmapParams& params) = 0;

  // Optional: dump the dense mesh / cloud for visualization (PLY path).
  virtual void exportMesh(const std::string& /*ply_path*/) {}

  // False for the no-op stub compiled when the real backend isn't built, so
  // callers can degrade gracefully (mirrors the VizSink available() pattern).
  virtual bool available() const { return true; }
};

}  // namespace slamko
