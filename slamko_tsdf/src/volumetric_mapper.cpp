// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys

#include "slamko_tsdf/volumetric_mapper.hpp"

#include <algorithm>
#include <utility>

namespace slamko {

namespace {
// Above this moved-fraction, clearing the touched window costs more than a clean
// full rebuild — fall back to reset + re-fuse all (the offline bend).
constexpr double kWindowFullFraction = 0.6;
}  // namespace

VolumetricMapper::VolumetricMapper(std::unique_ptr<VolumetricBackend> backend,
                                   VolumetricParams params)
    : backend_(std::move(backend)), params_(params) {}

void VolumetricMapper::addFrame(DepthFrame frame) {
  StoredFrame sf;
  sf.frame = std::move(frame);
  frames_.push_back(std::move(sf));
}

Aabb VolumetricMapper::footprintOf(const DepthFrame& f,
                                   const SE3& T_map_body) const {
  Aabb box;
  // Camera-in-map (depth is observed in the camera frame).
  const SE3 T_map_cam = T_map_body * f.T_body_cam;
  const double d = params_.max_integration_distance_m;
  const double fx = f.K.fx != 0 ? f.K.fx : 1.0;
  const double fy = f.K.fy != 0 ? f.K.fy : 1.0;
  // Camera origin + the four image-corner rays cast to the max range — a tight
  // conservative hull of everything this depth frame can write.
  box.expand(T_map_cam.translation());
  const double us[2] = {0.0, static_cast<double>(f.K.width)};
  const double vs[2] = {0.0, static_cast<double>(f.K.height)};
  for (double u : us) {
    for (double v : vs) {
      const Eigen::Vector3d ray_cam((u - f.K.cx) / fx * d,
                                    (v - f.K.cy) / fy * d, d);
      box.expand(T_map_cam * ray_cam);
    }
  }
  return box;
}

std::size_t VolumetricMapper::reintegrate(
    const std::unordered_map<std::uint64_t, SE3>& kf_world_pose) {
  backend_->reset();
  std::size_t integrated = 0;
  for (auto& sf : frames_) {
    sf.fused = false;
    if (sf.sealed || !sf.frame.valid()) continue;
    const auto it = kf_world_pose.find(sf.frame.kf_id);
    if (it == kf_world_pose.end()) continue;  // kf not anchored → skip
    backend_->integrate(sf.frame, it->second);
    sf.world_pose = it->second;
    sf.footprint = footprintOf(sf.frame, it->second);
    sf.fused = true;
    ++integrated;
  }
  return integrated;
}

bool VolumetricMapper::integrateLive(DepthFrame frame,
                                     const SE3& kf_world_pose) {
  if (!frame.valid()) return false;
  StoredFrame sf;
  sf.frame = std::move(frame);
  sf.world_pose = kf_world_pose;
  sf.footprint = footprintOf(sf.frame, kf_world_pose);
  sf.fused = true;
  backend_->integrate(sf.frame, kf_world_pose);
  frames_.push_back(std::move(sf));
  return true;
}

std::size_t VolumetricMapper::reintegrateWindow(
    const std::vector<std::uint64_t>& moved_kfs,
    const std::unordered_map<std::uint64_t, SE3>& kf_world_pose) {
  if (moved_kfs.empty()) return 0;

  // The region to rebuild = union of each moved frame's OLD footprint (where its
  // contribution is now stale) and NEW footprint (where it must be re-fused).
  Aabb region;
  std::size_t moved_live = 0;
  for (std::uint64_t kf : moved_kfs) {
    const auto it = kf_world_pose.find(kf);
    if (it == kf_world_pose.end()) continue;
    for (auto& sf : frames_) {
      if (sf.frame.kf_id != kf || sf.sealed || !sf.frame.valid()) continue;
      region.expand(sf.footprint);                          // old (stale) hull
      region.expand(footprintOf(sf.frame, it->second));     // new (corrected) hull
      ++moved_live;
    }
  }
  if (region.empty()) return 0;

  // Most of the map moved → a clean full rebuild is cheaper than clearing a
  // near-global region and re-fusing nearly everything anyway.
  const std::size_t live = numLiveFrames();
  if (live > 0 && static_cast<double>(moved_live) / static_cast<double>(live) >
                      kWindowFullFraction) {
    return reintegrate(kf_world_pose);
  }

  backend_->clearRegion(region);

  // Re-fuse every stored, unsealed frame whose (corrected) footprint overlaps the
  // cleared region — moved frames at their new pose, untouched neighbours at their
  // existing pose (their contribution was just cleared and must be put back).
  std::size_t refused = 0;
  for (auto& sf : frames_) {
    if (sf.sealed || !sf.frame.valid()) continue;
    const auto it = kf_world_pose.find(sf.frame.kf_id);
    const bool moved = it != kf_world_pose.end();
    const SE3 pose = moved ? it->second : sf.world_pose;
    const Aabb fp = moved ? footprintOf(sf.frame, pose) : sf.footprint;
    if (!fp.intersects(region)) continue;
    backend_->integrate(sf.frame, pose);
    sf.world_pose = pose;
    sf.footprint = fp;
    sf.fused = true;
    ++refused;
  }
  return refused;
}

bool VolumetricMapper::sealFrame(std::uint64_t kf_id) {
  for (auto& sf : frames_) {
    if (sf.frame.kf_id == kf_id && !sf.sealed && sf.frame.valid()) {
      sf.sealed = true;
      sf.frame.depth.clear();
      sf.frame.depth.shrink_to_fit();  // actually release the payload
      return true;
    }
  }
  return false;
}

std::size_t VolumetricMapper::storeBytes() const {
  std::size_t bytes = 0;
  for (const auto& sf : frames_)
    if (!sf.sealed) bytes += sf.frame.depth.size() * sizeof(float);
  return bytes;
}

std::size_t VolumetricMapper::numLiveFrames() const {
  std::size_t n = 0;
  for (const auto& sf : frames_)
    if (!sf.sealed && sf.frame.valid()) ++n;
  return n;
}

CostmapSlice VolumetricMapper::exportCostmap(const CostmapParams& params) {
  return backend_->exportCostmap(params);
}

void VolumetricMapper::exportMesh(const std::string& ply_path) {
  backend_->exportMesh(ply_path);
}

}  // namespace slamko
