// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys

#include "slamko_tsdf/volumetric_mapper.hpp"

#include <utility>

namespace slamko {

VolumetricMapper::VolumetricMapper(std::unique_ptr<VolumetricBackend> backend,
                                   VolumetricParams params)
    : backend_(std::move(backend)), params_(params) {}

void VolumetricMapper::addFrame(DepthFrame frame) {
  frames_.push_back(std::move(frame));
}

std::size_t VolumetricMapper::reintegrate(
    const std::unordered_map<std::uint64_t, SE3>& kf_world_pose) {
  backend_->reset();
  std::size_t integrated = 0;
  for (const auto& f : frames_) {
    if (!f.valid()) continue;
    const auto it = kf_world_pose.find(f.kf_id);
    if (it == kf_world_pose.end()) continue;  // kf not anchored in the map → skip
    backend_->integrate(f, it->second);
    ++integrated;
  }
  return integrated;
}

CostmapSlice VolumetricMapper::exportCostmap(const CostmapParams& params) {
  return backend_->exportCostmap(params);
}

void VolumetricMapper::exportMesh(const std::string& ply_path) {
  backend_->exportMesh(ply_path);
}

}  // namespace slamko
