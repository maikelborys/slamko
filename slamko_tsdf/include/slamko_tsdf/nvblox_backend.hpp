// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// nvblox_backend.hpp — VolumetricBackend backed by NVIDIA nvblox (GPU TSDF→ESDF).
// PIMPL so the nvblox + CUDA headers never leak into slamko_core consumers
// (mirrors the VizSink/Rerun pattern). When the package is built WITHOUT
// -DSLAMKO_WITH_NVBLOX the methods are no-ops and available() is false, so the
// whole workspace still builds CUDA-free and the unit tests run with a fake
// backend. Static-map invariants (decay OFF, high max_weight, honest unknowns)
// are set from VolumetricParams in the impl ctor.

#pragma once

#include <memory>
#include <string>

#include "slamko_core/volumetric_map.hpp"

namespace slamko {

class NvbloxBackend : public VolumetricBackend {
 public:
  explicit NvbloxBackend(VolumetricParams params);
  ~NvbloxBackend() override;

  void integrate(const DepthFrame& frame, const SE3& T_map_body) override;
  void reset() override;
  void clearRegion(const Aabb& region) override;
  CostmapSlice exportCostmap(const CostmapParams& params) override;
  void exportMesh(const std::string& ply_path) override;
  bool available() const override;
  bool queryDistanceField(const std::vector<Eigen::Vector3d>& pts_map,
                          std::vector<float>& dist,
                          std::vector<float>& weight) const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace slamko
