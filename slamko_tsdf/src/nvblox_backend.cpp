// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Two compilations behind one TU (the VizSink no-op pattern):
//   -DSLAMKO_WITH_NVBLOX  → the real GPU TSDF→ESDF backend (links nvblox + CUDA).
//   (default, macro off)  → a no-op stub; available()=false, the workspace builds
//                           CUDA-free and the unit tests run with a fake backend.
//
// nvblox API surface used (mapped from nvblox_core headers): Mapper(voxel),
// setMapperParams (decay OFF + high max_weight = clean static map), DepthImage +
// Camera + integrateDepth(depth, T_L_C, cam) where T_L_C = camera-in-LAYER (=
// camera-in-map, NOT inverse), updateEsdf + EsdfSlicer::sliceLayerToDistanceImage
// for the 2D ground slice. Honest-unknowns: never decay, never fill unobserved.

#include "slamko_tsdf/nvblox_backend.hpp"

#include <utility>

namespace slamko {

#ifdef SLAMKO_WITH_NVBLOX

}  // namespace slamko

#include "nvblox/core/types.h"
#include "nvblox/geometry/bounding_boxes.h"
#include "nvblox/integrators/esdf_slicer.h"
#include "nvblox/mapper/mapper.h"
#include "nvblox/mapper/mapper_params.h"
#include "nvblox/sensors/camera.h"
#include "nvblox/sensors/image.h"

#include <Eigen/Geometry>

namespace slamko {

namespace {
// SE3 (double) → nvblox Transform (Eigen::Isometry3f).
nvblox::Transform toTransform(const SE3& T) {
  nvblox::Transform iso;
  iso.matrix() = T.matrix().cast<float>();
  return iso;
}
}  // namespace

struct NvbloxBackend::Impl {
  VolumetricParams params;
  std::unique_ptr<nvblox::Mapper> mapper;
  nvblox::EsdfSlicer slicer;

  explicit Impl(VolumetricParams p) : params(p) { build(); }

  void build() {
    mapper = std::make_unique<nvblox::Mapper>(
        static_cast<float>(params.voxel_size_m));
    // Static-map invariants. Decay is explicit in nvblox (integration alone never
    // decays), so honest-unknowns just means we NEVER call decayTsdf(). High
    // max_weight = a clean, non-flickering static reconstruction.
    nvblox::MapperParams mp;
    mp.projective_integrator_params.projective_integrator_max_weight.set(
        static_cast<float>(params.max_weight));
    mp.projective_integrator_params
        .projective_integrator_max_integration_distance_m.set(
            static_cast<float>(params.max_integration_distance_m));
    mapper->setMapperParams(mp);
  }
};

NvbloxBackend::NvbloxBackend(VolumetricParams params)
    : impl_(std::make_unique<Impl>(params)) {}
NvbloxBackend::~NvbloxBackend() = default;

void NvbloxBackend::integrate(const DepthFrame& frame, const SE3& T_map_body) {
  if (!frame.valid()) return;
  // Camera-in-map (the "layer" frame) = body-in-map ∘ camera-in-body.
  const nvblox::Transform T_L_C = toTransform(T_map_body * frame.T_body_cam);

  nvblox::DepthImage depth(nvblox::MemoryType::kUnified);
  depth.copyFrom(frame.height, frame.width, frame.depth.data());

  const nvblox::Camera cam(
      static_cast<float>(frame.K.fx), static_cast<float>(frame.K.fy),
      static_cast<float>(frame.K.cx), static_cast<float>(frame.K.cy),
      frame.width, frame.height);

  impl_->mapper->integrateDepth(depth, T_L_C, cam);
}

bool NvbloxBackend::queryDistanceField(const std::vector<Eigen::Vector3d>& pts_map,
                                       std::vector<float>& dist,
                                       std::vector<float>& weight) const {
  // The DENSE geometric channel's read side: batch-sample the SIGNED TSDF at map-frame
  // points (one GPU→host copy). distance = signed distance to the nearest surface (within
  // the truncation band); weight = confidence (0 ⇒ never mapped ⇒ the ICP drops it). Used
  // by slamko_loop::registerToSdf to snap a live depth cloud onto the existing surfaces.
  const std::size_t n = pts_map.size();
  dist.assign(n, 0.0f);
  weight.assign(n, 0.0f);
  if (n == 0) return true;
  // Query the ESDF, not the truncated TSDF: the full Euclidean signed distance has a much
  // larger, smoother convergence basin than the ±truncation band, so the ICP pulls a cloud
  // in from farther (needed to recover real drift / a hard knock, not just sub-band). nvblox
  // updateEsdf() is incremental (recomputes only dirty blocks) → cheap when nothing changed
  // between query calls within one registration.
  impl_->mapper->updateEsdf();
  const auto& esdf = impl_->mapper->esdf_layer();
  const float vsize = esdf.voxel_size();
  std::vector<Eigen::Vector3f> pos(n);
  for (std::size_t i = 0; i < n; ++i) pos[i] = pts_map[i].cast<float>();
  std::vector<nvblox::EsdfVoxel> vox;
  std::vector<bool> ok;
  esdf.getVoxels(pos, &vox, &ok);
  for (std::size_t i = 0; i < n && i < vox.size(); ++i)
    if (ok[i] && vox[i].observed) {
      float dm = std::sqrt(vox[i].squared_distance_vox) * vsize;  // Euclidean dist [m]
      if (vox[i].is_inside) dm = -dm;                              // signed (− behind surface)
      dist[i] = dm;
      weight[i] = 1.0f;
    }
  return true;
}

void NvbloxBackend::reset() { impl_->build(); }  // fresh Mapper = drop all geometry

void NvbloxBackend::clearRegion(const Aabb& region) {
  // The live "touched window" primitive: drop only the TSDF blocks the region
  // overlaps so VolumetricMapper can re-fuse the affected frames at corrected
  // poses. The ESDF/mesh are derived → updateEsdf()/updateColorMesh() at export
  // recompute the cleared blocks from the re-fused TSDF.
  if (region.empty()) return;
  nvblox::AxisAlignedBoundingBox box(region.min.cast<float>(),
                                     region.max.cast<float>());
  auto& tsdf = impl_->mapper->tsdf_layer();
  const std::vector<nvblox::Index3D> blocks =
      nvblox::getBlockIndicesTouchedByBoundingBox(tsdf.block_size(), box);
  tsdf.clearBlocks(blocks);
}

CostmapSlice NvbloxBackend::exportCostmap(const CostmapParams& params) {
  CostmapSlice out;
  out.resolution = impl_->params.voxel_size_m;  // slice grid = voxel grid
  out.slice_height = params.slice_height_m;
  out.is_occupancy = params.occupancy;

  impl_->mapper->updateEsdf();  // 3D ESDF from the fused TSDF
  const auto& esdf = impl_->mapper->esdf_layer();

  const float h = static_cast<float>(params.slice_height_m);
  const nvblox::AxisAlignedBoundingBox aabb =
      impl_->slicer.getAabbOfLayerAtHeight(esdf, h);
  if (aabb.isEmpty()) return out;  // nothing observed yet

  // The slicer fills the image on the GPU (it reallocates to device memory
  // regardless of the type we pass), so we must copyTo a host buffer — a direct
  // memcpy from dataConstPtr() would dereference a device pointer (segfault).
  nvblox::Image<float> dist(nvblox::MemoryType::kDevice);
  impl_->slicer.sliceLayerToDistanceImage(esdf, h, out.unknown_value, aabb,
                                          &dist);
  if (dist.rows() <= 0 || dist.cols() <= 0) return out;

  out.height = dist.rows();
  out.width = dist.cols();
  out.origin_x = aabb.min().x();
  out.origin_y = aabb.min().y();
  out.data.resize(static_cast<std::size_t>(out.width) * out.height);
  dist.copyTo(out.data.data());  // device → host

  if (params.occupancy) {
    const float occ = static_cast<float>(params.occupied_distance_m);
    for (float& c : out.data) {
      if (c >= out.unknown_value) c = -1.f;          // unknown
      else if (c < occ) c = 100.f;                    // occupied (near/inside)
      else c = 0.f;                                    // free
    }
    out.is_occupancy = true;
  }
  return out;
}

void NvbloxBackend::exportMesh(const std::string& ply_path) {
  impl_->mapper->updateColorMesh();
  impl_->mapper->saveColorMeshAsPly(ply_path);
}

bool NvbloxBackend::available() const { return true; }

#else  // ---------------- no-op stub (CUDA-free build) ----------------

struct NvbloxBackend::Impl {};

NvbloxBackend::NvbloxBackend(VolumetricParams /*params*/) : impl_(nullptr) {}
NvbloxBackend::~NvbloxBackend() = default;
void NvbloxBackend::integrate(const DepthFrame&, const SE3&) {}
void NvbloxBackend::reset() {}
void NvbloxBackend::clearRegion(const Aabb&) {}
CostmapSlice NvbloxBackend::exportCostmap(const CostmapParams&) { return {}; }
void NvbloxBackend::exportMesh(const std::string&) {}
bool NvbloxBackend::available() const { return false; }

#endif

}  // namespace slamko
