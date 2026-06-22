// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// slamko_tsdf_export — the v0 OFFLINE driver. Loads a slamko submap archive +
// per-keyframe depth files, re-integrates the TSDF at the keyframes' CORRECTED
// world poses (the bend, run once after the graph is final), and writes a Nav2
// costmap (PGM + YAML) + a mesh (PLY) for the global A→B planner.
//
//   slamko_tsdf_export <submap_dir> <depth_dir> <out_prefix>
//       [voxel_m=0.05] [slice_height_m=0.10]
//
// <submap_dir>  : a saveSubMaps() archive (submap_*.smap + submaps.manifest).
// <depth_dir>   : *.skdf depth files (see depth_io.hpp), one per keyframe.
// <out_prefix>  : writes <prefix>.pgm + <prefix>.yaml + <prefix>.ply.
//
// Without -DSLAMKO_WITH_NVBLOX the backend is a no-op: the pipeline still runs
// (load → re-integrate → export) but the costmap is empty — a CUDA-free smoke.

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "slamko_core/submap.hpp"
#include "slamko_core/submap_io.hpp"
#include "slamko_tsdf/depth_io.hpp"
#include "slamko_tsdf/nvblox_backend.hpp"
#include "slamko_tsdf/submap_poses.hpp"
#include "slamko_tsdf/volumetric_mapper.hpp"

namespace fs = std::filesystem;
using namespace slamko;

namespace {

// Nav2 occupancy PGM (P5) + YAML. CostmapSlice has +row → +y; Nav2 reads the top
// image row as the HIGHEST y, so we emit rows bottom-up. origin = bottom-left.
bool writeOccupancyMap(const CostmapSlice& m, const std::string& prefix) {
  if (m.empty() || !m.is_occupancy) return false;
  const std::string pgm = prefix + ".pgm";
  std::ofstream o(pgm, std::ios::binary);
  if (!o) return false;
  o << "P5\n" << m.width << " " << m.height << "\n255\n";
  for (int r = m.height - 1; r >= 0; --r) {     // bottom-up → top row = max y
    for (int c = 0; c < m.width; ++c) {
      const float v = m.data[static_cast<std::size_t>(r) * m.width + c];
      unsigned char px = 205;                   // unknown (Nav2 grey)
      if (v == 0.f) px = 254;                    // free
      else if (v >= 100.f) px = 0;               // occupied
      o.put(static_cast<char>(px));
    }
  }
  o.close();

  std::ofstream y(prefix + ".yaml");
  if (!y) return false;
  y << "image: " << fs::path(pgm).filename().string() << "\n"
    << "resolution: " << m.resolution << "\n"
    << "origin: [" << m.origin_x << ", " << m.origin_y << ", 0.0]\n"
    << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: slamko_tsdf_export <submap_dir> <depth_dir> "
                 "<out_prefix> [voxel_m=0.05] [slice_height_m=0.10]\n";
    return 2;
  }
  const std::string submap_dir = argv[1];
  const std::string depth_dir = argv[2];
  const std::string out_prefix = argv[3];

  VolumetricParams vp;
  if (argc > 4) vp.voxel_size_m = std::stod(argv[4]);
  CostmapParams cp;
  cp.occupancy = true;
  cp.resolution_m = vp.voxel_size_m;
  if (argc > 5) cp.slice_height_m = std::stod(argv[5]);

  std::vector<SubMap> submaps;
  if (!loadSubMaps(submaps, submap_dir)) {
    std::cerr << "FAIL: could not load submap archive: " << submap_dir << "\n";
    return 1;
  }
  const auto poses = keyframeWorldPoses(submaps);
  std::cout << "loaded " << submaps.size() << " submaps, " << poses.size()
            << " keyframe poses\n";

  VolumetricMapper mapper(std::make_unique<NvbloxBackend>(vp), vp);
  if (!mapper.backendAvailable())
    std::cerr << "WARN: nvblox backend not built (-DSLAMKO_WITH_NVBLOX=OFF) — "
                 "costmap will be EMPTY (CUDA-free smoke)\n";

  std::size_t loaded = 0;
  if (fs::is_directory(depth_dir)) {
    std::vector<fs::path> files;
    for (const auto& e : fs::directory_iterator(depth_dir))
      if (e.path().extension() == ".skdf") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    for (const auto& p : files) {
      DepthFrame f;
      if (loadDepthFrame(f, p.string())) {
        mapper.addFrame(std::move(f));
        ++loaded;
      } else {
        std::cerr << "WARN: bad depth file skipped: " << p << "\n";
      }
    }
  }
  std::cout << "loaded " << loaded << " depth frames\n";

  const std::size_t integrated = mapper.reintegrate(poses);
  std::cout << "integrated " << integrated << "/" << mapper.numFrames()
            << " frames at corrected poses (skipped = unanchored/invalid)\n";

  const CostmapSlice cm = mapper.exportCostmap(cp);
  if (writeOccupancyMap(cm, out_prefix))
    std::cout << "wrote " << out_prefix << ".pgm/.yaml  (" << cm.width << "x"
              << cm.height << " @ " << cm.resolution << " m)\n";
  else
    std::cout << "no costmap written (empty slice)\n";

  mapper.exportMesh(out_prefix + ".ply");
  return 0;
}
