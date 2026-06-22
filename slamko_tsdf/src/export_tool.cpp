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
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "slamko_core/submap.hpp"
#include "slamko_core/submap_io.hpp"
#include "slamko_tsdf/depth_io.hpp"
#include "slamko_tsdf/nvblox_backend.hpp"
#include "slamko_tsdf/submap_poses.hpp"
#include "slamko_tsdf/volumetric_mapper.hpp"

namespace fs = std::filesystem;
using namespace slamko;

namespace {

// Load a TUM trajectory (timestamp tx ty tz qx qy qz qw) → (ts, pose) pairs.
std::vector<std::pair<double, SE3>> loadTum(const std::string& path) {
  std::vector<std::pair<double, SE3>> out;
  std::ifstream f(path);
  double t, x, y, z, qx, qy, qz, qw;
  while (f >> t >> x >> y >> z >> qx >> qy >> qz >> qw) {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    out.emplace_back(t, SE3(SO3(q), Eigen::Vector3d(x, y, z)));
  }
  return out;
}

// RAW poses for the bend A/B: each keyframe placed at the provider's UNCORRECTED
// odometry pose (nearest TUM sample to the kf timestamp), NOT the loop-corrected
// anchor∘T_WB. Integrating depth here shows the drift/doubling the bend removes.
std::unordered_map<std::uint64_t, SE3> rawKeyframePoses(
    const std::vector<SubMap>& submaps,
    const std::vector<std::pair<double, SE3>>& tum) {
  std::unordered_map<std::uint64_t, SE3> poses;
  for (const auto& sm : submaps) {
    for (const auto& kf : sm.keyframes) {
      double best = 1e18;
      const SE3* hit = nullptr;
      for (const auto& [ts, p] : tum) {
        const double d = std::abs(ts - kf.timestamp);
        if (d < best) { best = d; hit = &p; }
      }
      if (hit && best < 0.05) poses[kf.id] = *hit;  // 50 ms match window
    }
  }
  return poses;
}

// Load every *.skdf in a dir into the mapper, offsetting kf_ids (so a 2nd session
// can share one volume without colliding with session 1's kf numbering).
std::size_t loadDepthInto(VolumetricMapper& m, const std::string& dir,
                          std::uint64_t off) {
  if (!fs::is_directory(dir)) return 0;
  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(dir))
    if (e.path().extension() == ".skdf") files.push_back(e.path());
  std::sort(files.begin(), files.end());
  std::size_t n = 0;
  for (const auto& p : files) {
    DepthFrame f;
    if (loadDepthFrame(f, p.string())) {
      f.kf_id += off;
      m.addFrame(std::move(f));
      ++n;
    }
  }
  return n;
}

// A session's kf_world_pose (corrected anchor∘T_WB, or raw if raw_tum set), keyed
// with the same offset used for its depth frames.
std::unordered_map<std::uint64_t, SE3> sessionPoses(
    const std::vector<SubMap>& sm, const std::string& raw_tum,
    std::uint64_t off) {
  auto base = raw_tum.empty() ? keyframeWorldPoses(sm)
                              : rawKeyframePoses(sm, loadTum(raw_tum));
  std::unordered_map<std::uint64_t, SE3> p;
  for (const auto& [id, pose] : base) p[id + off] = pose;
  return p;
}

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
  CostmapParams cp;
  cp.occupancy = true;
  std::string raw_tum, map2, depth2, raw_tum2;  // --raw-tum / --map2 / --depth2 / --raw-tum2
  for (int i = 4; i < argc; ++i) {
    const std::string s = argv[i];
    if (s.rfind("--raw-tum=", 0) == 0) raw_tum = s.substr(10);
    else if (s.rfind("--map2=", 0) == 0) map2 = s.substr(7);
    else if (s.rfind("--depth2=", 0) == 0) depth2 = s.substr(9);
    else if (s.rfind("--raw-tum2=", 0) == 0) raw_tum2 = s.substr(11);
    else if (i == 4) vp.voxel_size_m = std::stod(s);
    else if (i == 5) cp.slice_height_m = std::stod(s);
  }
  cp.resolution_m = vp.voxel_size_m;

  std::vector<SubMap> submaps;
  if (!loadSubMaps(submaps, submap_dir)) {
    std::cerr << "FAIL: could not load submap archive: " << submap_dir << "\n";
    return 1;
  }

  VolumetricMapper mapper(std::make_unique<NvbloxBackend>(vp), vp);
  if (!mapper.backendAvailable())
    std::cerr << "WARN: nvblox backend not built (-DSLAMKO_WITH_NVBLOX=OFF) — "
                 "costmap will be EMPTY (CUDA-free smoke)\n";

  // Session 1.
  auto poses = sessionPoses(submaps, raw_tum, 0);
  std::size_t loaded = loadDepthInto(mapper, depth_dir, 0);
  std::cout << "session1 " << (raw_tum.empty() ? "CORRECTED" : "RAW") << ": "
            << submaps.size() << " submaps, " << poses.size() << " poses, "
            << loaded << " depth frames\n";

  // Optional session 2 fused INTO THE SAME volume (revisit/combined map). kf_ids
  // are offset so the two sessions don't collide; both sets of depth re-integrate
  // into one TSDF — revisited voxels get the weighted average of BOTH sessions.
  if (!map2.empty()) {
    const std::uint64_t OFF = 1000000000ull;
    std::vector<SubMap> sm2;
    if (loadSubMaps(sm2, map2)) {
      auto p2 = sessionPoses(sm2, raw_tum2, OFF);
      for (const auto& [k, v] : p2) poses[k] = v;
      const std::size_t l2 = loadDepthInto(mapper, depth2, OFF);
      loaded += l2;
      std::cout << "session2 " << (raw_tum2.empty() ? "CORRECTED" : "RAW") << ": +"
                << sm2.size() << " submaps, +" << p2.size() << " poses, +" << l2
                << " depth frames (one volume)\n";
    } else {
      std::cerr << "WARN: could not load session2 archive: " << map2 << "\n";
    }
  }

  const std::size_t integrated = mapper.reintegrate(poses);
  std::cout << "integrated " << integrated << "/" << mapper.numFrames()
            << " frames into one TSDF\n";

  const CostmapSlice cm = mapper.exportCostmap(cp);
  if (writeOccupancyMap(cm, out_prefix))
    std::cout << "wrote " << out_prefix << ".pgm/.yaml  (" << cm.width << "x"
              << cm.height << " @ " << cm.resolution << " m)\n";
  else
    std::cout << "no costmap written (empty slice)\n";

  mapper.exportMesh(out_prefix + ".ply");
  return 0;
}
