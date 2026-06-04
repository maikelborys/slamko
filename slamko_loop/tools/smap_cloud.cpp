// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// smap_cloud — export the global point cloud of a sealed-submap archive: each
// submap's landmarks composed to the global frame via its anchor (anchor·local).
// Decimated by --stride to keep the CSV light for viz. Optionally re-pose submaps
// from a corrected-KF .tum later; v1 dumps the as-sealed (baseline) cloud.
//
//   smap_cloud <smap_dir> <out.csv> [stride=20]

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "slamko_core/submap_io.hpp"

int main(int argc, char** argv) {
  if (argc < 3) { std::printf("usage: smap_cloud <smap_dir> <out.csv> [stride]\n"); return 2; }
  const std::string dir = argv[1], out = argv[2];
  const int stride = argc > 3 ? std::max(1, std::atoi(argv[3])) : 20;

  std::vector<slamko::SubMap> maps;
  if (!slamko::loadSubMaps(maps, dir)) { std::printf("FAIL load %s\n", dir.c_str()); return 1; }

  std::ofstream o(out);
  o << "x,y,z,submap\n";
  std::size_t n = 0, written = 0;
  for (const auto& m : maps) {
    for (const auto& lm : m.landmarks) {
      if ((n++ % static_cast<std::size_t>(stride)) != 0) continue;
      const Eigen::Vector3d p = m.anchor * lm.position;  // submap-local → global
      o << p.x() << ',' << p.y() << ',' << p.z() << ',' << m.id << '\n';
      ++written;
    }
  }
  std::printf("wrote %zu / %zu landmarks (stride %d) -> %s\n", written, n, stride, out.c_str());
  return 0;
}
