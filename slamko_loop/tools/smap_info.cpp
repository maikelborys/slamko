// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// smap_info — load a sealed-submap archive and report its lifelong-SLAM substrate:
// per-submap KF/landmark/descriptor counts, anchor, and the CRITICAL metric — VPR
// (per-KF global_descriptor) COVERAGE. Low coverage ⇒ loop closure cannot recall the
// return room regardless of geometry, so this is the hard gate before the offline
// loop-closure driver (the biggest documented risk: XFeat-64 has no place signal).
//
//   smap_info <smap_dir>

#include <cstdio>
#include <string>
#include <vector>

#include "slamko_core/submap_io.hpp"

int main(int argc, char** argv) {
  if (argc < 2) { std::printf("usage: smap_info <smap_dir>\n"); return 2; }
  const std::string dir = argv[1];

  std::vector<slamko::SubMap> maps;
  if (!slamko::loadSubMaps(maps, dir)) {
    std::printf("FAIL: loadSubMaps('%s') returned false (missing manifest or corrupt)\n", dir.c_str());
    return 1;
  }
  std::printf("loaded %zu submaps from %s\n", maps.size(), dir.c_str());

  std::size_t tot_kf = 0, tot_lm = 0, tot_desc = 0, kf_with_vpr = 0, tot_kfobs = 0;
  for (const auto& m : maps) {
    std::size_t vpr = 0;
    for (const auto& o : m.kf_obs) if (o.hasGlobalDescriptor()) ++vpr;
    const auto& a = m.anchor.translation();
    std::printf("  submap %llu: %zu KF, %zu lm, %dx%d desc, anchor_t=[%.2f %.2f %.2f], VPR %zu/%zu KF-obs\n",
                (unsigned long long)m.id, m.keyframes.size(), m.landmarks.size(),
                (int)m.descriptors.rows(), (int)m.descriptors.cols(),
                a.x(), a.y(), a.z(), vpr, m.kf_obs.size());
    tot_kf += m.keyframes.size();
    tot_lm += m.landmarks.size();
    tot_desc += static_cast<std::size_t>(m.descriptors.rows());
    kf_with_vpr += vpr;
    tot_kfobs += m.kf_obs.size();
  }
  const double cov = tot_kfobs ? 100.0 * static_cast<double>(kf_with_vpr) / tot_kfobs : 0.0;
  std::printf("TOTAL: %zu submaps · %zu KF · %zu landmarks · %zu descriptor rows\n",
              maps.size(), tot_kf, tot_lm, tot_desc);
  std::printf("VPR COVERAGE (hard gate): %zu/%zu KF-obs = %.1f%%  %s\n",
              kf_with_vpr, tot_kfobs, cov,
              cov >= 80.0 ? "[OK — loop recall viable]"
                          : "[LOW — loop closure will likely FAIL on recall, fix the VPR dump first]");
  return 0;
}
