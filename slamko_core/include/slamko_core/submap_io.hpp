// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// SubMap (de)serialization — the map-persistence schema slamko_core owns. A SubMap
// is the unit of "archive-don't-discard", so it must survive to disk and reload into
// a fresh session's relocalizer: that is what makes the system cross-session
// recoverable (load a prior map → relocalize → weld the new session onto it). The
// submaps are CONNECTED via their anchors, never fused into one cloud (Hard Rule #4,
// the disposable global graph), so each one round-trips independently.
//
// Format (binary, little-endian, "SMP1"): id · anchor(quat+t) · keyframes
// (id,ts,pose) · landmarks (id,xyz,descriptor_row) · descriptor block (N×D float,
// row-major). Same-architecture assumption (x86 robot + dev box); a portable
// fixed-width codec is a later refinement. CustomData (dense payloads) is NOT
// serialized here — that is a per-payload concern (TSDF slab, etc.), opt-in later.
//
// Header-only to match the rest of slamko_core (INTERFACE lib). Eigen + std only.

#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"

namespace slamko {

namespace submap_io_detail {

template <class T>
inline void wr(std::ostream& o, const T& v) {
  o.write(reinterpret_cast<const char*>(&v), sizeof(T));
}
template <class T>
inline void rd(std::istream& i, T& v) {
  i.read(reinterpret_cast<char*>(&v), sizeof(T));
}

inline void wrSE3(std::ostream& o, const SE3& T) {
  const Eigen::Quaterniond q = T.so3().unit_quaternion();
  const Eigen::Vector3d t = T.translation();
  const double b[7] = {q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z()};
  o.write(reinterpret_cast<const char*>(b), sizeof(b));
}
inline SE3 rdSE3(std::istream& i) {
  double b[7];
  i.read(reinterpret_cast<char*>(b), sizeof(b));
  const Eigen::Quaterniond q(b[3], b[0], b[1], b[2]);  // (w, x, y, z)
  return SE3(SO3(q), Eigen::Vector3d(b[4], b[5], b[6]));
}

}  // namespace submap_io_detail

// Write one submap to `path`. Returns false on any I/O error.
inline bool saveSubMap(const SubMap& sm, const std::string& path) {
  using namespace submap_io_detail;
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f.write("SMP2", 4);  // SMP2 adds the trailing global (VPR) descriptor; SMP1 still loads
  wr(f, sm.id);
  wrSE3(f, sm.anchor);

  const std::uint64_t nk = sm.keyframes.size();
  wr(f, nk);
  for (const auto& k : sm.keyframes) {
    wr(f, k.id);
    wr(f, k.timestamp);
    wrSE3(f, k.T_WB);
  }

  const std::uint64_t nl = sm.landmarks.size();
  wr(f, nl);
  for (const auto& l : sm.landmarks) {
    wr(f, l.id);
    const double p[3] = {l.position.x(), l.position.y(), l.position.z()};
    f.write(reinterpret_cast<const char*>(p), sizeof(p));
    const std::int32_t dr = l.descriptor_row;
    wr(f, dr);
  }

  const std::uint64_t rows = static_cast<std::uint64_t>(sm.descriptors.rows());
  const std::uint64_t cols = static_cast<std::uint64_t>(sm.descriptors.cols());
  wr(f, rows);
  wr(f, cols);
  if (rows * cols)
    f.write(reinterpret_cast<const char*>(sm.descriptors.data()),
            sizeof(float) * rows * cols);

  // SMP2: trailing global (VPR) descriptor — gdim · gdim floats (gdim=0 if absent).
  const std::uint64_t gdim = static_cast<std::uint64_t>(sm.global_descriptor.size());
  wr(f, gdim);
  if (gdim)
    f.write(reinterpret_cast<const char*>(sm.global_descriptor.data()),
            sizeof(float) * gdim);
  return static_cast<bool>(f);
}

// Read one submap from `path`. Returns false on I/O error or a bad magic/version.
inline bool loadSubMap(SubMap& sm, const std::string& path) {
  using namespace submap_io_detail;
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  char magic[4];
  f.read(magic, 4);
  const std::string ver(magic, 4);
  if (ver != "SMP1" && ver != "SMP2") return false;
  rd(f, sm.id);
  sm.anchor = rdSE3(f);

  std::uint64_t nk = 0;
  rd(f, nk);
  sm.keyframes.resize(nk);
  for (auto& k : sm.keyframes) {
    rd(f, k.id);
    rd(f, k.timestamp);
    k.T_WB = rdSE3(f);
  }

  std::uint64_t nl = 0;
  rd(f, nl);
  sm.landmarks.resize(nl);
  for (auto& l : sm.landmarks) {
    rd(f, l.id);
    double p[3];
    f.read(reinterpret_cast<char*>(p), sizeof(p));
    l.position = Eigen::Vector3d(p[0], p[1], p[2]);
    std::int32_t dr = 0;
    rd(f, dr);
    l.descriptor_row = dr;
  }

  std::uint64_t rows = 0, cols = 0;
  rd(f, rows);
  rd(f, cols);
  sm.descriptors.resize(rows, cols);
  if (rows * cols)
    f.read(reinterpret_cast<char*>(sm.descriptors.data()),
           sizeof(float) * rows * cols);

  sm.global_descriptor.resize(0);
  if (ver == "SMP2") {  // SMP1 maps have no global descriptor → leave empty
    std::uint64_t gdim = 0;
    rd(f, gdim);
    if (gdim) {
      sm.global_descriptor.resize(gdim);
      f.read(reinterpret_cast<char*>(sm.global_descriptor.data()),
             sizeof(float) * gdim);
    }
  }
  return static_cast<bool>(f);
}

// Persist a whole archive into `dir` (created if absent): one `submap_<id>.smap`
// per map + a `submaps.manifest` listing ids in order. Returns false on error.
inline bool saveSubMaps(const std::vector<SubMap>& maps, const std::string& dir) {
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  std::ofstream mf(dir + "/submaps.manifest");
  if (!mf) return false;
  for (const auto& m : maps) {
    if (!saveSubMap(m, dir + "/submap_" + std::to_string(m.id) + ".smap"))
      return false;
    mf << m.id << "\n";
  }
  return static_cast<bool>(mf);
}

// Load an archive from `dir` (reads the manifest, then each submap). Returns false
// if the manifest or any listed submap is missing/corrupt.
inline bool loadSubMaps(std::vector<SubMap>& maps, const std::string& dir) {
  std::ifstream mf(dir + "/submaps.manifest");
  if (!mf) return false;
  maps.clear();
  std::uint64_t id = 0;
  while (mf >> id) {
    SubMap m;
    if (!loadSubMap(m, dir + "/submap_" + std::to_string(id) + ".smap"))
      return false;
    maps.push_back(std::move(m));
  }
  return true;
}

}  // namespace slamko
