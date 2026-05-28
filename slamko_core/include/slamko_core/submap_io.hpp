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
// Format (binary, little-endian). Magic versions:
//   SMP1: id · anchor(quat+t) · keyframes (id,ts,pose) ·
//         landmarks (id,xyz,descriptor_row) · descriptor block (N×D float, row-major).
//   SMP2 (additive): + trailing global (VPR) descriptor — gdim · gdim floats.
//   SMP3 (additive): + per-keyframe observations block — nk per-KF entries each as
//         N · landmark_ids[N] (uint64) · uv[N×2] (float) · have_right (uint8) ·
//         uv_right[N×2] (float, only when have_right). Empty (N=0) when the KF
//         carries no observations. Enables global landmark BA + real two-image
//         LightGlue (see docs/PLAN_BA_GLOBAL.md).
//   SMP4 (additive): per-keyframe block gains a trailing per-KF VPR descriptor —
//         kf_gdim (uint64) · kf_gdim floats (L2-normalized, 0 if absent). Finer-grain
//         VPR retrieval (each KF, not just one per submap) for hard revisits — the
//         magistrale-return fix.
//   SMP5 (additive): per-keyframe block gains a trailing IMU window —
//         nimu (uint64) · nimu × (ts:double, ax,ay,az:double, gx,gy,gz:double) — the
//         IMU samples between the previous KF and this KF. Substrate for global BA
//         with CombinedImuFactor (Phase B.2, see docs/PLAN_BA_GLOBAL.md). First KF
//         of a submap stores nimu=0 (no previous KF in this submap). Legacy maps
//         (SMP1–SMP4) load with imu_since_prev empty.
//
// Loads accept all five versions; older formats leave the newer fields empty (the
// downstream backends gate on .empty()). Same-architecture assumption (x86 robot +
// dev box); a portable fixed-width codec is a later refinement. CustomData (dense
// payloads) is NOT serialized here — that is a per-payload concern (TSDF slab,
// etc.), opt-in later.
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
  // SMP5 adds per-keyframe IMU window inside each kf_obs block (the BA substrate for
  // CombinedImuFactor in the global smoother); SMP1–SMP4 still load (newer fields
  // stay empty).
  f.write("SMP5", 4);
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

  // SMP3: per-keyframe 2D observations. nk_obs blocks; nk_obs may be 0 (legacy) or
  // == sm.keyframes.size() (aligned 1:1). Each block: N · landmark_ids[N] (uint64) ·
  // uv[N×2] (float, row-major) · have_right (uint8) · uv_right[N×2] (only when set).
  // SMP4 trailer per block: kf_gdim (uint64) · kf_gdim floats (per-KF VPR vector,
  // 0 if absent).
  const std::uint64_t nk_obs = static_cast<std::uint64_t>(sm.kf_obs.size());
  wr(f, nk_obs);
  for (const auto& ko : sm.kf_obs) {
    const std::uint64_t N = static_cast<std::uint64_t>(ko.landmark_ids.size());
    wr(f, N);
    if (N) {
      f.write(reinterpret_cast<const char*>(ko.landmark_ids.data()),
              sizeof(std::uint64_t) * N);
      f.write(reinterpret_cast<const char*>(ko.uv.data()),
              sizeof(float) * N * 2);
    }
    const std::uint8_t have_right = ko.hasStereo() ? 1 : 0;
    wr(f, have_right);
    if (have_right && N)
      f.write(reinterpret_cast<const char*>(ko.uv_right.data()),
              sizeof(float) * N * 2);
    // SMP4: per-KF VPR descriptor (kf_gdim · floats, 0 if absent).
    const std::uint64_t kf_gdim = static_cast<std::uint64_t>(ko.global_descriptor.size());
    wr(f, kf_gdim);
    if (kf_gdim)
      f.write(reinterpret_cast<const char*>(ko.global_descriptor.data()),
              sizeof(float) * kf_gdim);
    // SMP5: per-KF IMU window (nimu samples between prev KF and this KF). Each sample
    // is 7 doubles: ts, accel(x,y,z), gyro(x,y,z). The first KF of a submap stores 0
    // (no previous KF in this submap). The codec is structurally independent of the
    // IMU dt rate.
    const std::uint64_t nimu = static_cast<std::uint64_t>(ko.imu_since_prev.size());
    wr(f, nimu);
    for (const auto& s : ko.imu_since_prev) {
      double rec[7] = {s.timestamp,
                       s.accel.x(), s.accel.y(), s.accel.z(),
                       s.gyro.x(),  s.gyro.y(),  s.gyro.z()};
      f.write(reinterpret_cast<const char*>(rec), sizeof(rec));
    }
  }
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
  if (ver != "SMP1" && ver != "SMP2" && ver != "SMP3" && ver != "SMP4" && ver != "SMP5") return false;
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
  sm.kf_obs.clear();
  if (ver == "SMP2" || ver == "SMP3" || ver == "SMP4" || ver == "SMP5") {  // SMP1 leaves global_descriptor empty
    std::uint64_t gdim = 0;
    rd(f, gdim);
    if (gdim) {
      sm.global_descriptor.resize(gdim);
      f.read(reinterpret_cast<char*>(sm.global_descriptor.data()),
             sizeof(float) * gdim);
    }
  }
  if (ver == "SMP3" || ver == "SMP4" || ver == "SMP5") {  // SMP1/SMP2 leave kf_obs empty (BA substrate absent)
    std::uint64_t nk_obs = 0;
    rd(f, nk_obs);
    sm.kf_obs.resize(nk_obs);
    for (auto& ko : sm.kf_obs) {
      std::uint64_t N = 0;
      rd(f, N);
      ko.landmark_ids.resize(N);
      ko.uv.resize(N, 2);
      if (N) {
        f.read(reinterpret_cast<char*>(ko.landmark_ids.data()),
               sizeof(std::uint64_t) * N);
        f.read(reinterpret_cast<char*>(ko.uv.data()), sizeof(float) * N * 2);
      }
      std::uint8_t have_right = 0;
      rd(f, have_right);
      if (have_right && N) {
        ko.uv_right.resize(N, 2);
        f.read(reinterpret_cast<char*>(ko.uv_right.data()),
               sizeof(float) * N * 2);
      }
      if (ver == "SMP4" || ver == "SMP5") {  // SMP3 leaves per-KF VPR descriptor empty
        std::uint64_t kf_gdim = 0;
        rd(f, kf_gdim);
        if (kf_gdim) {
          ko.global_descriptor.resize(kf_gdim);
          f.read(reinterpret_cast<char*>(ko.global_descriptor.data()),
                 sizeof(float) * kf_gdim);
        }
      }
      if (ver == "SMP5") {  // SMP1–SMP4 leave per-KF IMU window empty
        std::uint64_t nimu = 0;
        rd(f, nimu);
        ko.imu_since_prev.resize(nimu);
        for (auto& s : ko.imu_since_prev) {
          double rec[7];
          f.read(reinterpret_cast<char*>(rec), sizeof(rec));
          s.timestamp = rec[0];
          s.accel = Eigen::Vector3d(rec[1], rec[2], rec[3]);
          s.gyro  = Eigen::Vector3d(rec[4], rec[5], rec[6]);
        }
      }
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
