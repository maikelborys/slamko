// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// depth_io.hpp — a tiny binary file format for a per-keyframe DepthFrame, the
// handoff between the depth producer (HITNet/ESS, Python) and the C++ export
// driver. One file per keyframe. Header-only, little-endian, same-arch (x86),
// matching the submap_io.hpp convention. The world POSE is never stored here —
// it comes from the (corrected) submap archive at integration time (the bend).
//
// Layout:
//   char    magic[4]   = "SKDF"
//   int32   version    = 1
//   uint64  kf_id
//   int32   width, height
//   double  fx, fy, cx, cy            (depth-camera intrinsics)
//   double  T_body_cam[16]            (4x4 row-major, camera-in-body extrinsic)
//   float32 depth[width*height]       (metres, row-major, 0 = invalid)

#pragma once

#include <cstdint>
#include <fstream>
#include <string>

#include "slamko_core/volumetric_map.hpp"

namespace slamko {

namespace detail {
template <typename T>
inline void wr(std::ofstream& o, const T& v) {
  o.write(reinterpret_cast<const char*>(&v), sizeof(T));
}
template <typename T>
inline void rd(std::ifstream& i, T& v) {
  i.read(reinterpret_cast<char*>(&v), sizeof(T));
}
}  // namespace detail

inline bool saveDepthFrame(const DepthFrame& f, const std::string& path) {
  if (!f.valid()) return false;
  std::ofstream o(path, std::ios::binary);
  if (!o) return false;
  o.write("SKDF", 4);
  detail::wr<std::int32_t>(o, 1);
  detail::wr<std::uint64_t>(o, f.kf_id);
  detail::wr<std::int32_t>(o, f.width);
  detail::wr<std::int32_t>(o, f.height);
  detail::wr<double>(o, f.K.fx);
  detail::wr<double>(o, f.K.fy);
  detail::wr<double>(o, f.K.cx);
  detail::wr<double>(o, f.K.cy);
  const Eigen::Matrix4d M = f.T_body_cam.matrix();
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) detail::wr<double>(o, M(r, c));
  o.write(reinterpret_cast<const char*>(f.depth.data()),
          static_cast<std::streamsize>(f.depth.size() * sizeof(float)));
  return static_cast<bool>(o);
}

inline bool loadDepthFrame(DepthFrame& f, const std::string& path) {
  std::ifstream i(path, std::ios::binary);
  if (!i) return false;
  char magic[4];
  i.read(magic, 4);
  if (std::string(magic, 4) != "SKDF") return false;
  std::int32_t version = 0;
  detail::rd(i, version);
  if (version != 1) return false;
  detail::rd(i, f.kf_id);
  detail::rd(i, f.width);
  detail::rd(i, f.height);
  detail::rd(i, f.K.fx);
  detail::rd(i, f.K.fy);
  detail::rd(i, f.K.cx);
  detail::rd(i, f.K.cy);
  f.K.width = f.width;
  f.K.height = f.height;
  Eigen::Matrix4d M;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) detail::rd(i, M(r, c));
  f.T_body_cam = SE3(M);
  if (f.width <= 0 || f.height <= 0) return false;
  f.depth.resize(static_cast<std::size_t>(f.width) * f.height);
  i.read(reinterpret_cast<char*>(f.depth.data()),
         static_cast<std::streamsize>(f.depth.size() * sizeof(float)));
  return static_cast<bool>(i) && f.valid();
}

}  // namespace slamko
