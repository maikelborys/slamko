// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
#pragma once

#include <cstdint>
#include <vector>

namespace slamko_vio {

struct Keypoint {
  float x;
  float y;
  float score;
};

enum class TrackStatus : int8_t {
  Lost = 0,
  Ok   = 1,
};

struct Track {
  uint32_t id;
  Keypoint kp;
  TrackStatus status;
  uint32_t age;
};

struct StereoIntrinsics {
  float fx, fy, cx, cy;
  float baseline_m;
};

}
