// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Relocalizer — the long-loss / kidnap recovery contract (MASTER_PLAN §5,
// timescale 3). Place-recognition (default LiftFeat-m1 + DBoW) over archived
// submaps; on a hit it returns ONE relative-pose weld constraint. Runs decoupled
// from the estimator graph — the never-lost supervisor (slamko_loop) consumes
// the result and gates the merge OUTSIDE the Ceres/GTSAM solve, because tight
// in-graph coupling "pendulates" (the user's OKVIS2-X finding).

#pragma once

#include <cstdint>
#include <string>

#include "slamko_core/features.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"

namespace slamko {

struct RelocResult {
  bool found = false;
  std::uint64_t submap_id = 0;   // the matched archived submap
  SE3 T_query_match;             // relative pose query -> matched (weld constraint)
  double confidence = 0.0;       // place-recognition score
  int num_inliers = 0;           // geometric-verification inliers (PnP RANSAC)
};

class Relocalizer {
 public:
  virtual ~Relocalizer() = default;
  virtual std::string name() const = 0;

  // Register an archived submap into the searchable descriptor database.
  virtual void addSubMap(const SubMap& submap) = 0;

  // Query with the current frame's features; best match (found=false if none).
  virtual RelocResult relocalize(const Features& query) const = 0;
};

}  // namespace slamko
