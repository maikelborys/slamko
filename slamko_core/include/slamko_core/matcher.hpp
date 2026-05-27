// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Matcher — two-view descriptor matching for relocalization / loop closure
// (NOT the hot path; KLT flow handles frame-to-frame). LighterGlue (libtorch)
// is the robust default; MNN + Lowe ratio (cuBLAS cosine) is the fast fallback.
// Both native 64-d. Concrete impls live in slamko_loop (optional torch target)
// behind this Eigen-only contract.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "slamko_core/features.hpp"

namespace slamko {

struct DescMatch {
  std::uint32_t query_idx = 0;  // row in query Features
  std::uint32_t train_idx = 0;  // row in train Features
  float score = 0.0f;
};

class Matcher {
 public:
  virtual ~Matcher() = default;
  virtual std::string name() const = 0;

  // Match query descriptors against train descriptors. Both must carry
  // descriptors of equal dim (hasDescriptors()).
  virtual std::vector<DescMatch> match(const Features& query,
                                       const Features& train) = 0;
};

}  // namespace slamko
