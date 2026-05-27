// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// FeatureTracker — frame-to-frame association, the second swappable seam.
// Default is KLT optical flow (µs, the klt_vo hot path); a descriptor-match
// tracker is the fast-motion fallback (doc 13 option c). Decoupling detect ⊥
// track lets us run "XFeat detect + KLT track" (the doc-13 sweet spot) without
// touching PnP/BA.

#pragma once

#include <string>
#include <vector>

#include "slamko_core/features.hpp"
#include "slamko_core/image_view.hpp"

namespace slamko {

class FeatureTracker {
 public:
  virtual ~FeatureTracker() = default;
  virtual std::string name() const = 0;

  // Advance the existing tracks from `prev` to `curr` image, seeding new tracks
  // from `detected` where tracks were lost. Returns the live tracks in `curr`.
  virtual std::vector<Track> track(const ImageView& prev, const ImageView& curr,
                                   const Features& detected) = 0;

  // Drop all state (re-init on tracking loss / new submap).
  virtual void reset() = 0;
};

}  // namespace slamko
