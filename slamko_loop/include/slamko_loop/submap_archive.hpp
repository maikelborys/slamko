// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// SubMapArchive — the multi-map "Atlas" and the archive-don't-discard store. One
// ACTIVE submap is grown by the live estimator; on tracking loss the supervisor
// seals it (frozen, append-only) and branches a fresh active one — so a corrupted
// post-loss estimate can never poison the sealed map. The archive OWNS each
// submap's id and `anchor` (submap-local → global/map); the estimator only
// provides content. Post-seal a submap is immutable EXCEPT its `anchor` — the
// documented sole cross-submap coupling (submap.hpp), which the weld updates.

#pragma once

#include <cstdint>
#include <vector>

#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"

namespace slamko {

class SubMapArchive {
 public:
  SubMapArchive();  // one active submap, id 0, anchor = identity

  std::uint64_t  activeId() const { return active_.id; }
  const SubMap&  active() const { return active_; }

  // Ingest the estimator's assembled content into the active submap, KEEPING the
  // archive-owned id + anchor (the estimator's submap.id/anchor are ignored).
  void setActiveContent(SubMap content);

  // SEAL: freeze a copy of the active submap into the append-only sealed set.
  // Returns the sealed id. The active submap is left intact (call branch() to
  // start fresh) — seal+branch are separate so the supervisor controls ordering.
  std::uint64_t seal();

  // BRANCH: replace the active submap with a fresh one (new id, identity anchor =
  // a fresh local origin == the odom frame, see never_lost_supervisor map→odom).
  std::uint64_t branch();

  const std::vector<SubMap>& sealed() const { return sealed_; }
  std::size_t sealedCount() const { return sealed_.size(); }

  // Find a submap by id among sealed + active; nullptr if absent.
  const SubMap* find(std::uint64_t id) const;

  // Set a submap's anchor (the only legal post-seal mutation — the weld uses it).
  void setAnchor(std::uint64_t id, const SE3& anchor);

  // Cross-session: seed the archive with a PRIOR map's submaps (loaded from disk) as
  // frozen sealed submaps — they keep their ids + anchors (session-1 frame). The live
  // active submap is re-id'd to a fresh id PAST the priors so in-session seals never
  // collide. This is what makes the Atlas grow across sessions (the prior map is just
  // more sealed submaps; the same weld machinery localizes the live session into it).
  void seedPriorMap(std::vector<SubMap> priors);

 private:
  std::vector<SubMap> sealed_;   // append-only, frozen priors
  SubMap              active_;
  std::uint64_t       next_id_ = 0;
};

}  // namespace slamko
