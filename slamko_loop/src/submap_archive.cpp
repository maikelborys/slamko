// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// SubMapArchive — see header. Pure bookkeeping over slamko_core::SubMap; no math
// beyond holding an SE3 anchor.

#include "slamko_loop/submap_archive.hpp"

#include <algorithm>

namespace slamko {

SubMapArchive::SubMapArchive() {
  active_.id = next_id_++;        // id 0
  active_.anchor = SE3();         // identity: active-local == odom frame until welded
}

void SubMapArchive::setActiveContent(SubMap content) {
  // Keep the archive-owned id + anchor; take only the estimator's content.
  content.id = active_.id;
  content.anchor = active_.anchor;
  active_ = std::move(content);
}

std::uint64_t SubMapArchive::seal() {
  const std::uint64_t id = active_.id;
  sealed_.push_back(active_);     // frozen copy (append-only)
  return id;
}

std::uint64_t SubMapArchive::branch() {
  SubMap fresh;
  fresh.id = next_id_++;
  fresh.anchor = SE3();           // fresh origin
  active_ = std::move(fresh);
  return active_.id;
}

const SubMap* SubMapArchive::find(std::uint64_t id) const {
  if (active_.id == id) return &active_;
  for (const auto& s : sealed_)
    if (s.id == id) return &s;
  return nullptr;
}

void SubMapArchive::setAnchor(std::uint64_t id, const SE3& anchor) {
  if (active_.id == id) { active_.anchor = anchor; return; }
  for (auto& s : sealed_)
    if (s.id == id) { s.anchor = anchor; return; }
}

void SubMapArchive::seedPriorMap(std::vector<SubMap> priors) {
  std::uint64_t maxid = 0;
  for (auto& p : priors) {
    maxid = std::max(maxid, p.id);
    sealed_.push_back(std::move(p));
  }
  next_id_ = std::max<std::uint64_t>(next_id_, maxid + 1);
  active_.id = next_id_++;   // live submap id past the priors (no collision)
  active_.anchor = SE3();    // fresh local origin == odom (welds re-anchor it)
}

}  // namespace slamko
