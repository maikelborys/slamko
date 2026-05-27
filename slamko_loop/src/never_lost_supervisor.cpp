// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// NeverLostSupervisor — see header for the state machine + the map→odom
// convention. No solver: the weld is one exact SE3 composition.

#include "slamko_loop/never_lost_supervisor.hpp"

namespace slamko {

NeverLostSupervisor::NeverLostSupervisor(const SupervisorConfig& cfg,
                                         Relocalizer* reloc)
    : cfg_(cfg), reloc_(reloc), gate_(cfg) {}

bool NeverLostSupervisor::attemptWeld(RecoveryAction& act) {
  if (!reloc_ || !have_query_) return false;
  const RelocResult r = reloc_->relocalize(query_);
  if (!r.found) return false;

  WeldCandidate c;
  c.sealed_submap_id = r.submap_id;
  c.T_active_sealed  = r.T_query_match;  // active(query) → sealed-local (contract)
  c.confidence       = r.confidence;
  c.inliers          = r.num_inliers;

  SE3 consensus;
  std::uint64_t sealed_id = 0;
  if (!gate_.tryCluster(c, consensus, sealed_id)) return false;

  const SubMap* s = archive_.find(sealed_id);
  if (!s) return false;  // welds only to a known sealed submap

  // map→odom = active.anchor = S.anchor * (active→sealed). Held until next weld.
  archive_.setAnchor(archive_.activeId(), s->anchor * consensus);
  act.welded = true;
  act.welded_to_id = sealed_id;
  act.applied_T_active_sealed = consensus;
  return true;
}

RecoveryAction NeverLostSupervisor::step(const HealthSignal& h,
                                         const EstimationFrame& /*odom*/,
                                         double monotonic_now_s) {
  last_now_s_ = monotonic_now_s;
  RecoveryAction act;
  const double gap = h.odom_stale_gap_s;

  if (state_ == SupervisorState::OK || state_ == SupervisorState::RecentlyLost) {
    if (gap > cfg_.lost_gap_s) {
      ++lost_count_;
      recover_count_ = 0;
      if (lost_count_ >= cfg_.lost_dwell_frames) {
        // Tracking lost for the full dwell → SEAL the (now-suspect) active map and
        // BRANCH a fresh one. Keep producing odom; map→odom resets to identity
        // (the branch IS the map until a weld re-anchors it).
        act.sealed = true;   act.sealed_id   = archive_.seal();
        act.branched = true; act.branched_id = archive_.branch();
        gate_.reset();
        state_ = SupervisorState::Relocalizing;
        lost_count_ = 0;
      } else {
        state_ = SupervisorState::RecentlyLost;  // climbing toward Lost
      }
    } else if (gap > cfg_.recently_lost_gap_s) {
      state_ = SupervisorState::RecentlyLost;     // VIO dead-reckoning coasts here
      lost_count_ = 0;
    } else {
      state_ = SupervisorState::OK;
      lost_count_ = 0;
    }
    return act;
  }

  // state_ == Lost / Relocalizing: try to re-anchor, and recover locally on
  // sustained healthy odom (the branch is tracking fine again).
  attemptWeld(act);

  if (gap <= cfg_.recently_lost_gap_s) {
    if (++recover_count_ >= cfg_.recover_dwell_frames) {
      state_ = SupervisorState::OK;
      recover_count_ = 0;
      gate_.reset();  // stop the recovery's relocalization attempts (loop-closure = P2.5)
    }
  } else {
    recover_count_ = 0;
  }
  return act;
}

}  // namespace slamko
