// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// NeverLostSupervisor — see header for the state machine + the map→odom
// convention. No solver: the weld is one exact SE3 composition.

#include "slamko_loop/never_lost_supervisor.hpp"

#include <algorithm>

namespace slamko {

NeverLostSupervisor::NeverLostSupervisor(const SupervisorConfig& cfg,
                                         Relocalizer* reloc)
    : cfg_(cfg), reloc_(reloc), gate_(cfg) {}

bool NeverLostSupervisor::attemptWeld(RecoveryAction& act) {
  if (!reloc_ || !have_query_) return false;
  const RelocResult r = reloc_->relocalize(query_);
  if (!r.found) return false;

  // RelocResult.T_query_match is the query BODY pose in sealed-local (absolute).
  // Compose with the live odom to get the active→sealed weld constraint
  // (OKVIS2-X: T_AB = T_AS_query · T_WS_current⁻¹). The gate clusters THIS — a
  // frame transform invariant over the short reloc window even as odom moves.
  WeldCandidate c;
  c.sealed_submap_id = r.submap_id;
  c.T_active_sealed  = r.T_query_match * odom_T_WB_.inverse();
  c.confidence       = r.confidence;
  c.inliers          = r.num_inliers;

  SE3 consensus;
  std::uint64_t sealed_id = 0;
  if (!gate_.tryCluster(c, consensus, sealed_id)) return false;

  const SubMap* s = archive_.find(sealed_id);
  if (!s) return false;  // welds only to a known sealed submap

  // Weld-once-per-target: the clustered consensus is already an average, so a second
  // weld to the SAME sealed map this episode adds nothing (and a duplicate graph edge).
  // Welding to a different sealed map is still allowed.
  if (cfg_.weld_once_per_target) {
    for (const auto id : episode_welded_ids_)
      if (id == sealed_id) return false;
  }

  if (!cfg_.use_pose_graph) {
    // v1 closed form: map→odom = active.anchor = S.anchor * (active→sealed).
    archive_.setAnchor(archive_.activeId(), s->anchor * consensus);
  } else if (cfg_.auto_seal_distance_m <= 0.0) {
    // P2.5 incremental (loss-seal merge, e.g. V1_01) — UNCHANGED. Record the weld as a
    // graph edge and re-solve. Single fixed sealed node + one edge ⇒ reduces
    // algebraically to the closed form above (asserted in tests).
    if (!graph_.hasNode(s->id))
      graph_.setNode(s->id, s->anchor, graph_.nodeCount() == 0);  // first node = gauge
    if (!graph_.hasNode(archive_.activeId()))
      graph_.setNode(archive_.activeId(), s->anchor * consensus, false);  // warm start
    PoseGraphEdge e;
    e.from_id = s->id;
    e.to_id = archive_.activeId();
    e.T_from_to = consensus;  // measured anchor_sealed⁻¹ · anchor_active
    graph_.addEdge(e);
    graph_.optimize();
    for (const auto& sm : archive_.sealed())
      if (graph_.hasNode(sm.id)) archive_.setAnchor(sm.id, graph_.node(sm.id));
    archive_.setAnchor(archive_.activeId(), graph_.node(archive_.activeId()));
  } else {
    // CHAIN DISTRIBUTION (auto-seal long traversal). A loop closure on a clean
    // traversal must distribute its correction across the WHOLE submap chain, not just
    // re-anchor the active submap. We model the chain explicitly: every sealed submap
    // is a node; consecutive submaps are tied by an IDENTITY sequential edge (they
    // coincide in the shared continuous odom frame absent a loop); the first sealed
    // submap is the fixed gauge (the start IS the reference). Each loop closure is a
    // stored edge (deduped by from/to). Rebuilding from identity + all edges and
    // running ONE optimize() spreads every closure smoothly along the chain (GN on the
    // SE3 manifold), bending the drifted trajectory back. Disposable graph (Rule #4):
    // a bad edge is dropped by the optimizer's outlier guard, never crashes the tracker.
    PoseGraphEdge le;
    le.from_id = s->id;
    le.to_id = archive_.activeId();
    le.T_from_to = consensus;
    bool replaced = false;
    for (auto& e : loop_edges_)
      if (e.from_id == le.from_id && e.to_id == le.to_id) { e = le; replaced = true; break; }
    if (!replaced) loop_edges_.push_back(le);

    graph_.clear();
    std::uint64_t prev = 0;
    bool have_prev = false;
    for (const auto& sm : archive_.sealed()) {
      graph_.setNode(sm.id, SE3(), /*fixed=*/!have_prev);  // identity init; first = gauge
      if (have_prev) {
        PoseGraphEdge se;                       // sequential: consecutive ≡ identity
        se.from_id = prev; se.to_id = sm.id;
        graph_.addEdge(se);
      }
      prev = sm.id; have_prev = true;
    }
    graph_.setNode(archive_.activeId(), SE3(), false);
    if (have_prev) {
      PoseGraphEdge se; se.from_id = prev; se.to_id = archive_.activeId();
      graph_.addEdge(se);
    }
    for (const auto& e : loop_edges_)
      if (graph_.hasNode(e.from_id) && graph_.hasNode(e.to_id)) graph_.addEdge(e);
    graph_.optimize({});
    for (const auto& sm : archive_.sealed())
      if (graph_.hasNode(sm.id)) archive_.setAnchor(sm.id, graph_.node(sm.id));
    archive_.setAnchor(archive_.activeId(), graph_.node(archive_.activeId()));
  }
  episode_welded_ids_.push_back(sealed_id);
  act.welded = true;
  act.welded_to_id = sealed_id;
  act.applied_T_active_sealed = consensus;
  return true;
}

RecoveryAction NeverLostSupervisor::step(const HealthSignal& h,
                                         const EstimationFrame& odom,
                                         double monotonic_now_s) {
  last_now_s_ = monotonic_now_s;
  odom_T_WB_ = odom.T_WB;  // latest live body pose — used to compose the weld
  RecoveryAction act;
  const double gap = h.odom_stale_gap_s;

  // Track odometry distance travelled since the last seal (for auto-sealing).
  if (have_last_odom_t_)
    dist_since_seal_ += (odom.T_WB.translation() - last_odom_t_).norm();
  last_odom_t_ = odom.T_WB.translation();
  have_last_odom_t_ = true;

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
        recover_count_ = 0;
        episode_welded_ = false;  // fresh recovery episode
        episode_welded_ids_.clear();
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
    // Periodic AUTO-SEAL (OK only). Voluntarily checkpoint the active submap into the
    // sealed set every auto_seal_distance_m of travel so loop closure has a target to
    // match on revisit (the relocalizer DB only holds sealed submaps). Unlike a loss
    // seal we stay OK and the branch INHERITS the sealed anchor, so map→odom is
    // continuous (no jump — nothing actually went wrong). The node registers the
    // sealed submap with the relocalizer + begins a fresh VIO epoch on act.sealed/branched.
    if (cfg_.auto_seal_distance_m > 0.0 && state_ == SupervisorState::OK &&
        dist_since_seal_ >= cfg_.auto_seal_distance_m) {
      const SE3 prev_anchor = archive_.active().anchor;
      act.sealed   = true;  act.sealed_id   = archive_.seal();
      act.branched = true;  act.branched_id = archive_.branch();
      archive_.setAnchor(archive_.activeId(), prev_anchor);  // seamless continuation
      dist_since_seal_ = 0.0;
      // Fresh weld-once scope per ACTIVE submap. weld_once_per_target is keyed on the
      // sealed target; in continuous OK mode the FIRST active welds to the start submap
      // trivially (near-zero correction, no drift yet) and would then block the REAL
      // loop closure when a LATER active revisits that same start submap. Clearing on
      // each branch makes weld-once per (target, active) — bounded edges, but every
      // genuine revisit by a new submap can still close the loop.
      episode_welded_ids_.clear();
    }

    // P4b-2: continuous relocalization. While healthy (OK), periodically try to weld
    // the live submap to a prior/sealed one — localize into a prior map or close a
    // loop without getting lost. Gate-guarded; updates map→odom only, stays OK.
    if (cfg_.continuous_reloc && state_ == SupervisorState::OK &&
        (++cont_counter_ % std::max(1, cfg_.continuous_reloc_interval)) == 0) {
      attemptWeld(act);
    }
    return act;
  }

  // state_ == Lost / Relocalizing: keep attempting the weld. We do NOT exit to OK
  // on healthy odom alone — the re-acquired vision after the blackout is exactly
  // what lets the branch re-anchor to the sealed map on revisit, so we stay here
  // until WELDED (then a short dwell) or until we give up re-anchoring.
  if (attemptWeld(act)) episode_welded_ = true;

  if (gap <= cfg_.recently_lost_gap_s) ++recover_count_;
  else recover_count_ = 0;

  const bool recovered =
      (episode_welded_ && recover_count_ >= cfg_.recover_dwell_frames) ||
      (recover_count_ >= cfg_.reloc_give_up_frames);  // gave up re-anchoring
  if (recovered) {
    state_ = SupervisorState::OK;
    recover_count_ = 0;
    gate_.reset();
  }
  return act;
}

}  // namespace slamko
