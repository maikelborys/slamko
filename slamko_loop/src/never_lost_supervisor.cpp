// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// NeverLostSupervisor — see header for the state machine + the map→odom
// convention. No solver: the weld is one exact SE3 composition.

#include "slamko_loop/never_lost_supervisor.hpp"

#include <algorithm>
#include <unordered_map>

#include "slamko_core/global_smoother.hpp"

namespace slamko {

NeverLostSupervisor::NeverLostSupervisor(const SupervisorConfig& cfg,
                                         Relocalizer* reloc)
    : cfg_(cfg), reloc_(reloc), gate_(cfg) {}

namespace {
// Append a SubMap's KFs/landmarks/observations/IMU windows into a GlobalBAInput, with
// optional rigid pre-transform from submap-local → BA frame. Used by the pair-BA
// builder below: sealed submap is appended as-is (BA frame == sealed-local),
// active submap is appended with the weld-consensus transform applied so its KFs
// land in sealed-local frame. IMU samples stay body-frame (no transform needed —
// they're intrinsic to the body's motion, the BA preintegration uses them raw).
void appendSubmapToBA(const SubMap& sm, const SE3& T_BAframe_local,
                      GlobalBAInput& in) {
  for (const auto& kf : sm.keyframes)
    in.keyframes.emplace_back(kf.id, SE3(T_BAframe_local.matrix() * kf.T_WB.matrix()));
  for (const auto& l : sm.landmarks)
    in.landmarks.emplace_back(l.id, T_BAframe_local * l.position);
  for (std::size_t k = 0; k < sm.keyframes.size() && k < sm.kf_obs.size(); ++k) {
    const auto& ko = sm.kf_obs[k];
    const bool stereo = ko.hasStereo();
    for (int i = 0; i < ko.size(); ++i) {
      GlobalBAObservation o;
      o.kf_id = sm.keyframes[k].id;
      o.landmark_id = ko.landmark_ids[i];
      o.uv_left = ko.uv.row(i).transpose();
      if (stereo && std::isfinite(ko.uv_right(i, 0)))
        o.uv_right = ko.uv_right.row(i).transpose();
      in.observations.push_back(o);
    }
  }
}

// IMU windows + bootstrap velocity/bias for a submap. KF poses are taken from the
// BA-frame KF estimates already inserted (so finite-diff is in BA frame for sanity,
// though IMU samples are body-frame so the transform doesn't affect them).
void appendImuToBA(const SubMap& sm, GlobalBAInput& in) {
  bool any_imu = false;
  for (const auto& ko : sm.kf_obs) if (ko.hasImu()) { any_imu = true; break; }
  if (!any_imu || sm.keyframes.size() < 2) return;
  // Bootstrap velocity from KF translation finite-diff. Use BA-frame poses if they're
  // already in `in.keyframes`; otherwise fall back to the submap-local KF poses.
  std::unordered_map<std::uint64_t, Eigen::Vector3d> kf_t;
  for (const auto& kv : in.keyframes) kf_t[kv.first] = kv.second.translation();
  for (std::size_t k = 0; k < sm.keyframes.size(); ++k) {
    const auto id = sm.keyframes[k].id;
    Eigen::Vector3d v(0, 0, 0);
    const std::size_t kp = (k + 1 < sm.keyframes.size()) ? k + 1 : k;
    const std::size_t km = (k == 0) ? 0 : k - 1;
    if (kp != km) {
      const double dt = sm.keyframes[kp].timestamp - sm.keyframes[km].timestamp;
      auto itp = kf_t.find(sm.keyframes[kp].id);
      auto itm = kf_t.find(sm.keyframes[km].id);
      if (dt > 1e-6 && itp != kf_t.end() && itm != kf_t.end())
        v = (itp->second - itm->second) / dt;
    }
    in.velocities.emplace_back(id, v);
    in.biases.emplace_back(id, ImuBias{});
  }
  for (std::size_t k = 1; k < sm.keyframes.size() && k < sm.kf_obs.size(); ++k) {
    if (!sm.kf_obs[k].hasImu()) continue;
    GlobalBAImuWindow w;
    w.kf_from = sm.keyframes[k - 1].id;
    w.kf_to   = sm.keyframes[k].id;
    w.samples = sm.kf_obs[k].imu_since_prev;
    in.imu_windows.push_back(w);
  }
}

// C.live V1: pair-BA on weld + ANCHOR refinement. Builds a 2-submap (sealed + active)
// BA in SEALED-LOCAL frame with:
//   - sealed KFs/landmarks as-is (anchor_kf = sealed.keyframes.front()  → gauge)
//   - active KFs/landmarks pre-transformed by the weld consensus (active-local →
//     sealed-local), so the two halves co-exist in one frame.
//   - BetweenFactor on (sealed.last_kf, active.first_kf) derived from consensus —
//     the loop-closure constraint (the only link between the two halves; epoch-
//     disjoint submaps share no landmarks).
//   - IMU windows from both submaps (intra-submap each; no cross-submap window
//     stored — the BetweenFactor bridges the gap).
// On convergence, the refined T_WB of active.first_kf in sealed-local is the refined
// "consensus" — return refined_anchor = sealed.anchor * refined_consensus_origin,
// where refined_consensus_origin is the active-submap-LOCAL-ORIGIN pose in sealed-
// local frame (derived from refined active.first_kf and its original local pose).
//
// The refined ANCHOR is what we write back — it PERSISTS across VIO's next
// setActiveContent (which overwrites KFs/landmarks but NOT the archive-owned anchor).
// That's the V0 → V1 fix: V0 refined intra-submap KFs that got immediately
// overwritten; V1 refines the cross-submap anchor that survives.
bool refineWeldPair(const SubMap& sealed, const SubMap& active,
                    const SE3& consensus, const SupervisorConfig& cfg,
                    SE3& refined_anchor_out) {
  if (!cfg.global_smoother) return false;
  if (sealed.keyframes.empty() || active.keyframes.empty()) return false;
  if (sealed.kf_obs.size() != sealed.keyframes.size() ||
      active.kf_obs.size() != active.keyframes.size()) return false;

  GlobalBAInput in;
  in.calib = cfg.ba_calib;
  in.T_BS  = cfg.ba_T_BS;
  in.pixel_sigma = cfg.ba_pixel_sigma;
  in.max_iters   = cfg.ba_max_iters;
  in.imu_params  = ImuParams{};

  // Sealed: BA frame == sealed-local. No pre-transform.
  appendSubmapToBA(sealed, SE3(), in);
  // Active: T_BAframe_local = consensus (active-local → sealed-local).
  appendSubmapToBA(active, consensus, in);

  if (in.observations.empty()) return false;
  in.anchor_kf = sealed.keyframes.front().id;

  // IMU windows + velocity/bias per submap (intra-submap; the bridge is the loop
  // factor). Append after keyframes so finite-diff sees BA-frame translations.
  appendImuToBA(sealed, in);
  appendImuToBA(active, in);

  // Loop-closure BetweenFactor: the cross-submap link. measurement = T_a^-1 * T_b
  // (relative pose of `to` in `from`'s body frame), where `from` = last sealed KF
  // and `to` = first active KF, both initial poses in sealed-local.
  const auto& sealed_kf_last = sealed.keyframes.back();
  const auto& active_kf_first = active.keyframes.front();
  const SE3 T_sealed_last_in_BAframe = sealed_kf_last.T_WB;  // already in sealed-local
  const SE3 T_active_first_in_BAframe(consensus.matrix() * active_kf_first.T_WB.matrix());
  in.has_loop     = true;
  in.loop_kf_from = sealed_kf_last.id;
  in.loop_kf_to   = active_kf_first.id;
  in.T_from_to    = SE3(T_sealed_last_in_BAframe.matrix().inverse() *
                        T_active_first_in_BAframe.matrix());
  in.loop_sigma_t = cfg.ba_loop_sigma_t;
  in.loop_sigma_r = cfg.ba_loop_sigma_r;

  const GlobalBAOutput out = cfg.global_smoother->optimize(in);
  std::fprintf(stderr, "[ba] pair sealed=%lu active=%lu obs=%zu imu=%zu kfs=(%zu,%zu)"
                       " conv=%d init=%.1f final=%.1f iters=%d\n",
               (unsigned long)sealed.id, (unsigned long)active.id,
               in.observations.size(), in.imu_windows.size(),
               sealed.keyframes.size(), active.keyframes.size(),
               (int)out.converged, out.initial_cost, out.final_cost, out.iterations);
  if (!out.converged) return false;

  // Extract refined active.first_kf pose (in sealed-local). Convert to the active
  // submap's anchor: anchor_active (in world) = sealed.anchor * T_consensus_refined,
  // where T_consensus_refined is the active-LOCAL-ORIGIN pose in sealed-local.
  // If active.kf[0].T_WB = T_origin_first (the first KF's pose in active-local),
  // then T_consensus_refined = T_refined_first_in_sealed_local * T_origin_first^-1.
  SE3 refined_first_kf_in_sealed_local;
  bool found = false;
  for (const auto& kv : out.keyframes) {
    if (kv.first == active_kf_first.id) {
      refined_first_kf_in_sealed_local = kv.second;
      found = true;
      break;
    }
  }
  if (!found) return false;
  const SE3 T_consensus_refined(
      refined_first_kf_in_sealed_local.matrix() *
      active_kf_first.T_WB.matrix().inverse());
  refined_anchor_out = SE3(sealed.anchor.matrix() * T_consensus_refined.matrix());
  return true;
}
}  // namespace

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

  // C.live V1: refine the consensus via pair-BA in sealed-local frame BEFORE the
  // mode-specific anchor application. The BA's BetweenFactor is seeded with the
  // AnchorGate consensus; the visual + IMU residuals then pull both halves of the
  // weld pair into a metric-consistent geometry. The output is the refined ACTIVE
  // anchor (in world), from which we derive refined_consensus to feed all three
  // weld modes uniformly. Silent fallback to the AnchorGate consensus on failure
  // (BA unset / unconverged / no obs / id-mismatch). Off-by-default — back-compat.
  if (cfg_.global_smoother) {
    SE3 refined_anchor;
    if (refineWeldPair(*s, archive_.active(), consensus, cfg_, refined_anchor)) {
      // anchor_active = s.anchor * consensus  ⇒  consensus = s.anchor^-1 * anchor_active
      consensus = SE3(s->anchor.matrix().inverse() * refined_anchor.matrix());
    }
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
