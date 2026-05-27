// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// NeverLostSupervisor — the Tier-3 never-lost policy (slamko's flagship). It runs
// OUTSIDE the estimator graph (DigiForest decoupling): each step it consumes a
// HealthSignal + the live EstimationFrame, drives the seal→branch→relocalize→weld
// state machine, and OWNS the map→odom correction. Loss is the odometry STALE-GAP
// (not a covariance spike). The weld is gated by the AnchorGate (lazy-anchor
// cluster) so a false relocalization can never jump the map — and applied by exact
// SE3 composition (no solver in v1), so it is deterministically testable.
//
// map→odom convention (LOAD-BEARING — pinned here, asserted in tests):
//   * SubMap.anchor is submap-local → global/map. The ACTIVE branch's local frame
//     IS the odom frame, so  T_map_odom == active.anchor  (identity until welded).
//   * RelocResult.T_query_match is active(query) → sealed-local (per its contract).
//   * On weld to sealed submap S with consensus C (active→sealed-local):
//       active.anchor = S.anchor * C        // (active→sealed→map) == T_map_odom
//   * Downstream composes  T_map_base = T_map_odom * T_odom_base  (T_odom_base = the
//     VIO odom EstimationFrame.T_WB). Held constant between welds (odom runs free —
//     disposable global graph).

#pragma once

#include <cstdint>
#include <vector>

#include "slamko_core/estimation_frame.hpp"
#include "slamko_core/features.hpp"
#include "slamko_core/health.hpp"
#include "slamko_core/relocalizer.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"

#include "slamko_loop/anchor_gate.hpp"
#include "slamko_loop/pose_graph.hpp"
#include "slamko_loop/submap_archive.hpp"
#include "slamko_loop/supervisor_state.hpp"

namespace slamko {

// What the supervisor decided this step — observable so tests assert behaviour.
struct RecoveryAction {
  bool          sealed = false;     std::uint64_t sealed_id = 0;
  bool          branched = false;   std::uint64_t branched_id = 0;
  bool          welded = false;     std::uint64_t welded_to_id = 0;
  SE3           applied_T_active_sealed;  // the consensus weld (active→sealed)
};

class NeverLostSupervisor {
 public:
  // reloc may be null (no relocalization → never welds, but still seals/branches
  // and recovers locally). Owned by the caller; the supervisor never frees it.
  NeverLostSupervisor(const SupervisorConfig& cfg, Relocalizer* reloc);

  // One policy step. monotonic_now_s is wall-clock/monotonic (sim-time-proof).
  RecoveryAction step(const HealthSignal& h, const EstimationFrame& odom,
                      double monotonic_now_s);

  SupervisorState state() const { return state_; }
  HealthState     healthState() const { return toHealthState(state_); }
  SE3             mapToOdom() const { return archive_.active().anchor; }
  const SubMapArchive& archive() const { return archive_; }
  const PoseGraph&     poseGraph() const { return graph_; }  // P2.5 (opt-in) state

  // Estimator inputs (staged; consumed at the next step — single-writer doctrine).
  void submitActiveSubMap(SubMap sm) { archive_.setActiveContent(std::move(sm)); }
  void submitQueryFeatures(const Features& f) { query_ = f; have_query_ = true; }

  // Cross-session: seed the archive with a prior map's submaps (frozen, anchored).
  // The relocalizer DB is seeded by the caller (it owns the Relocalizer*); thereafter
  // the SAME weld machinery localizes the live session into the prior map.
  void seedPriorMap(std::vector<SubMap> priors) { archive_.seedPriorMap(std::move(priors)); }

 private:
  bool attemptWeld(RecoveryAction& act);

  SupervisorConfig cfg_;
  Relocalizer*     reloc_ = nullptr;
  SubMapArchive    archive_;
  AnchorGate       gate_;
  PoseGraph        graph_;  // P2.5: submap-anchor pose graph (used iff cfg.use_pose_graph)

  SupervisorState  state_ = SupervisorState::OK;
  int              lost_count_ = 0;     // consecutive over-threshold steps
  int              recover_count_ = 0;  // consecutive healthy steps (recovery dwell)
  bool             episode_welded_ = false;  // re-anchored this Relocalizing episode?
  std::vector<std::uint64_t> episode_welded_ids_;  // sealed targets welded this episode
  double           last_now_s_ = 0.0;
  SE3              odom_T_WB_;           // latest live odom body pose (for the weld)
  int              cont_counter_ = 0;    // throttle for OK-state continuous reloc

  // Auto-seal odometry-distance tracker (cfg_.auto_seal_distance_m).
  double           dist_since_seal_ = 0.0;
  Eigen::Vector3d  last_odom_t_ = Eigen::Vector3d::Zero();
  bool             have_last_odom_t_ = false;
  // Chain-mode (auto-seal): accumulated loop-closure edges, deduped by (from,to) so a
  // revisit refreshes rather than appends. Re-applied on every rebuild of the submap
  // chain so a single optimize() distributes ALL closures across the whole chain.
  std::vector<PoseGraphEdge> loop_edges_;

  Features         query_;
  bool             have_query_ = false;
};

}  // namespace slamko
