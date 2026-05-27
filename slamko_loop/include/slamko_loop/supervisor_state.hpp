// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// SupervisorState + SupervisorConfig — the never-lost state machine's vocabulary
// and tunables. The state is a pure function of the odometry STALE-GAP
// (HealthSignal.odom_stale_gap_s) + dwell debounce; loss is NOT a covariance
// spike (MASTER_PLAN §7 / the OKVIS2-X finding). The internal 4-state machine is
// richer than core's HealthState{Good,Marginal,Lost}; toHealthState() projects it
// at the boundary.

#pragma once

#include "slamko_core/health.hpp"

namespace slamko {

enum class SupervisorState {
  OK,           // tracking healthy
  RecentlyLost, // odom gap in (recently_lost_gap_s, lost_gap_s] — VIO dead-reckoning coasts
  Lost,         // odom gap > lost_gap_s sustained — sealed + branched
  Relocalizing  // running place-rec on the fresh branch, attempting the weld
};

struct SupervisorConfig {
  // Loss trigger (seconds of odom stale-gap). lost_gap_s MUST be >= the VIO's
  // dead-reckoning horizon (dr_max_s_ = 1.0 s) or the supervisor seals while the
  // VIO is still confidently IMU-coasting — double-handling timescale-1 (R1).
  double recently_lost_gap_s = 0.25;
  double lost_gap_s          = 1.0;
  int    lost_dwell_frames   = 3;   // consecutive over-threshold steps before sealing
  int    recover_dwell_frames = 3;  // healthy steps AFTER a weld before declaring OK
  // While Relocalizing we keep attempting the weld even once odom is healthy
  // again — the re-acquired vision is exactly what lets us re-anchor to the
  // sealed map on revisit. Only give up (accept the branch as a standalone map,
  // un-welded) after this many healthy steps without a weld.
  int    reloc_give_up_frames = 400;

  // Lazy-anchor weld gate — THE false-relocalization defense (R2). A weld fires
  // only once weld_min_matches candidates agree within these radii (the analog of
  // OKVIS2-X's drift-budget gate that rejects a "100 m jump in a 5 m room").
  double weld_cluster_radius_m   = 0.10;
  double weld_cluster_radius_rad = 0.05;
  int    weld_min_matches    = 3;
  int    weld_min_inliers    = 15;   // RelocResult.num_inliers floor (pre-cluster gate)
  double weld_min_confidence = 0.0;  // RelocResult.confidence floor

  // P2.5: route the weld through the SE3 pose-graph backend instead of the single
  // closed-form composition. Each weld becomes a loop-closure FACTOR (sealed→active)
  // and ALL submap anchors are re-solved, so accumulated drift is distributed across
  // a multi-submap map. With one fixed sealed node + one edge this reduces exactly to
  // the composition (anchor_active = anchor_sealed · consensus), so default-off is
  // byte-identical to the validated P2c behavior. Turn on when merging >1 sealed map.
  bool   use_pose_graph = false;

  // P2.5 polish: weld at most once to each sealed TARGET per recovery episode. The
  // gate already fires whenever a fresh cluster of ≥weld_min_matches agreeing hits
  // forms, so without this it re-welds every few frames (the V1_01 "7× weld"
  // artifact) — harmless for the closed form, but in the pose-graph path it appends a
  // duplicate edge each time (unbounded growth). The clustered consensus is already an
  // average, so one weld per target carries the same correction. Welding to a DIFFERENT
  // sealed map in the same episode is still allowed. false = legacy refine-every-cycle.
  bool   weld_once_per_target = true;
};

// Boundary projection to the core health vocabulary.
inline HealthState toHealthState(SupervisorState s) {
  switch (s) {
    case SupervisorState::OK:           return HealthState::Good;
    case SupervisorState::RecentlyLost: return HealthState::Marginal;
    case SupervisorState::Lost:
    case SupervisorState::Relocalizing: return HealthState::Lost;
  }
  return HealthState::Lost;
}

}  // namespace slamko
