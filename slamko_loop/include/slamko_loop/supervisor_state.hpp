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

#include "slamko_core/global_smoother.hpp"
#include "slamko_core/health.hpp"
#include "slamko_core/stereo_observation.hpp"

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
  //
  // Defaults relaxed for the robust-localization regime (V.3, 2026-05-28): the
  // pre-cluster floor (weld_min_inliers=15 + LightGlue's geometric verify upstream)
  // makes 1 verified hit already strong evidence of a true place. Requiring 3-in-10cm
  // produced 0 welds on a 1500 s replay despite 579 PnP-verified hits (each PnP-RANSAC
  // call has ~10 cm single-shot translation noise → 3 in 10 cm is over-calibrated).
  // 2-in-30cm still rejects scattered perceptual-alias singletons (the gate's purpose)
  // while admitting the dense correct hits on a revisit. The 4 layers of defense are:
  // (1) PnP-RANSAC inlier consensus, (2) min_inliers floor, (3) this cluster gate,
  // (4) LightGlue's learned matcher (when on). Tests pin tighter values explicitly.
  double weld_cluster_radius_m   = 0.30;
  double weld_cluster_radius_rad = 0.10;
  int    weld_min_matches    = 2;
  int    weld_min_inliers    = 15;   // RelocResult.num_inliers floor (pre-cluster gate)
  double weld_min_confidence = 0.0;  // RelocResult.confidence floor

  // P2.5: route the weld through the SE3 pose-graph backend instead of the single
  // closed-form composition. Each weld becomes a loop-closure FACTOR (sealed→active)
  // and ALL submap anchors are re-solved, so accumulated drift is distributed across
  // a multi-submap map. With one fixed sealed node + one edge this reduces exactly to
  // the composition (anchor_active = anchor_sealed · consensus), so default-off is
  // byte-identical to the validated P2c behavior. Turn on when merging >1 sealed map.
  bool   use_pose_graph = false;

  // P4b-2: continuous relocalization — attempt a weld in the OK state too (not only
  // during recovery), throttled to every continuous_reloc_interval frames. This is
  // how a session localizes into a PRIOR map / closes a loop with its own sealed
  // submaps WITHOUT having to get lost first (the deploy-correct behavior). It only
  // updates map→odom on a clustered match (the gate still guards false-relocs); the
  // state stays OK and the fast odometry is untouched (Hard Rule #4). Default off.
  bool continuous_reloc          = false;
  int  continuous_reloc_interval = 15;  // frames between OK-state weld attempts

  // Periodic AUTO-SEALING (OK state, independent of tracking loss). On a long,
  // clean-tracking traversal slamko otherwise stays in ONE active submap forever —
  // and since the relocalizer DB only holds SEALED submaps, a revisit to the start
  // (still inside that one active submap) can never close a loop. Sealing the active
  // submap every `auto_seal_distance_m` of travel checkpoints it into the sealed set
  // (→ the reloc DB), so when continuous_reloc later matches it the weld closes the
  // loop. The branch inherits the sealed submap's anchor so map→odom stays continuous
  // (no jump — this is a voluntary checkpoint, NOT a loss). 0 = off (default; existing
  // loss-only behavior is byte-identical). Pairs with continuous_reloc=true + a
  // relocalizer. Typical: 8–15 m.
  double auto_seal_distance_m = 0.0;

  // P2.5 polish: weld at most once to each sealed TARGET per recovery episode. The
  // gate already fires whenever a fresh cluster of ≥weld_min_matches agreeing hits
  // forms, so without this it re-welds every few frames (the V1_01 "7× weld"
  // artifact) — harmless for the closed form, but in the pose-graph path it appends a
  // duplicate edge each time (unbounded growth). The clustered consensus is already an
  // average, so one weld per target carries the same correction. Welding to a DIFFERENT
  // sealed map in the same episode is still allowed. false = legacy refine-every-cycle.
  bool   weld_once_per_target = true;

  // C.live: visual+IMU bundle-adjustment on weld. When non-null, after each weld the
  // supervisor builds a 2-submap (active + sealed) BA over their KFs + landmarks +
  // observations + IMU windows, with a BetweenFactor loop-closure constraint from the
  // weld consensus. The refined active KFs/landmarks are written back into the archive
  // so the next welds compound on a sharper geometry. The smoother is owned by the
  // caller (composition root — vio_node), the supervisor never frees it; null = legacy
  // closed-form / chain pose-graph behaviour, bit-identical to pre-C.live runs.
  // Hard Rule #2: this is the slamko_core abstract type, so slamko_loop stays free
  // of slamko_fusion (the concrete GtsamGlobalSmoother lives in fusion).
  GlobalSmoother* global_smoother = nullptr;
  StereoCalib     ba_calib;            // intrinsics + baseline for the reprojection factors
  SE3             ba_T_BS;             // cam→body (same convention as the relocalizer)
  double          ba_pixel_sigma = 1.0;
  int             ba_max_iters   = 20;
  // Loop-closure factor sigmas (the BetweenFactor between sealed_last_KF & active_first_KF
  // derived from the weld consensus). Tight enough to anchor; not so tight it overrides
  // the visual reprojection evidence.
  double          ba_loop_sigma_t = 0.05;  // m
  double          ba_loop_sigma_r = 0.02;  // rad
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
