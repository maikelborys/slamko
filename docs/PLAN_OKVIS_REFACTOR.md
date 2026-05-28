# PLAN — OKVIS-style architectural refactor

Branch: `okvis-arch-refactor` (off main @ 8294225).
Target: replace the multi-submap-anchor live trajectory layer with an OKVIS-class
single-graph architecture. Stable trajectory FIRST (no boundary jumps, no vertical
drift between submaps); cm-class ATE second (compound benefit, not the gate).

## The problem (diagnosed 2026-05-28)

slamko's corrected trajectory shows 100% of frame-to-frame jumps at SUBMAP
BOUNDARIES. Max single-frame jump 478 cm; vertical-only 303 cm. Cause is
structural, not config:

1. **Per-submap anchors**. Each `SubMap.anchor` is its independent submap-local
   → world transform. When the active submap transitions (auto-seal every 10 m
   on a clean traversal), the corrected-frame map → odom changes discretely. If
   two adjacent submaps' anchors differ (e.g. a weld updated one but not the
   other, or the chain pose-graph solves them differently), the world-frame
   trajectory jumps at the boundary.

2. **Identity sequential edges in the chain pose-graph** (`never_lost_supervisor.cpp:269`).
   The chain mode encodes adjacent sealed submaps as "consecutive ≡ identity",
   which is only correct if no weld changed any anchor between them. With ≥1
   weld in the chain, the chain optimiser fights the identity edges against the
   loop edges, producing inconsistent anchors → boundary jumps.

3. **Anchor algebra is layered ON TOP of VIO** instead of being baked into the
   estimator. VIO produces a continuous world trajectory; the supervisor's
   anchor corrections retroactively re-project past frames into a new world
   frame. This is fundamentally not how OKVIS works.

## What OKVIS2-X does (per `~/coding/OKVIS2-X/`)

One pose-graph for the live system, period. Loop closure is implemented by
**un-marginalizing** old keyframes back into the graph as reprojection-factor
sources (`addLoopClosureFrame` / `convertToObservations`) and running joint
VI-BA over the affected window in `fullGraph_`. The result is a self-consistent
trajectory at all times — no anchor layer, no boundary discontinuities. The
"submap" concept exists in OKVIS only for occupancy mapping (supereight),
**never** for loop-closure pose algebra.

Verified empirically this session: OKVIS2-X on magistrale1 = SE3-ATE 5.91 cm,
scale 0.9994, 15446-pose trajectory closes the loop within 0.96 m of the
start. slamko same dataset = SE3-ATE 10.06 m, end pose 21 m off.

## The refactor — phases

### P0 — Branch + plan (this commit)

- Branch `okvis-arch-refactor` off main @ 8294225.
- `main` stays untouched as the rollback floor.
- Document the design in this file + a `PLAN_OKVIS_REFACTOR_MILESTONES.md` file
  recording numerical gates as the refactor progresses.

### P1 — Single-graph backbone (1 week)

Replace the supervisor's "multi-submap-anchor + closed-form weld" layer with a
**single SessionGraph** holding **all KFs + landmarks + IMU windows** of the
live session. The slamko_vio frontend and the local fixed-lag smoothers
(`CeresLocalSmoother` / `GtsamLocalSmoother`) stay UNCHANGED — they already
output a continuous world trajectory; this is the right primitive. What
changes:

- **NEW**: `slamko_loop::SessionGraph` — owns a GTSAM `NonlinearFactorGraph` +
  `Values` over the WHOLE session. Receives KFs, landmark observations, IMU
  windows from the live VIO. Initial implementation: a fixed-lag pre-stage
  bridging the local smoother's output into the global graph (so the graph
  doesn't blow up; KFs older than N frames stay as marginalized priors).
- **NEW**: `slamko_loop::SessionGraphOptimizer` — async thread, runs LM /
  iSAM2 over the SessionGraph when (a) a loop closure was added or (b) a
  watermark threshold of KFs was reached. Lock-free seam between live and
  optimizer (OKVIS pattern: alternate `std::thread::join` boundary).
- **DROP from the live path**: `SubMapArchive`, `AnchorGate`, `PoseGraph`,
  `never_lost_supervisor`'s seal/branch/weld state machine. The SubMap struct
  + SMP4/5 codec are KEPT — used for offline saved maps + relocalizer DB.
- **VIO ↔ graph contract**: VIO publishes the live world pose every frame
  unchanged (no anchor layer); the SessionGraph publishes its refined node
  poses on a separate topic (the "true" trajectory). map ≡ world (no
  map→odom transform layer — the live trajectory IS in the refined frame
  after each SessionGraph update).

**Validation gate P1**: V1_01_easy 60 s replay. Pure VIO produces a smooth
trajectory; SessionGraph publishes the same poses with marginalization
working. No jumps. ATE ≥ pure-VIO baseline.

### P2 — Loop closure via un-marginalization (1 week)

The cm-class win. When the relocalizer (per-KF EigenPlaces VPR + PnP/LightGlue,
KEPT from current code) detects a verified loop:

1. Compute the loop relative pose (currently from the AnchorGate consensus).
2. **Un-marginalize** the historic KF in the SessionGraph: re-instantiate it as
   a free variable, replay its observations as reprojection factors against
   the current landmarks (those still alive in the graph, or via stored
   `kf_obs`). IMU window between it and its current neighbour rejoined.
3. Run VI-BA over the window encompassing all touched KFs (fixed-lag
   relinearization).
4. Result: refined trajectory + landmarks; the loop closure correction
   propagates smoothly across the window (no jump anywhere — the graph IS the
   trajectory).

This is the slamko equivalent of OKVIS's `addLoopClosureFrame` →
`convertToObservations` → `optimiseFullGraph()` sequence.

**Validation gate P2**: magistrale1 770 s replay. SE3-ATE within 2× of OKVIS
(target ≤ 12 cm). No boundary jumps. Loop demonstrably closes at the return.

### P3 — Recovery (tracking loss, simplified) (3-5 days)

Replace the seal/branch/weld dance with a simpler `TrackingMonitor`:

- Watches `health.odom_stale_gap_s` from the local smoother.
- On loss: VIO continues dead-reckoning on IMU; the SessionGraph keeps its
  pre-loss state.
- On recovery: aggressive relocalization (wider candidate window, higher
  vpr_top_n). When a verified loop closure lands, just feed it into the
  un-marginalization path (same as P2). No "seal" needed because the graph
  was never partitioned in the first place.

OKVIS's behaviour: when tracking is lost OKVIS basically dies and restarts.
slamko's CONTRIBUTION beyond OKVIS is the recovery path here — we keep it,
but as a SIMPLE policy on top of the same single graph, not as an
architectural concern.

**Validation gate P3**: EuRoC V1_01 with `dr_force_loss_windows` injected.
Recovery succeeds, trajectory is continuous across the loss + recovery.

### P4 — Cross-session persistence (3-5 days)

Save: serialize the SessionGraph (KFs, landmarks, IMU windows, descriptors,
VPR vectors per KF). One file per session. No multi-submap split — just one
graph.

Load: insert the saved graph as **frozen prior factors** into the new
session's SessionGraph. On first localization, the new session anchors to
the prior via a SessionGraph update (un-marginalization style, same as P2).

The existing SMP5 SubMap format becomes a LEGACY READER (kept for already-
saved maps). New saves write the new format.

**Validation gate P4**: V1_01 → V1_02 cross-session smoke. Live session
localizes into the prior map within 1 m on first verified loop closure;
trajectory continuous after the localization.

### P5 — Cleanup + docs (2-3 days)

- Delete now-unused code: `AnchorGate`, `PoseGraph` (the slamko version),
  `SubMapArchive`'s anchor algebra, `never_lost_supervisor`'s seal/branch/weld.
- Migrate tests: most P2/P3/P4 tests are over the dead code path → delete or
  rewrite minimally.
- Update `CLAUDE.md`, `MASTER_PLAN.md`, `docs/SYSTEM.md`, `docs/DECOUPLING.md`
  to reflect the single-graph architecture.

## Estimated total: 3 weeks of concentrated work

Per-phase gates are independent — each phase commits + can be merged to main
in isolation. If P2 (the cm-class fix) doesn't land the ATE target within its
budget, we re-evaluate before continuing.

## What stays from main (the salvage list)

- `slamko_core`: SE3 types, `ImuSample/ImuBias/ImuParams`, `StereoObservation`,
  `Features`, `KeyframeObservations` (SMP5 — IMU+VPR per KF still useful),
  `GlobalSmoother` contract, `GlobalBAInput/Output` (extended to support a
  whole-graph batch, not just pair).
- `slamko_vio`: ENTIRE frontend stack (XFeat, KLT, PnP, EigenPlaces TRT
  wrapper). The local smoother (`CeresLocalSmoother` / `GtsamLocalSmoother`)
  needs a minor adapter to feed SessionGraph instead of returning a per-call
  refined window — but the math is unchanged.
- `slamko_fusion`: `GtsamGlobalSmoother` (extended) + `GtsamLocalSmoother`.
- `slamko_loop`: `XFeatRelocalizer` (with EigenPlaces VPR + LightGlue verify)
  is KEPT — it's the front-end for loop closure detection. `LightGlueMatcher`,
  `BowVocabulary` (legacy), `Matcher` interface kept.
- Per-KF VPR + LightGlue + IMU substrate (SMP4/SMP5 schema) all KEPT.
- The benchmarks (`bench_ate.sh`, `bench_neverlost.sh`) get updated to gate
  the new graph path, not the seal/branch path.

## What we DROP

- `slamko_loop::NeverLostSupervisor` (replaced by `TrackingMonitor` + SessionGraph).
- `slamko_loop::SubMapArchive`'s anchor algebra (struct still useful for offline
  data; the multi-anchor live path is the part dropped).
- `slamko_loop::AnchorGate` (cluster gate — replaced by the relocalizer's own
  inlier-ratio + LightGlue gate plus the BA's outlier robustness).
- `slamko_loop::PoseGraph` (slamko's homegrown GN posegraph — GTSAM handles
  the whole graph; no need for a parallel posegraph).
- All "weld_*" / "seal_*" / "branch_*" config knobs, params, launch flags.
- The "auto-seal every N m" logic.
