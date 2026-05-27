# slamko_loop — Status log

Living, dated progress + numbers log. Plan: [`PLAN_P2_loop.md`](PLAN_P2_loop.md).

## 2026-05-27 — P2a: never-lost supervisor v1 (decoupled, no-solver) ✅

**What:** the Tier-3 never-lost spine — slamko's flagship. A decoupled policy
(`NeverLostSupervisor`) that runs OUTSIDE the estimator graph (DigiForest): consumes
`HealthSignal` + `EstimationFrame`, drives the `OK→RecentlyLost→Lost→Relocalizing`
state machine, and OWNS `map→odom`. Built on `slamko_core` contracts only (Hard Rule
#2; no GTSAM, no ROS, no dense). New package `slamko_loop` (was README-only).

**Architecture (synthesis, confirmed with the user):** submap structure (OKVIS2-X/GLIM)
+ ORB-SLAM3 multi-map archive-restart + DigiForest decoupling (gate the weld OUTSIDE
the graph — tight coupling "pendulates"). Loss trigger = odometry **stale-gap**
(`HealthSignal.odom_stale_gap_s`), not a covariance spike.

**Components:**
- `NeverLostSupervisor` — `step(HealthSignal, EstimationFrame, t) → RecoveryAction`;
  seals + branches on a sustained stale-gap, attempts the weld while Relocalizing,
  recovers to OK on healthy-odom dwell. Owns `mapToOdom()`.
- `SubMapArchive` — multi-map Atlas; seal (frozen append-only) → branch (fresh
  origin); the archive owns each submap's id + `anchor` (the only post-seal mutation).
- `AnchorGate` — **multi-cluster** lazy-anchor weld gate (RANSAC-like): a weld fires
  only when ≥`weld_min_matches` place-rec candidates agree within a radius; consensus =
  manifold tangent-mean. THE false-relocalization defense (analog of OKVIS2-X's
  drift-budget gate). Outlier ordering can't poison the consensus.

**`map→odom` convention (load-bearing, pinned in the header + asserted):** active-branch
local frame == odom frame, so `T_map_odom == active.anchor` (identity until welded). On
weld to sealed `S` with consensus `C` (active→sealed, == `RelocResult.T_query_match`):
`active.anchor = S.anchor · C`. Held constant between welds (odom runs free —
disposable global graph).

**v1 = NO solver.** A single weld is exact SE3 composition — no Ceres/GTSAM. Validates
the whole architecture (seal→branch→relocalize→weld, decoupled, lazy-gated, owns
map→odom) deterministically, keeps Hard Rule #2 trivially true, and dodges the GTSAM
SONAME fragility. The nonlinear pose-graph (averaging ≥2 conflicting constraints) is P2.5.

**GATE — 10 gtests, 0 failures** (`colcon test --packages-select slamko_loop`; synthetic
inputs, no ROS/rosbag2 — sidesteps the box's flaky run-harness, as P1 did):
seal+branch on stale-gap·dwell · one-frame blip doesn't seal · RecentlyLost doesn't seal ·
**weld on consensus + exact map→odom** · scattered hits rejected (false-place-rec) ·
mixed outliers weld on the agreeing consensus · low-inlier rejected · **non-identity
sealed-anchor composition** (`T1·T2`) · archive seal/branch/find/anchor primitives.

**Next — P2b:** XFeat relocalizer (`slamko_core::Relocalizer` impl) — descriptor match
on the N×64 XFeat index `slamko_vio::buildSubMap()` already ships + PnP/RANSAC
verification → drives the real weld. Then **P2.5:** loop-closure-as-factor + a tiny
self-contained SE3 pose-graph solver (catch→damp→rebuild). Deferred plugin: dense
submap-to-submap alignment `Factor` (the OKVIS2-X map-to-map mechanism) for
forest/repetitive robustness, opt-in via the pluggable `Factor` contract + a dense
payload — zero core changes.

## 2026-05-27 — P2b: XFeat relocalizer + supervisor weld refinement ✅

**What:** the weld is now real — `XFeatRelocalizer` (implements
`slamko_core::Relocalizer`) localizes a query frame against archived submaps using the
XFeat descriptors the VIO already attaches to landmarks (`SubMap`'s N×64 index) — **no
new model**: brute-force NN match (Lowe ratio) → 2D-3D correspondences → **PnP-RANSAC
(core P3P)** → query camera pose in the matched submap's local frame → converted to the
**body** frame via the cam↔body extrinsic. Depends on `slamko_core` only (no OpenCV).

- **P3P → slamko_core** (`slamko_core/include/slamko_core/p3p_solver.hpp`, `slamko::p3p`):
  the header-only pure-C P3P copied from slamko_vio so loop reuses it without a
  cross-package dep (Hard Rule #2). HD macro renamed (`SLAMKO_P3P_HD`) so both copies
  can coexist in one TU (the P2c app). vio keeps its copy; dedup later.
- **PnP-RANSAC** = a small Eigen helper in the relocalizer (mirrors `pnp_cuda.cu`'s pure
  math: sample 3 → P3P → reproject `u=fx·X/Z+cx`, count inliers < thr² → argmax),
  deterministic RNG seed.
- **Supervisor weld refinement (load-bearing, OKVIS2-X-validated):** the relocalizer
  returns `RelocResult.T_query_match` = the query **body** pose in sealed-local
  (absolute). The supervisor composes it with the live odom to get the weld constraint
  `T_active_sealed = T_query_match · odom.T_WB⁻¹` (the OKVIS2-X `T_AB = T_AS_query ·
  T_WS_current⁻¹` formula) and feeds THAT to the lazy-anchor gate — a frame transform
  invariant over the short reloc window even as odom moves. Keeping the extrinsic in the
  (camera-aware) relocalizer lets the supervisor stay body-frame. Backward-compatible:
  the P2a tests use identity odom, so the composition is a no-op there — still green.

**GATE — 16 gtests, 0 failures** (`colcon test slamko_loop`): the 10 P2a supervisor/
archive tests still pass + a new **odom-composition weld** test; **4 relocalizer tests** —
recovers a synthetic query pose with **identity AND non-identity extrinsic** (the
cam↔body round-trip, asserted to 1e-4 — 3-pt P3P precision), **RANSAC rejects 10/30
outlier correspondences** while recovering the exact pose, and **no-match / too-few →
`found=false`**. Synthetic, no ROS/rosbag2.

**Next — P2c:** the integration harness — a composition-root app linking
slamko_vio+loop+core that replays EuRoC, runs the VIO, feeds the supervisor +
relocalizer, induces a vision-loss window (`dr_force_loss_start_s/end_s`), and logs
seal→branch→relocalize→weld on real data (the first end-to-end never-lost bag test;
replay + action-logging, no rosbag2 recording).

**Integration note (R1):** `lost_gap_s` (1.0 s default) ≥ the VIO dead-reckoning horizon
(`dr_max_s_`=1.0) so the supervisor doesn't double-handle the ms-gap net. `odom_stale_gap_s`
is populated by the VIO only while `in_dead_reckoning_` (gated by `dr_enabled_`, default
off) — a fully general wall-clock stale-gap is a minor VIO refinement for the live wiring
(P4); the v1 supervisor is validated on synthetic signals, so not blocked.
