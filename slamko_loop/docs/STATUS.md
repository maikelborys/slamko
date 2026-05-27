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

## 2026-05-27 — P2c: never-lost spine validated END-TO-END on a real bag ✅

**What:** the supervisor + XFeat relocalizer are now wired into the **live VIO** (the
`slamko_vio_node` is the composition root, behind `enable_neverlost:=true`; it gains a
node-only dep on `slamko_loop`, core lib stays decoupled). Each frame the node feeds the
supervisor: `health()` (the odom stale-gap), the odom `EstimationFrame`
(`T_WB = worldPose · T_BS⁻¹`), the active `buildSubMap()` (every 30 frames), and query
`Features` from the current tracks; on SEAL it registers the sealed submap with the
relocalizer. Logs every recovery action + state transition.

**GATE — forced-loss replay on MH_01 (rate 1.0, `dr_force_loss=[30,33]s`):** the FULL
never-lost cycle fired on real VIO health (not synthetic):
```
t=30.0  tracking loss (forced) → IMU dead-reckoning
        [neverlost] OK → RecentlyLost
t=31.8  [neverlost] SEAL submap 0 + BRANCH 1  (odom_stale_gap=1.15s > lost_gap 1.0, after dwell)
        [neverlost] RecentlyLost → Relocalizing
t=33.0  tracking recovered (60 dead-reckoned frames, 3.05s)
        [neverlost] Relocalizing → OK
```
So **seal→branch→keep-emitting-odom→recover** works end-to-end on a bag, the loss
trigger being the real odometry stale-gap. The **weld** (re-anchor on revisit) is the
remaining piece — it needs XFeat descriptors (this run used Shi-Tomasi, descriptor-less)
AND a revisit of the pre-loss area; a revisiting/xfeat sequence is the P2c follow-on.

**Harness lessons (documented so the next session doesn't relearn):** (1) double-typed
launch args MUST be passed with a decimal (`dr_force_loss_start_s:=30.0`, not `30`) or
rclcpp aborts with InvalidParameterType. (2) NEVER put `pkill -f <pattern>` inside a
run-script whose own text contains `<pattern>` — it matches the script's bash and
self-kills (this caused the earlier empty-output failures). Reap by PID, or in a
separate command, or rely on the process-group kill. (3) `ros2 launch` children can
escape the launch process group — reap leftover `euroc_player`/node by PID after.

## 2026-05-27 — P2c weld VALIDATED end-to-end on V1_01 (XFeat) ✅

**What:** the **weld** (re-anchor on revisit) now fires on a real bag — the full
never-lost loop **seal→branch→relocalize→WELD→recover** is closed. Two fixes made it work:
- **Supervisor stays Relocalizing until welded** (or a give-up timeout,
  `reloc_give_up_frames`) — it no longer exits to OK on healthy odom alone, so the
  re-acquired vision after the blackout actually gets used to re-anchor (the earlier
  run recovered to OK before a weld could cluster).
- **Relocalizer DB cap** (`max_db_landmarks`, stride-subsample) — brute-force NN match
  against the cumulative submap (tens of thousands of landmarks) would stall the node;
  capped to keep `relocalize()` cheap (a vocabulary/inverted index is the scalable swap).

**GATE — V1_01_easy (Vicon Room, `feature_source:=xfeat`, `dr_force_loss=[25,28]s`):**
```
t=25.0  forced loss → IMU dead-reckoning;  OK → RecentlyLost
t=26.4  SEAL submap 0 + BRANCH 1 (odom_stale_gap 1.15s) → Relocalizing
t=26.x  WELD to submap 0 (inliers-gated); map→odom t=[0.18 0.20 0.00]  ← XFeat re-localized
        the branch against the sealed map; 7 welds refined map→odom (~0.2 m correction)
t=28.0  tracking recovered → OK
```
So the branch's XFeat descriptors matched the **sealed** submap (same room) → PnP-RANSAC
→ the lazy-anchor gate cleared → `map→odom` re-anchored. Plot (GT + Sim3-aligned estimate
+ 47.6k-landmark map + the red dead-reckoned loss segment) via the new
`scripts/plot_neverlost.py`: Sim3-ATE 22.5 cm (inflated by the 3 s blackout; scale 0.9975).
Loop unit tests: **17 gtest 0 fail** (added `RelocalizingStaysUntilWelded`).

**Note:** the weld re-fires every gate cycle (7× here) — fine for v1 (each refines
map→odom); a "weld-once-then-cooldown" is a cheap future polish.

**Next:** P2.5 (loop-closure-as-factor + self-contained SE3 pose-graph solver) ·
offline plotter is `scripts/plot_neverlost.py`.

## 2026-05-27 — P2.5: SE3 pose-graph backend (loop-closure-as-factor) ✅

**What:** the closed-form single weld is now backed by a tiny, self-contained
**SE(3) pose-graph solver** (`PoseGraph`, `pose_graph.{hpp,cpp}`) — nodes = submap
**anchors**, edges = revisit/loop-closure constraints **as factors**
(`Z = anchor_from⁻¹ · anchor_to`). A Gauss-Newton sweep on the manifold (right-
perturbation, `se3.hpp` only) finds the anchors that best satisfy all edges, so
accumulated drift is **distributed across a multi-submap map** instead of jumping one
anchor. Depends on `slamko_core` only (Hard Rule #2) — no GTSAM (dodges the SONAME
fragility the fusion tier hit), ~210 lines.

- **Jacobians:** small-residual approximation `J_r⁻¹(r) ≈ I` → `∂r/∂δ_from =
  -Adj(X_to⁻¹·X_from)`, `∂r/∂δ_to = I`. **Exact at a consistent optimum** (all r→0,
  weighting drops out) — the regime the never-lost weld lives in — and a standard,
  robust approximation off it. Full SE(3) `J_r⁻¹` is a drop-in later. Win condition =
  map-merge robustness, not sub-cm MAP optimality ([[slamko-robustness-over-accuracy]]).
- **Disposable-graph robustness (Hard Rule #4):** LM damping on the H diagonal +
  per-step trial/accept (reject → damp harder) means a non-SPD / ill-conditioned step
  never crashes; an optional outlier pass drops the single worst-χ² edge and re-solves,
  so a bad loop closure **can't stick**. `optimize()` never throws.
- **Gauge:** exactly one fixed node (auto-pins the lowest id). One fixed node + one
  edge reduces ALGEBRAICALLY to the closed-form weld `anchor_active = anchor_sealed ·
  consensus` — the backward-compat property asserted in tests.

**Supervisor integration (opt-in, `SupervisorConfig.use_pose_graph`, default OFF):**
each weld now records a `PoseGraphEdge` (sealed→active) and re-solves ALL anchors,
writing the optimized poses back via `archive_.setAnchor` (the sole legal post-seal
mutation). **Default-off is byte-identical to the validated P2c behavior**, so the
flagship V1_01 result is untouched; turn it on to merge >1 sealed map. Unbounded edge
growth over a very long session = a P4 marginalization concern (noted, not blocking).

**GATE — 22 gtests, 0 failures** (`colcon test slamko_loop`; synthetic, no ROS):
- `test_pose_graph` (5): single edge == closed-form weld (1e-9) · consistent 3-submap
  loop recovered exactly from a perturbed start (cost→0) · conflicting edges balance
  residuals (Σrₖ=0, genuinely blended) · under-excited node stays put (LM stability) ·
  gross-outlier edge dropped + good consensus recovered.
- `test_supervisor` (+2 = 13): weld-via-pose-graph matches the composition (2 nodes,
  1 edge) · two sequential welds chain submaps exactly (`T1·T2`, 3 nodes, 2 edges).
- `test_relocalizer` (4) unchanged.

**Next:** continuous relocalization in the OK state (welds add edges + re-optimize
rather than only firing during recovery) now that the graph backend exists; full SE(3)
`J_r⁻¹` if a real inconsistent-graph ATE pass ever shows the approximation costs us;
deferred dense submap-to-submap `Factor` (OKVIS2-X map-to-map) as a graph edge.

**Integration note (R1):** `lost_gap_s` (1.0 s default) ≥ the VIO dead-reckoning horizon
(`dr_max_s_`=1.0) so the supervisor doesn't double-handle the ms-gap net. `odom_stale_gap_s`
is populated by the VIO only while `in_dead_reckoning_` (gated by `dr_enabled_`, default
off) — a fully general wall-clock stale-gap is a minor VIO refinement for the live wiring
(P4); the v1 supervisor is validated on synthetic signals, so not blocked.
