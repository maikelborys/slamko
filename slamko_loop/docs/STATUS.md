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

## 2026-05-27 — P2.5 hardening: stress suite + weld-once polish ✅

**What:** stress-tested the pose-graph backend + supervisor at scale, and shipped the
polish the P2c log flagged (the weld re-firing every gate cycle — the V1_01 "7× weld").

- **Polish — `weld_once_per_target` (default ON):** the supervisor now welds at most
  once to each sealed TARGET per recovery episode (tracks `episode_welded_ids_`, cleared
  on episode start). The gate's clustered consensus is already an average, so a second
  weld to the same map adds nothing — and in the pose-graph path it appended a DUPLICATE
  edge each cycle (unbounded growth). Welding to a *different* sealed map in the same
  episode is still allowed. Final anchor quality is unchanged (same clustered consensus);
  the V1_01 closed-form path now logs 1 weld instead of 7. `false` = legacy refine-every-cycle.

- **Stress suite (`test_stress.cpp`, +10 gtests):**
  - *PoseGraph (7):* 30-submap chain + loop-closure recovered (consistent ⇒ cost→0) ·
    5×5 grid, 16 loops, converges · 6 good + **5 gross outliers ALL dropped** (worst-χ²
    first) → good consensus · **deterministic** (two identical graphs → bit-identical
    anchors, `EXPECT_DOUBLE_EQ`) · **idempotent** re-optimize (≤2 iters, no drift) ·
    **under-constrained gauge-free component stays FINITE** + preserves its internal
    relative (LM damping, Hard Rule #4) · large-rotation (~1.2 rad) loop converges.
  - *Supervisor (3):* **10 seal→branch→weld→recover cycles** (Atlas scale) → graph grows
    exactly 1 node + 1 edge/cycle, no crash · **weld-once bounds edges** (30 hits → 1 weld
    / 1 edge ON; >1 / >1 OFF) · **flapping health doesn't thrash** (alternating gap never
    seals — needs consecutive dwell).

**GATE — 32 gtests, 0 failures** (`colcon test slamko_loop`; was 22). No new deps, no
RNG in the solver (the determinism test guarantees reproducibility the VIO harness lacks).

## 2026-05-27 — P2 CLOSED: multi-submap pose-graph merge validated on a live bag ✅

**What:** the pose-graph backend now ran end-to-end on the live `slamko_vio_node`,
merging **TWO sealed submaps** on a real replay — the validation gap (the backend was
only unit-tested at scale) is closed. Wired `neverlost_use_pose_graph` + `neverlost_weld_once`
node params (default off / on) and a multi-window forced-loss hook
(`dr_force_loss_windows="s:e,s:e"`) so one replay induces several seals.

**GATE — V1_01_easy, `feature_source:=xfeat neverlost_use_pose_graph:=true
neverlost_weld_once:=true dr_force_loss_windows:="20:23,45:48"` (rate 1.0, ~real-time):**
```
t=20  forced loss → SEAL submap 0 + BRANCH 1 (odom_stale_gap 1.15s)
      WELD branch 1 → submap 0   map→odom t=[0.40 0.53 -0.38]   → recover (state 3→0)
t=45  forced loss → SEAL submap 1 + BRANCH 2 (odom_stale_gap 1.15s)
      WELD branch 2 → submap 0   map→odom t=[1.85 0.71 1.54]    → recover
```
So the archive held **2 sealed submaps {0,1}**, and the SE3 pose graph solved over
nodes {0 (fixed gauge), 1, 2} with edges {0→1, 0→2} on **real XFeat→PnP** weld
consensuses — the first live multi-submap merge. **weld-once held**: exactly ONE weld
per episode (vs the 7× of the single-loss P2c run), so the graph grew by exactly one
edge per recovery. 1363 poses + 194.6k landmarks dumped; plot (GT + Sim3-aligned est +
48.4k-landmark map @ min-obs 3 + both red dead-reckon segments) via
`scripts/plot_neverlost.py --loss 20 23 45 48`. Sim3-ATE **59.9 cm** — inflated by 6 s
of IMU-only coasting across the TWO blackouts (scale 0.991); per
[[slamko-robustness-over-accuracy]] the win is the never-lost recovery + merge, not ATE.

**Harness note:** launch passes bool params as strings; a bare `LaunchConfiguration`
silently leaves a bool node-param at its default (saw `pose_graph=0` despite `:=true`).
Fix = `ParameterValue(LaunchConfiguration(name), value_type=bool)` (the `_bool()` helper
in `vio_euroc.launch.py`) — the bool analog of the `30` vs `30.0` double lesson.

**P2 status: CLOSED.** Never-lost spine validated end-to-end — single-submap weld
(P2c) and now multi-submap pose-graph merge — on real V1_01 data. Next phase is P3/P4.

## 2026-05-27 — merge VISUALIZATION fix: anchor-corrected map (the welds were right) ✅

**Symptom (user):** in the live multi-submap plot the sealed submaps looked "shifted,
one turned." **Root cause (NOT a seal bug):** the VIO landmark + pose dumps are the raw
continuous **odom frame** — dead-reckoning drift across each forced blackout is baked in,
and the never-lost weld correction (each submap's `anchor`, i.e. `map = anchor·odom`) was
**never applied to the dumps.** The `.submaps` sidecar makes it concrete: submap 0 = gauge
(identity), submap 1 anchor = small (the 0.33 m weld), **submap 2 anchor = a ~49° rotation
+ 2.43 m shift** — exactly the yaw+translation DR drift the user saw as "turned" (3 s of
IMU-only with the known gravity-direction error). The weld *measured* it correctly.

**Fix (viz/reconstruction, no algorithm change):** the node now tags the map by submap —
the landmark-id seam at each SEAL (`pipeline_->maxLandmarkId()`; IDs are monotonic) →
`<lm>.submaps` (per-submap id range + final welded anchor) + a per-frame `<pose>.epoch`
(active submap id). `scripts/plot_neverlost.py --submaps --pose-epoch` moves each submap's
landmarks/poses into the merged MAP frame via its anchor BEFORE the Sim3 fit. Result on
V1_01: the cloud tightens into one coherent room and **Sim3-ATE drops 56.9 → 31.1 cm**
purely from applying the welds — the quantitative proof the merge realigns the submaps.

**Honest residual:** a single rigid anchor per submap fixes the submap's *placement*, not
the *intra-blackout* trajectory wiggle (continuous DR drift), so the red dead-reckon
segments still bend; a fully clean map needs re-integration from the corrected poses (P4
mapping) or shorter/again-bounded blackouts. The never-lost contract — never lost, re-anchor
on revisit — holds; map polish is P4.

**Auto-check (no more eyeballing):** `scripts/check_neverlost.py` is a PASS/FAIL gate over a
run's log + dumps — asserts (1) expected #SEAL/#WELD + ended OK, (2) weld anchors SANE (no
"100 m jump in a 5 m room" false-reloc), (3) the welds IMPROVE ATE (corrected < raw), (4)
corrected ATE under a bound, (5) #submaps == #seals+1. On the V1_01 multi-loss run: **7/7
PASS** (seals 2, welds 2, ended OK, max|anchor t| 2.39 m < 5, ATE 31.1 < 56.9 cm < 45). So
the seal/weld is objectively correct — the earlier "shifted/turned" was purely the
uncorrected-odom viz.

## 2026-05-27 — submap partition: disjoint, self-contained sealed submaps ✅

**The "overexpose" wart, fixed.** Before: the VIO's `landmark_world_` is never pruned and
`buildSubMap()` returned the WHOLE cumulative map, so each sealed submap was a SUPERSET of
the earlier ones (reloc DB registered overlapping copies — landmarks duplicated, not
dissolved). Now the VIO tags each landmark with a **submap epoch** at creation; the node
calls `pipeline_->beginSubmap()` on BRANCH (++epoch); `buildSubMap()` returns ONLY the
active epoch's landmarks → **disjoint, self-contained submaps** (the OKVIS2-X/GLIM model).
Epoch stays 0 with no branch ⇒ a normal/no-loss run is byte-identical to before.

**GATE (V1_01, re-run, auto-check 7/7 PASS):** SEAL submap 0 = **40,615** landmarks, submap
1 = **90,707** — its OWN epoch, NOT the cumulative ~132k it used to be. Both welds still
fired (disjoint reloc DB doesn't starve the match), anchors sane (max |t| 2.18 m),
anchor-corrected Sim3-ATE **16.9 cm < raw 56.1 cm**. So the seal/weld was always correct;
now the sealed submaps are genuinely independent (no duplicate landmarks across the archive
or the reloc DB) — the foundation cross-session persistence (P4) needs.

**Integration note (R1):** `lost_gap_s` (1.0 s default) ≥ the VIO dead-reckoning horizon
(`dr_max_s_`=1.0) so the supervisor doesn't double-handle the ms-gap net. `odom_stale_gap_s`
is populated by the VIO only while `in_dead_reckoning_` (gated by `dr_enabled_`, default
off) — a fully general wall-clock stale-gap is a minor VIO refinement for the live wiring
(P4); the v1 supervisor is validated on synthetic signals, so not blocked.

## 2026-05-27 — P4b-1: cross-session Atlas seeding (load prior map → relocalize into it) ✅

**What:** the never-lost archive now seeds from a PRIOR map on disk, so a fresh session
localizes into it — the cross-session pillar. `SubMapArchive::seedPriorMap(priors)` imports
the loaded submaps as frozen sealed maps (keeping ids+anchors = session-1 frame) and
re-ids the live submap PAST them (no collision). The node gains `prior_map_dir` (loadSubMaps
→ seed archive + reloc DB at startup) and `map_save_dir` (saveSubMaps at shutdown). The SAME
weld machinery then localizes the live session into the prior map — the prior map is just
more sealed submaps in the Atlas (no new "localization mode").

**GATE — two sessions on V1_01 (XFeat, pose-graph):**
- Session 1: mapped the room, saved a 1-submap Atlas (`submap_0.smap`, 43 MB w/ descriptors).
- Session 2: `loaded 1 prior submaps (live ids start at 1)`; early forced loss → SEAL submap
  1 (14,295 lm) + BRANCH 2 → `WELD to submap 1` (own) → **`WELD to submap 0 [CROSS-SESSION/
  prior map]`** — relocalized into session 1's map, tying session 2's frame into it via a
  pose-graph edge (live→prior). `check_neverlost.py` **7/7 PASS**, anchor-corrected ATE
  13.9 cm < raw 24.0. Welding to BOTH its own submap 1 AND the prior submap 0 is the
  weld-once-per-TARGET multi-target behavior working as designed.

**Unit gate:** `SeedPriorMapImportsAndWeldsCrossSession` (loop) — seed 2 priors, live id past
them, weld to a prior id re-anchors map→odom into the prior frame. Loop suite 14 supervisor
gtests, 0 fail.

**Next — P4b-2:** continuous relocalization in the OK state — so a session localizes into the
prior map WITHOUT needing a forced loss (the deploy-correct behavior; the forced loss here
just reuses the recovery flow to exercise the data path). Then map-merge viz across sessions
(overlay the prior `.smap` landmarks) + split slamko_mapping when the I/O grows.

## 2026-05-27 — P4b-2: continuous relocalization in OK (deploy-correct localize) ✅

**What:** the supervisor now attempts welds in the **OK state** too (config `continuous_reloc`,
throttled by `continuous_reloc_interval` frames), gate-guarded as always. So a session
localizes into a prior map / closes a loop **without having to get lost first** — the
deploy-correct behavior (the forced loss in P4b-1 only existed to exercise the data path).
A weld in OK updates map→odom only; the state stays OK and the fast odometry is untouched
(Hard Rule #4). Default off → existing runs unchanged.

**GATE — session 2 on V1_01, prior map loaded, `continuous_reloc=true`, NO forced loss:**
```
loaded 1 prior submaps (live ids start at 1)
WELD to submap 0 [CROSS-SESSION/prior map]   ← t≈2s, NO seal, state stayed OK
map→odom t=[0.004 0.017 -0.005]              ← ~2 cm: recognized it's already in the known room
```
So the live session auto-localized into the prior map from startup. `check_neverlost.py`
**7/7 PASS** (seals 0, welds 1, ended OK via no-transition, anchor 0.02 m, ATE 6.9 cm).
Unit: `ContinuousRelocWeldsInOKWithoutLoss` (loop, 15 supervisor gtests 0 fail). The
auto-check learned the no-loss case (stayed-OK = success; "welds don't worsen ATE").

**P4b status: cross-session localization complete** — both reactive (forced-loss recovery,
P4b-1) and proactive (continuous reloc in OK, P4b-2). Next: cross-session merge VIZ (overlay
the prior `.smap` landmarks with the live map in one frame) + split `slamko_mapping`.

## 2026-05-27 — cross-session validated on a DIFFERENT trajectory (V1_02 → V1_01) ✅

The earlier P4b runs replayed the SAME sequence (V1_01→V1_01). The honest test is a
**different trajectory** localizing into a prior map. Built the V1_01 Atlas (`map_save_dir`),
then ran **V1_02_medium** (faster, different path, same Vicon room) with `prior_map_dir` +
`continuous_reloc`, no forced loss:
```
loaded 1 prior submaps
WELD to submap 0 [CROSS-SESSION/prior map]   ← t≈15s, OK state, map→odom [0.07 0.02 -0.40]
```
V1_02 recognized V1_01's room from a new viewpoint and anchored into its frame — a **real
41 cm** cross-session offset (vs ~2 cm for the identical replay). `check_neverlost.py`
**7/7 PASS** (seals 0, weld 1, anchor 0.41 m, ATE 25 cm). Merge plot: V1_02's live map
overlaps V1_01's prior map in the room. So prior-map relocalization works from a genuinely
new path — the deploy-relevant case. (No code change — validation only; reuses P4b.)

## 2026-05-27 — never-lost regression harness (one command) ✅

`scripts/bench_neverlost.sh [PRIOR_SEQ] [LIVE_SEQ]` runs the whole spine end-to-end:
session 1 maps + saves the Atlas, session 2 replays a DIFFERENT sequence, loads that map,
and localizes into it via continuous reloc — then `check_neverlost.py` gates it. Reuses
bench_ate.sh's zombie-guard (SIGINT-first reap so the map save + landmark dump flush).
Exit 0 = the full never-lost + cross-session pipeline works on real data. First green run
(V1_01→V1_02): 1 submap saved, cross-session weld YES, auto-check 7/7 PASS (anchor 0.37 m,
ATE 32.1 cm). This is the reproducible guard before P3 refactors the relocalizer.

## 2026-05-27 — P3a: BoW vocabulary + inverted-index database (scalable place-rec) ✅

The relocalizer's brute-force NN is O(maps × descriptors) — fine for a room, not for a
large map. `bow.{hpp,cpp}` adds the classic vocabulary + **inverted index**, hand-rolled
over XFeat's 64-d float descriptors (no DBoW2 dep; Eigen + std, depends on slamko_core only):
- **`BowVocabulary`** — K visual words via **deterministic** k-means++ (seeded init + Lloyd);
  `quantize` (nearest word), `transform` (L2-normalized TF bag-of-words).
- **`BowDatabase`** — per-submap TF BoW behind an inverted index (word → submaps), scored by
  **TF-IDF cosine**; `query()` returns the top-k candidate submaps in sublinear time (only
  visits submaps that share a word). It is a CANDIDATE retriever — geometry (the XFeat PnP
  gate) still verifies the weld.

**GATE — 4 gtests** (`colcon test slamko_loop`, now 38 total 0 fail): vocabulary recovers 3
separated clusters (distinct words); **deterministic** (same seed → bit-identical words);
database retrieves the right submap for a query drawn from a known place; empty/edge cases.

**Next — P3b:** wire `BowDatabase` into `XFeatRelocalizer` — train the vocab on the DB
descriptors, BoW-index each `addSubMap`, and in `relocalize()` PnP-verify only the top-k BoW
candidates instead of every submap. Validate it does NOT regress `bench_neverlost.sh` (the
guard) — same welds, faster. Then lift the `max_db_landmarks` brute-force cap.
