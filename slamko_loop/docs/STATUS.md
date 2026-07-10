# slamko_loop — Status log

Living, dated progress + numbers log. Plan: [`PLAN_P2_loop.md`](PLAN_P2_loop.md).

## 2026-07-09/10 — associateByProjection (welds por proyección) + veredicto del techo ✅ (commit 46d331a)

`XFeatRelocalizer::associateByProjection` (opt-in `projection_assoc` en el nodo): proyecta
landmarks del prior en la imagen del query vía la pose estimada, matchea por descriptor en
gate de píxeles, PnP → weld por el 3-tier. Correcto y validado negativamente: en EuRoC MH
cross-trayectoria el coseno XFeat de pares verdaderos < 0.7 entre viewpoints de vuelo
distintos (gate 150px/cos 0.70 → matches basura que PnP rechaza honestamente) — la
medición más pura del techo de viewpoint; en régimen terrestre same-viewpoint debería
rendir (A/B casa pendiente). Contexto completo: docs/RESEARCH_ATLAS_MULTISESSION_01.md +
docs/PLAN_CUVSLAM_MULTISESSION_01.md §5-6 (el mismo XFeat da 0.102 m en orbslam3_xfeat →
la variable es la maquinaria de fusión, no el descriptor).

## 2026-07-09 — catch-up entry for commit 220c130 (2026-06-30 session) ✅

Loop-side pieces of the immortal-gates session (documented centrally in
`docs/PIPELINE_STATUS_01.md` §0b + `slamko_ros/docs/STATUS.md`; closes the
check_doc_freshness gap): depth geometric loop weld integration (`depth_loop_refine`,
point-to-SDF ICP via sdf_registration, 7.2 cm validated) + supervisor gate wiring.

## 2026-06-26 — GTSAM pose-graph backend (LevenbergMarquardt) — A/B-validated vs Ceres ✅

The step toward the MASTER_PLAN P-C′ iSAM2 smoother (GTSAM is purpose-built for robotics factor
graphs). `optimize()` now dispatches on `PoseGraphConfig::backend` (`Ceres` default | `GtsamLM`):
- `src/pose_graph.cpp` `optimize()` → `optimizeCeres_()` (the renamed existing body) or
  `optimizeGtsam_()`.
- `src/pose_graph_gtsam.cpp` (new): builds the SAME factor graph in GTSAM — relative
  `BetweenFactor<Pose3>` (loops Huber-robust), cross-session `PriorFactor<Pose3>`, per-CONNECTED-
  COMPONENT gauge (identical union-find; fixed/anchor/auto-gauge nodes pinned by a σ≈1e-6 tight
  prior since batch LM has no constant variables) → `LevenbergMarquardtOptimizer`. **Load-bearing
  detail:** slamko's 6×6 information is `[trans;rot]`, GTSAM's Pose3 tangent is `[rot;trans]` — we
  PERMUTE the information block-wise so the whitened cost `r·I·r` is identical → both solvers
  minimise the same objective → same minimum.
- **Opt-in** `-DSLAMKO_LOOP_WITH_GTSAM=ON` (default OFF keeps the build Ceres-only + GTSAM-free; the
  .cpp still compiles as a stub that falls back to Ceres + warns when GtsamLM is requested).

**VALIDATED** (`test_pose_graph_gtsam`, built only with the flag): a drifted 6-node square loop +
loop closure optimised by BOTH backends → optimised absolute poses agree node-by-node to
**< 2 mm / < 2 mrad**; both reduce the cost. Default build (GTSAM OFF) green, existing
`test_pose_graph` 7/7 unaffected. GTSAM 4.2.0 (system). v1 limitation: yaw priors not yet ported to
the GTSAM factor (skipped + warned — use Ceres if compass yaw is active; gated OFF indoors anyway).
**EuRoC ATE A/B vs ground truth** (`tools/pose_graph_tum_ab`, MH_03 — real OKVIS odometry + 268 GT
revisit loops, 540 keyframes, Sim3-aligned ATE): baseline OKVIS **22.48 mm** → CERES **19.10 mm**
(6 iters) / GTSAM **19.14 mm** (3 iters). Ceres↔GTSAM = **0.04 mm ATE diff**, pose agreement max
1.32 mm / mean 0.53 mm → statistically identical trajectory; both reduce the OKVIS drift (the loops
redistribute it). **GTSAM converged in HALF the iterations** — the first hint of the efficiency
iSAM2 amplifies. So the backend is ATE-validated against real ground truth, not just numerically
matched on a synthetic loop.

## 2026-06-26 — iSAM2 INCREMENTAL backend — the real robotics win, demonstrated ✅

`PoseGraphBackend::GtsamISAM2` (`optimizeGtsamIsam2_()` in pose_graph_gtsam.cpp). Keeps a persistent
`gtsam::ISAM2` Bayes tree in an OPAQUE `isam2_state_` handle (header stays GTSAM-free, HR#2) +
counters of which edges/priors/nodes it has already absorbed. Each `optimize()` feeds ONLY the
factors added SINCE the last call → `isam.update()` relinearises just the affected sub-tree
(O(touched), not O(graph)). Gauge handled incrementally: a tight prior on each component ROOT (a
node that is never an edge's `to` — chain head OR Atlas-break island root) + fixed/anchor nodes.
`catch` → GLIM disposable principle: a corrupted update discards the tree and rebuilds batch (LM),
re-inits next call. `relinearizeThreshold=0.01, relinearizeSkip=1` (pose-graph-accurate).

**DEMONSTRATED** (`pose_graph_tum_ab --incremental`, MH_03, 540 keyframes fed one-by-one, each step
re-optimised):
| backend | total | per-step mean | first-10 | last-10 (graph 540× bigger) |
|---|---|---|---|---|
| **Ceres** (batch re-solve) | 2929 ms | 5.42 ms | 0.05 ms | **9.35 ms** ⬆ O(graph) |
| **iSAM2** (incremental) | **170 ms** | 0.31 ms | 0.14 ms | **0.24 ms** ≈ O(touched) |

**17× faster total; the per-step cost stays ~CONSTANT as the map grows** (Ceres' grows 187×) — the
lifelong-SLAM payoff (a million-pose map stays cheap to update). Batch ATE on MH_03 = **19.19 mm**,
identical to Ceres 19.10 / GtsamLM 19.14. Unit test `Isam2MatchesCeresOnADriftedLoop` green
(<5 mm/<5 mrad). v1 limitation: cosmetic — the iSAM2 path doesn't fill Result.initial/final_cost.

**Compass yaw-prior = native GTSAM factor (2026-06-26, commit d13507a).** `GtsamYawFactor` (unary on
Pose3, error = wrap(atan2(R10,R00) − target), 1×6 finite-diff Jacobian, robust-wrappable) in BOTH
the LM and incremental iSAM2 paths (iSAM2 tracks `yaw_applied`). NO more Ceres fall-back for compass.
Test `YawPriorMatchesCeres`: Ceres and GTSAM pull a node's heading 0.30→0 identically (<0.05 rad).
GTSAM is now auto-detected (CMake option default ON) and **iSAM2 is the live default**
(provider_fusion `pose_graph_backend=isam2`); the library `PoseGraphConfig` default stays Ceres.

**NEXT:** call `optimize()` per-keyframe live (today event-driven) to reap MORE of the O(touched)
win; a live GT-bag ATE A/B; iSAM2 multi-prior cross-session soak.

## 2026-06-26 — DENSE geometric channel: nvblox ESDF query + end-to-end (step 2) + HONEST finding

Step 2 done (commit pending): `VolumetricBackend::queryDistanceField(pts)->{dist,weight}`
(slamko_core contract) + `NvbloxBackend` impl (slamko_tsdf) — batch-samples the **ESDF**
(signed Euclidean distance, large smooth basin) via `mapper->updateEsdf()` + `esdf_layer().
getVoxels()` (the truncated TSDF's ±band was too small a basin; ESDF is the right field).
`registerToSdf` refactored to a BATCH functor (one GPU→host query per ICP iteration; per-point
wrapper kept for the analytic tests). End-to-end GPU self-test `slamko_tsdf/tools/
nvblox_sdf_selftest` (synthetic corner → integrate → query → ICP a drifted cloud).
**VALIDATED:** the ESDF query is CORRECT on real nvblox (probe z=1.8 → +0.220 m in front,
z=2.0 → 0.000 on the wall, behind → unobserved); the ICP CONVERGES and snaps the cloud onto
the mapped surfaces (rms → 0). **HONEST FINDING (the load-bearing one):** point-to-SDF ICP on
FLAT/symmetric geometry is **tangentially DEGENERATE** — it converges to rms 0 (cloud on the
surfaces) but at an AMBIGUOUS pose (full-SE3 err ~0.11 m on bare walls), because the SDF
constrains each point's surface NORMAL but not the in-plane slide; bare walls admit many
surface-fitting poses. So the dense channel is a **normal-direction drift REFINER that
COMPLEMENTS the appearance/feature channel (which constrains the tangential), NOT a standalone
6-DOF solver** — it shines on TEXTURED/cluttered real rooms, degenerates on bare planar scenes.
Gotcha: ESDF central-diff gradient needs a ≥2-voxel finite-diff step (1-voxel = discretization
noise → divergence). NEXT (step 3): live — register the depth cloud against the map BEFORE
integrating (avoid double-surface), FUSE the dense normal constraint WITH the appearance loop
(disjunctive/joint) so the tangential is covered, gate by rms+inliers. Answers the user's "match
the nvblox geometry" + the recovery-from-knock goal — honestly: dense refines normal, appearance
does tangential, together they recover.

## 2026-06-26 — P0 stability primitives: ImuShockDetector + ScanContext (Eigen-only, gtest)

Two more header-only primitives over `slamko_core` (Hard Rule #2), unit-tested:
- **`imu_shock.hpp` — `ImuShockDetector`** (P0.3, commit 5796e96): the kidnap/knock/drop trigger the
  quality-break MISSES (a clean lift/bump = NO pose speed-jump because the provider coasts on IMU,
  but the RAW IMU shows a jerk/accel/gyro spike). `feed(accel,gyro,dt,t)` → IMPACT / FREEFALL / YANK
  + refractory. Wired into `provider_fusion_node` (`use_imu_shock`) → `imu_shock_pending_` so the
  never-lost path seals+breaks on a knock; also feeds the P0.1 catastrophic hard-break. 5 gtests
  (calm stream = 0 false positives = no spurious seals).
- **`scan_context.hpp` — ScanContext** viewpoint-invariant polar ring/sector descriptor for
  VPR-independent place recognition. 5 gtests. (Honest scope: the recall limiter is viewpoint
  COVERAGE, not the descriptor — see the immortality reframe; ScanContext is a geometric second
  opinion, not a recall silver bullet.)

Both feed the universal evaluator (`docs/EVAL_SYSTEM_01.md`): IMU-shock = the inertial witness's
kidnap channel; the dense ICP = channel-7's live v2 hook. Suite `test_imu_shock` + `test_scan_context`
+ `test_sdf_registration` green.

## 2026-06-26 — DENSE geometric channel: point-to-SDF ICP primitive (step 1, unit-validated)

`include/slamko_loop/sdf_registration.hpp` (header-only, Eigen + slamko_core SE3). The STRONG
geometric verify/align channel the sparse ScanContext is not (user's call: "our submaps have
nvblox geometry — match and align with THAT"). At a revisit / after a brusque-motion or blackout
divergence, the robot is physically at a place whose surfaces are ALREADY in the nvblox TSDF/ESDF;
a depth cloud placed at the drifted pose sits offset from them, and minimizing the SDF value at
each point (residual = signed distance, 0 when aligned) snaps the cloud onto the existing geometry
— that snap IS the drift/loop correction. Viewpoint-robust where appearance + sparse fail (a room's
dense shape is the same regardless of facing, given shared structure). = OKVIS SubmapIcpError /
Voxgraph field-align (RESEARCH_LIFELONG_NAV_ARCH §5). Gauss-Newton on SE3 (twist [rho;omega], right
perturbation dx/dξ = R·[I|-[p]_×]), Huber kernel, correspondence gate, min-inlier guard. DECOUPLED
(Hard Rule #2): templated on a DistanceField functor `p->{dist,grad}`, so it unit-tests against an
analytic field AND the nvblox ESDF query (slamko_tsdf) plugs in behind the same functor.
**3 gtests green** (test_sdf_registration): RecoversKnownTransform (drifted corner cloud → recovers
the SE3 to <1e-2, rms <1e-3, >200 inliers), RejectsTooFewInliers (far cloud → not converged),
EmptyCloudSafe. NEXT (step 2): expose `queryDistanceField(points)->{dist,grad}` on the nvblox ESDF
in slamko_tsdf (VolumetricBackend); (step 3) provider_fusion runs registerToSdf of the live depth
cloud against the map BEFORE integrating the frame (avoid double-surface), gate by rms+inliers, add
the correction as a loop/prior constraint → this is the recovery-from-brusque-motion that needs a
recognized revisit. This is the strong answer to the recall limiter + the never-distorted goal.

## 2026-06-26 — Geometric loop channel: LIVE wired (step 3) + honest A/B finding

Steps 1-3 SHIPPED (descriptor → per-KF retrieval → live union with VPR; commits a50129e,
9610ab2, df1310b). `use_scan_context` param, query fed `R_world_body·(body_T_cam·p_cam)`,
`relocalize(query,query_pts)` UNIONs `geometricCandidates()` with the VPR top-N. Built in the
gravity-aligned WORLD frame (z-up, recentered at the KF — tilt-robust). 14 loop tests green.
**LIVE diagnostic CONFIRMS the channel is correct**: on casa (cuVSLAM provider) it is fed
(qpts≈200), per-KF kf_sc fully built (50/50 per submap), and it matches with ScanContext
distance **0.000–0.062** where geometric overlap exists (well under the 0.40 gate).
**HONEST A/B (CASA1_100cmH different-heading return, SC ON vs OFF): NO recall gain DEMONSTRATED
(5 = 5 verified loops, +0geom).** Two reasons, both expected: (1) the casa returns are either
same-heading (the appearance VPR already retrieves them → geometry correctly adds nothing new)
or a LARGE viewpoint change where the ~90° depth-cam FOV has too little overlap for EITHER
channel (the documented physics — geometry can't match what doesn't overlap); (2) the A/B was
contaminated by cuVSLAM's run-to-run divergence (provider |disp|max 8.3 m OFF vs 83.5 m ON — the
provider instability, independent of this channel). A clean gain needs a STABLE provider run +
a MODERATE-viewpoint-change return (partial overlap) — the regime these bags don't cleanly hit.
The feature is correct and committed; the recall-gain demo is pending the right bag + a stable
provider. (LiDAR/360° or a 2nd rear camera would widen the overlap envelope — the real lever.)

## 2026-06-26 — Geometric loop channel: ScanContext descriptor (step 1/N, unit-validated)

`include/slamko_loop/scan_context.hpp` (header-only, Eigen only — Hard Rule #2, no OpenCV). The
VIEWPOINT-INVARIANT geometric place descriptor that complements the appearance channel
(EigenPlaces/XFeat). WHY: every doc converges on recall = VIEWPOINT-coverage limiter (PIPELINE_STATUS §0,
RESEARCH_LIFELONG_NAV_ARCH §5); appearance retrieval dies on different-heading revisits (images don't
overlap), geometry doesn't. ScanContext (Kim 2018): polar bird's-eye matrix (n_ring×n_sector, max-height
per cell) matched yaw-invariantly by column shift; ring-key (row mean) = rotation-invariant fast prefilter.
Disjunctive loop gate target: accept if visual OR geometric passes. HONEST scope: built for 360° LiDAR; a
~90° depth-cam fills only its FOV → invariance over the OVERLAP → extends recall to MODERATE viewpoint
change (e.g. 90° turn); a TRUE 180°-opposite revisit has zero overlap = unmatchable by any method (physics).
Convention: points in a gravity-aligned frame (z up) — slamko's odom/world frame qualifies.
**5 gtests green** (test_scan_context): non-empty build, empty-on-no-range, YAW-INVARIANT same-place (90°
rotation → dist <0.15 + column shift recovers the yaw ±2 sectors + ring-key dist <0.05), different-scene
far (>0.25), identical=0.

**Step 2 DONE (relocalizer integration, unit-validated):** `XFeatRelocalizer` now stores a PER-KEYFRAME
ScanContext (`Entry.kf_sc`, aligned with `keyframes`) built at `addSubMap()` from each KF's observed
landmarks (kf_obs.landmark_ids → submap-local 3D → transformed into the KF body frame T_WB⁻¹). New public
`geometricCandidates(query_pts)` matches a query's local 3D yaw-invariantly (ring-key prefilter →
column-shift distance) against every stored per-KF descriptor and returns up to `sc_top_m` submap ids under
`sc_max_dist`, ranked. Config: `use_scan_context` (default OFF), `sc_top_m`, `sc_max_dist`, `sc_ring_gate`,
`sc_cfg` (incl. `sc_cfg.up` = body-frame gravity axis — ScanContext::compute rotates up→z so any frame
convention works; optical y-down → {0,-1,0}). Per-KF (not per-submap) because a ScanContext is a single-pose
descriptor and a submap spans 30–80 m — mirrors the per-KF VPR granularity. **3 new gtests green** (in
test_relocalizer): GeometricCandidatesYawInvariant (store room@0°, query room@90° → returns it, dist <0.4),
RejectsDifferentScene (corridor ≠ room → empty), OffByDefault. Gotcha fixed: ScanContext needs
`<Eigen/Geometry>` (cross product). NEXT (step 3): plumb the query's stereo-triangulated 3D + the provider
gravity-up into provider_fusion_node's reloc call; UNION geometricCandidates with the VPR top-N before
PnP-verify (accept if visual OR geometric); persist kf_sc in the .smap; validate the recall gain on a
different-heading revisit bag (the multi-DIRECTION revisit in the queue). Unlocked by cuVSLAM 120fps + live
45fps depth this session.

## 2026-06-21 — MapPointStore: persistent point identity (Phase A/B/C/D)

`include/slamko_loop/mappoint_store.hpp` — the ORB-SLAM3/PLVS abstraction slamko lacked: a global
store of MapPoints (global position + L2-normalised XFeat descriptor + `n_obs` maturity), header-only.
- `associate(pos, desc)` — drift-tolerant cross-submap data association (3×3×3 cell stencil, generous
  radius absorbs VIO drift, descriptor cosine is the discriminator). The Phase A dedup.
- `refine(id, pos, desc)` — multi-view running-mean consensus (position + descriptor) + bump n_obs.
  The Phase B "re-observe → tighten".
- `add(..., n_obs)` / `nObs(id)` / `position(id)` — the cross-session seed (Phase C) + back-propagation
  and maturity persist (Phase D).
Used by `slamko_ros::provider_fusion_node` to kill the revisit doubling (numbers there + in
[`../../docs/PLAN_PERSISTENT_MAPPOINTS_02.md`](../../docs/PLAN_PERSISTENT_MAPPOINTS_02.md)).
`test/test_mappoint_store.cpp` 3/3 (associate + running-mean refine + null-safety); **suite 20/0**.

## 2026-06-04 — Thin global pose-graph + offline loop-closure driver (branch klt-fork-loopclosure)

Built the clean "loop closure + optimization, separate" layer (the disposable global
graph). Depends only on `slamko_core` contracts + Ceres (Hard Rule #2).

- **`pose_graph.{hpp,cpp}`** — Ceres SE3 pose-graph: SE3 nodes, relative-pose odometry
  edges + robust-Huber loop edges, gauge anchor, `optimize()`. Solver encapsulated in the
  .cpp; header solver-free. **`test_pose_graph` 3/3 green**: closes a synthetic drifted
  24-pose loop (gap 0.018, error collapses), weld-math + anchor-invariance proven.
- **`tools/loop_offline.cpp`** — offline driver: `loadSubMaps` → odometry pose-graph
  (gid=(submap<<32)|kf) → `XFeatRelocalizer` incremental causal detection (DB only holds
  submaps ≥`min_gap` older → every match is a genuine revisit) → weld
  `T_from_to=T_query_match⁻¹·matchedKF.T_WB` → `addLoopEdge` → optimize → corrected.tum +
  metrics. `--query_stride` decouples loop-detection cost from over-dense odometry KFs.
- **`tools/smap_info.cpp`** (archive inspector + VPR-coverage hard gate) + **`smap_cloud.cpp`**
  (global point-cloud export, anchor·local).
- **Validated on magistrale1** (578 submaps, 14429 KFs, VPR 100%): 175 loops detected +
  geometrically verified (inliers 40-147, VPR cos 0.7-0.95), pose-graph converged
  (cost 26.7k→11.6k). Machinery works end-to-end. Open: end→start BRIDGE doesn't form
  (single-best matching → disconnected start/end clusters; 0 loops submap>500→<100) → only
  14% of the 26.5m gap closed. NEXT: multi-candidate acceptance + submap culling. See memory
  `slamko-lifelong-build-roadmap`.

## 2026-05-29 — Deleted the dead anchor-era machinery; shrank to the relocalizer front-end (dormant)

The okvis-arch-refactor forced the live trajectory to PURE VIO (anchors = identity), so the
never-lost SUPERVISOR / anchor layer / homegrown pose-graph never reached `/tf`, `/odom`, or
the pose dump — dead on the live path (and documented as a 7× precision destroyer + the 478 cm
boundary-jump generator). The async SessionGraph (slamko_fusion) likewise injected 10–122 cm
jitter without loops and its loop-use path was inert (recall is the real bottleneck — see the
2026-05-28 entries). **Deleted** (≈2.5k LOC): `never_lost_supervisor`, `anchor_gate`,
`pose_graph`, `submap_archive`, `supervisor_state` + tests (`test_supervisor`, `test_stress`,
`test_pose_graph`); and `slamko_fusion::SessionGraph`. `vio_node` rewritten to the clean
minimal path (params → Tier-2 estimator → publish + pose dump); its `slamko_loop` dependency
dropped.

**KEPT, DORMANT (the loop-closure front-end for Phase C):** `XFeatRelocalizer` + `bow` +
`lightglue_matcher` (+ tests `test_relocalizer`, `test_bow`, `test_lightglue`). Still compiled,
not wired into the node. The PnP + LightGlue verifier was proven perfect (100% positive
control); only VPR RETRIEVAL fails — Phase C re-wires this stack with a learned VPR head
(SALAD/MixVPR/NetVLAD) for recall + a thin TrackingMonitor for recovery.

**Validation:** full gtest suite green (core 29 + fusion 6 + loop 12 + vio 17, 0 failures);
production Ceres MH_01 unchanged after the deletion (Sim3-ATE 6.9 cm, 3419 poses). `main`
untouched. See `slamko_fusion/docs/STATUS.md` (2026-05-29) for the estimator decision.

## 2026-05-28 (night) — Phase V.1: per-KF VPR granularity (SMP4)

**Shipped:** `XFeatRelocalizer` now ranks VPR candidates by **per-keyframe** cosine, not per-submap.
Each Entry caches a `kf_global_desc[]` aligned 1:1 with the submap's `keyframes`; the candidate
score is `max_k cosine(query.global_descriptor, kf_global_desc[k])`. Per-submap `global_desc`
remains as the SMP3-fallback path so legacy Atlases keep relocalizing after the schema bump.

**Schema:** `slamko_core` codec SMP3 → **SMP4** — additive trailer on each `kf_obs` block carries
`kf_gdim · floats` (the per-KF EigenPlaces vector). Codec accepts SMP1/SMP2/SMP3/SMP4; older
versions load with per-KF descriptors empty (graceful).

**Tests:** 3 new gtests pass:
- `SubMapIO.PerKeyframeVprRoundTrip` — distinct per-KF descriptors round-trip bit-exact.
- `XFeatRelocalizer.VprPerKfTopNRanking` — with `vpr_top_n=1`, the correct submap surfaces by
  per-KF cosine (registered AFTER two distractors, so order doesn't help). The magistrale-return
  regression guard.
- `XFeatRelocalizer.VprPerSubmapFallback` — SMP3-style legacy maps (per-submap descriptor only)
  still rank correctly.

**EuRoC V1_01→V1_02 cross-session smoke:** PASS, corrected ATE 27.1 cm (no regression vs the
prior per-submap path).

**magistrale1 (1500 s replay, 88 submaps sealed):** 5 welds — **all in the first 90 s** (start-
room re-visits at submap 0/1/3), **0 on the return**. Same shape as the pre-V.1 baseline (6
welds, 0 returns). Per-KF VPR is now the substrate (architecture done, finer granularity
available) but **didn't move the magistrale needle on its own**. Next step (V.2): diagnostic
dump of per-KF cosine scores + reloc-attempt counts during the return to decide whether
granularity is insufficient (→ swap model: AnyLoc / SALAD / MixVPR) or the relocalizer never
gets asked (supervisor throttle / state issue). See `docs/PLAN_BA_GLOBAL.md` V.1 + V.2.

## 2026-05-28 — LighterGlue verify (rescue mode) + two methodology walls found

**Built (the verification fix from PLAN_VPR_RELOC.md):** `LightGlueMatcher` (slamko_loop) — the
verlab XFeat-64 LightGlue (`lighterglue.pt`) via libtorch TorchScript, implementing
`slamko::Matcher`. Optional target `-DSLAMKO_LOOP_WITH_TORCH=ON` (default OFF → no-op fallback,
header torch-free via pimpl, default build + gtests stay torch-free). Adapts the AirSLAM_XFEAT
`lighter_glue.cc` port to the slamko Features layout (N×3 kpts, N×64 desc), top-K to N=512 with
index mapping back to original rows. **Tests: 49 gtests 0 fail** (torch-free) + 2 guarded
LighterGlue tests (load `.pt`, identity-view match, top-K validity) when torch ON.

**Wired into `XFeatRelocalizer` as a RESCUE verifier (not the default):** the SubMap had no 2D
keypoints, so a candidate's landmarks are projected into its keyframe poses (now populated by
`vio_pipeline::buildSubMap` — `kf_poses_` tagged by epoch) to synthesize train views, matched
against the live query. **Order matters and was the key fix:** brute-force NN runs FIRST (matches
the whole cloud → hundreds of correspondences → accurate PnP); LighterGlue fires ONLY when
brute-force can't verify a candidate (the hard revisit). This guarantees the build is **≥ the
brute-force baseline by construction** (it only ADDs closures NN missed, never overrides a good
weld). A first verify-FIRST attempt regressed (suppressed a good brute-force weld + sparse
synthetic-view matches) — corrected. Node params `reloc_use_lightglue`/`reloc_lightglue_model`/
`reloc_lg_max_views` + launch args; model ships in `slamko_vio/models/` (gitignored `.pt`).

**Two walls found that block ATE-gated loop-closure work (both bigger than LighterGlue):**

1. **TUM VI magistrale GT is room-only** (see memory `slamko-tumvi-magistrale-gt-roomonly`). The
   `gt.tum` covers ONLY the start/end mocap room (~2 m box, path 97.8 m — identical extent to
   room1) even though the walk is ~822 m. Naive full-traj ATE compares a ~50 m estimate to a 2 m
   GT → **absurd scale 0.006–0.20**, SE3-ATE 1.9–22 m = pure GT artifact. **The "magistrale1
   baseline 1.9 m" in the prior entries is therefore meaningless**; the OKVIS 8.6 cm must have used
   valid-segment-only eval. → use EuRoC (full GT) or build a valid-segment GT mask.

2. **VIO replay is non-deterministic** — on EuRoC V1_03_difficult, the SAME code+config gives
   raw-odom SE3-ATE **52.9 cm vs 74.7 cm vs 95.3 cm** across runs (~40–80% variance; ~all frames
   processed, so not frame drops). **→ FIXED same day** (IMU↔frame gating in vio_node; root cause
   was the rclcpp imu-vs-stereo callback-order race dropping late IMU samples). Two identical V1_03
   runs now within **1.2 cm**; ATE A/B is now trustworthy. See slamko_vio/docs/STATUS.md
   2026-05-28 + memory `slamko-vio-replay-nondeterministic`.

**VIO confirmed HEALTHY though:** EuRoC MH_05 raw-odom Sim3 **scale 1.0127** (metric), SE3-ATE
~18–22 cm. So slamko_vio is sound; magistrale's scale weirdness was 100% the GT. On V1_03 the
welds DO help (raw→corrected 74.7→38.5, 95.3→48.4) — loop closure works, just unmeasurably-A/B'd.

**Next (ranked):** (a) **VIO determinism/stability** (the real "stable" blocker — deterministic
GPU + solver, robustness on hard seq); (b) **measurable hard-revisit GT** (mask magistrale to
in-room segments, or a synthetic hard-revisit bag); (c) **per-keyframe real 2D features** in SubMap
(store each KF's keypoints+descriptors+landmark links → real two-image LightGlue, in-distribution)
— the synthetic projected-cloud train view is too sparse to rescue the hard magistrale return
(0 late welds), and per-keyframe matching is the correct LightGlue usage. Then #1 global landmark BA.

## 2026-05-28 — Loop-closure recall: XFeat carries NO place signal (config exhausted → need global VPR)

**Finding (definitive, proven offline + 3 live magistrale1 runs):** the loop-closure bottleneck is
NOT matcher config. Exposed the relocalizer knobs as node params (`reloc_match_ratio`,
`reloc_use_bow`, `reloc_bow_top_k`, `reloc_mutual_check`, `reloc_min_inlier_ratio`,
`reloc_min_inliers`) + added a mutual/cross-check matcher option + an inlier-ratio precision gate,
then swept them. Every config (Lowe 0.9, top_k 25, mutual-NN, use_bow=false/all-submap PnP) still
produced **0 real loop closures on the start-room return**.

Offline proof on the saved Atlas (submap 0 = start vs submaps 76-80 = genuine return): geometric
inliers <1%, **indistinguishable from places 80 m away**; positive control (sm0 vs sm0 under a known
SE3) = **100%** → the XFeat-NN + PnP-RANSAC geometric back-end is perfect, only RETRIEVAL fails.
Root cause: XFeat-64 is so self-similar (intra-set NN cosine 0.92) it has no place-level
discriminability (genuine-return matched cosine 0.79 ≈ far-place 0.77). **Config-fixable = NO.**

**Next (the real fix):** a global VPR front-end (learned NetVLAD/CosPlace/SALAD head, or a
binary-descriptor + DBoW2 path with a NON-GPL vocab — ORB-SLAM's is GPL, slamko is Apache/BSD) for
per-keyframe retrieval → feed the existing XFeat+PnP verifier. The auto-seal + chain-distribution
machinery (2026-05-27) is the substrate, ready for when retrieval fires. Benchmark target:
OKVIS2-X 8.6 cm / 63 closures (BRISK-binary + pretrained DBoW2 + distance-scaled drift gate).

## 2026-05-27 — Loop closure on a clean long traversal: auto-seal + chain distribution; recall is the bottleneck

**What:** to close loops on a long CLEAN-tracking traversal (no loss → previously 0 seals
→ nothing to relocalize against), added (1) **periodic auto-sealing** (`auto_seal_distance_m`,
OK-state voluntary checkpoint; branch inherits the anchor so map→odom is continuous),
(2) **weld-once per (target, active-submap)** (clear the set on each branch, so a later
revisit by a new active submap can re-close to an early submap — the first active welds the
start submap trivially and would otherwise block the real return closure), (3) **chain
distribution** in the pose-graph weld path: all sealed submaps as nodes + identity sequential
edges + stored loop edges, rebuilt and `optimize()`d each weld so a closure spreads across the
whole chain instead of re-anchoring only the active submap. Off-by-default (auto_seal=0 → the
validated V1_01 incremental path is byte-identical). Tests: **+3** (2 auto-seal trigger, 1
pose-graph chain-distribution-uniformity) → slamko_loop **41 gtests 0 fail**.

**Validated on TUM VI magistrale1 (822 m, returns to the start room) vs OKVIS2-X benchmark:**
- **OKVIS2-X** (sparse `okvis_app_synchronous`, DBoW2 loop closure): **SE3-ATE 8.6 cm**, scale
  0.994, **63 loop closures** — loop CLOSED (end-room ATE ≈ start-room ATE).
- **slamko full-fix**: auto-seal made 88 submaps, but the relocalizer fired only **6 welds, all
  trivial near the start, ZERO on the return** → loop did NOT close, SE3-ATE ~8 m.
- **Finding:** the auto-seal + chain machinery is correct and unit-validated, but the
  **bottleneck is relocalization RECALL** — slamko's XFeat + hand-rolled BoW (trained on ONE
  submap) + PnP (`weld_min_inliers=15`) does not recognize the start room on return, whereas
  OKVIS's DBoW2 + RANSAC + distance-scaled drift gate recalls it 63×. Next: attack recall
  (vocabulary trained over the whole map / persisted, matching+inlier thresholds, a
  distance-scaled weld gate). The pose-graph distribution is ready for when welds fire.

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

## 2026-05-27 — P3b: BoW candidate pre-selection wired into XFeatRelocalizer ✅

`XFeatRelocalizer` now trains a `BowVocabulary` on the first registered submap's
descriptors (the prior map / first sealed room — representative), BoW-indexes every
`addSubMap` in a `BowDatabase`, and at `relocalize()` PnP-verifies only the **top-k BoW
candidate submaps** instead of every submap. **Fallback-safe:** if the vocab is untrained
(submap < `bow_vocab_size`) or no candidate shares a word, it falls back to all-submaps —
so recall is never reduced, only hopeless submaps skipped. `use_bow` default on.

**GATE — bench_neverlost.sh (V1_01 → V1_02, BoW active), 7/7 PASS:** session 2 still fires
`WELD to submap 0 [CROSS-SESSION]` (anchor 0.40 m, ATE 28.7 cm) — i.e. the BoW pre-selection
did NOT regress the cross-session weld. Relocalizer + BoW unit suites green (38 loop gtests).

**Harness lesson (re-learned hard):** running validation runs CONCURRENTLY makes the
global `pkill -f slamko_vio_node|euroc_player` reap each other (CLAUDE.md "run serially"),
and a `setsid`-detached launch survives if its bench is killed before teardown → a duplicate
publisher (`TF_OLD_DATA` flood) that corrupts the next run. Fixes: `bench_neverlost.sh` now
`trap reap EXIT INT TERM` (teardown always runs) + the reap loop waits on BOTH process names;
**run benches one at a time.** Most of the P3b "failures" were these races, not the code.

**Next:** with BoW pre-selecting a few candidates, the per-submap brute-force only runs on
the top-k, so the `max_db_landmarks` cap can be raised (more recall per candidate) — a cheap
follow-on. Then a persisted/pre-trained vocabulary (so it's not re-trained per session).

---

## 2026-06-12 — P-B step 1: VPR recall diagnostic — EigenPlaces VERDICT: GO (top-N + verify)

New tool `tools/vpr_recall_diag.cpp` (per-KF cosine recall over smap archives,
submap-index OR GT-time split) + `scripts/vpr_recall_frames.py` (same metrics
directly on dataset frames, no VIO in the loop, onnxruntime + eigenplaces.onnx).

**Root-cause chain on the magistrale1 start↔end bridge (the P-B gate):**
1. smap archive `perkfvpr_mag1_diag` (76 submaps, VPR 100%) gives R@10=0.000 —
   but the archive's KFs start at t=777.6 while GT says the start room ends at
   t=707.2: **the deprecated VIO ate 133 s initializing, the start room never
   entered the map.** The historical "didn't close" was a DATA hole, not (only)
   a recall failure. Un-measurable from smaps; measured from frames instead.
2. Frame-level, honest setup (DB = GT start-room window ≤707.2, queries = GT
   return window ≥1357.8, distractors = the full 650 s corridor trek, stride 10
   ≈ 2 Hz, 127 DB / 1301 distractors / 117 queries):
   **R@1=0.316 · R@5=0.846 · R@10=0.949 · median rank of first start-room
   hit = 3 (p90=8, worst=32)**; best_db median cos 0.394 vs best_distractor
   0.468 (margin −0.046).

**Verdict:** EigenPlaces is sufficient — no SALAD/CosPlace fallback needed —
**iff** the relocalizer (a) retrieves top-N ≥ 10 candidates (not top-1), (b)
geometric-verifies each (XFeat/LighterGlue + PnP), and (c) attempts on every
return KF (117 queries ≈ 59 s of return: per-frame R@5=0.85 compounds to
practical certainty). Absolute-threshold designs are DEAD (distractor cosines
exceed true-match cosines) — ranking+verify only, as PLAN_VPR_RELOC said.

**Design consequences for P-B step 2 (wire into the provider chain):** reloc
attempts continuous per-KF; candidate budget 10; the provider chain must
capture KF images from t≈0 (OKVIS inits in seconds — the 133 s hole was a
deprecated-VIO artifact and is gone by architecture).
