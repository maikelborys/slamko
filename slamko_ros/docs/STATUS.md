# slamko_ros — STATUS (validated facts + numbers)

<!-- validated: 2026-06-19 · tests: GAP-2 CULL BACKSTOP — revisit grows 0 submaps (was 9), fresh pass culls 0; the immortality plateau -->

## 2026-06-19 — GAP-2 cull backstop: the immortality CEILING (map bounded by AREA)

**Why (proven, the immortality blocker):** a 9-visit study showed dup-suppression slows growth
but does NOT stop it — ~3.5 submaps LEAK every visit (blind spots whose VPR recall is too low to
suppress), extrapolating to **~112 submaps / ~110k lm at 30 visits for ONE house**, linear, no
plateau (`results/r01/lifelong/growth_9visits.png`). Recall-improvement alone can't guarantee a
ceiling (a hard tail always leaks). The guarantee is a **culling backstop** = ORB-SLAM's
`KeyFrameCulling` lifted to submap level, but GEOMETRIC (robust to recall).

**What landed** (`provider_fusion_node.cpp` `sealSubmap` + `occKey`): a real-world voxel
**occupancy** set (`occ_`, voxel `cull_voxel_m`=0.10), seeded from the prior map and grown by
every KEPT submap. At seal, if `cull_redundant_frac`=0.7 of a submap's landmarks fall in already
-occupied voxels, the submap is **CULLED** — the seal is rolled back verbatim (id, anchor edges,
prev-anchor, loss flag all restored from a snapshot; nothing persisted/registered). GEOMETRIC, so
it bounds the map by AREA regardless of VPR recall. `cull_enabled`=false restores old behavior.

**Validated** (CASA1_Suave, rate 0.5):

| run | submaps kept | culled | meaning |
|---|---|---|---|
| **fresh pass** (no prior, SAFETY) | 9 (full map) | **0** | forward motion never false-culls |
| **revisit** (prior=fresh map, EFFICACY) | **0** | 9 | revisit adds NOTHING — map bounded by area |

Reloc intact on the revisit (34 re-anchors), fused-vs-provider PASS (0.000 m). Combined with
dup-suppression (appearance, live) + maturation (refine prior), slamko now has BOTH ORB-SLAM nets:
reuse + cull.

**HONEST multi-visit result (correction — the single revisit's 0-kept was a best-case alignment).**
A 4-visit cull-ON run (acc prior grows between visits) gives acc submaps 9→10→11→13→14 = **~1.25
new submaps/visit** vs **3.56/visit WITHOUT cull → cull cuts growth ~65%** (30-visit projection ~45
vs ~112), but it does **NOT fully flatten** at frac=0.7/voxel=0.10. Residual leak = segments where
BOTH nets miss: appearance recall low AND geometric redundancy <0.7 (from `T_global_map_` frame
drift + genuinely-new-viewpoint landmarks). `results/r02_cull/immortal_ceiling.png`.

**TRIED AND REJECTED (2026-06-19) — spatial drift-tolerance is the WRONG lever.** Added a 3x3x3
neighbour-voxel occupancy test (`occContains`, ~30 cm tolerance) to absorb the frame drift and
catch the leak. It FALSE-CULLED legitimate new map on the fresh pass: 9→6 submaps (lost 1/3 of the
house). Adding **age-gating** (occupancy only from submaps older than the loop gap, so the forward
seam is excluded — the principled ORB covisibility/loop-gap idea) reduced false-culls 3→1 but did
NOT eliminate them: indoors you pass WITHIN 30 cm of old-but-aged areas without truly revisiting,
so any spatial dilation over-culls. Both reverted; the committed single-voxel cull (0.10/0.7) stays
(safe: 0 false-cull). **The real plateau closer is FUSE-ON-CULL** — merge the leaked segment's NEW
landmarks into the overlapping existing submap (ORB SearchAndFuse; needs descriptor/kf_obs-consistent
merge), NOT spatial dilation.

**LANDMARK-LEVEL data-association cull SHIPPED (2026-06-19, commit 3ad33fa) — the real ORB bound.**
Moved the cull from whole-submap to PER-LANDMARK inside the dedup phase: each merged landmark already
in the global occupancy is dropped (known point), only genuinely-NEW points are kept + added to occ;
a submap is dropped whole only if ≥`cull_redundant_frac` of its deduped landmarks were already mapped.
This is ORB SearchAndFuse at landmark granularity (poses-fixed) — a PARTIAL revisit keeps its new
sliver + drops the redundant bulk, so map CONTENT is bounded by AREA. Validated CASA1_Suave: fresh =
9 submaps kept, 0 whole-culls, cross-submap seam dedup works (kept 1328 new / culled 574 per seam),
6 loops; **a FULL revisit (visit 2) added only 8 landmarks (~94% landmark-growth cut).** Asymptote:
occupancy is a voxel grid over a FINITE house → finite cells → after enough visits every reachable
cell is filled → 0 new landmarks → PLATEAU (10 or 1000 visits converge to the same bounded map).
**Honest residual = run-to-run FRAME DRIFT:** the single 0.10 m voxel is intolerant, so visits with a
poorer re-anchor (drift ~voxel) leak 1.2–1.6 k landmarks (visits 3/5) while a good-align visit leaks 8.
The drift envelope still saturates (bounded) but the plateau is higher + noisier than ideal. Next
lever: reduce drift (better per-submap re-anchor) or a modestly coarser cull voxel (drift-tolerant
without the 30 cm 3×3×3 over-cull that was rejected).

**DRIFT-TOLERANT VOXEL 0.10→0.15 (2026-06-19, commit f193c1a) — cuts the leak ~64%.** Coarsening
ONLY the redundancy-test voxel (stored-map dedup stays 0.06) tolerates the ~10 cm frame drift.
Validated CASA1_Suave 4-visit: landmark growth **+8.7%/visit (0.10) → +3.1%/visit (0.15)**; submaps
1.5→1.25/visit; **reloc fully healthy (35 re-anchors/visit)**. Fresh pass culls 1 end segment = the
bag's in-session loop return (correct, not false-cull; map keeps 8 submaps/8794 lm, 6 loops).
`results/r04_v15/bounded.png`. The map is now ORB-SLAM-style bounded by AREA (data-association +
drift-tolerant occupancy); the small remaining +3%/visit is residual drift that saturates (finite
voxels). 3×3×3 neighbour-dilation stays rejected (over-culls); 0.15 single-cell is the sweet spot.

## 2026-06-19 — GAP-2 maturation V1: prior map REFINED on revisit (task #4)

**5-visit growth study first** (`scripts/lifelong_visits.sh`, `results/r01/lifelong/growth.png`):
visiting the SAME house 5× against an ACCUMULATING prior, dup-suppression slows growth
~58% (accumulated submaps 9→24 vs 9→45 without) but does NOT stop it — new submaps/visit
stays ~4-5 (not →0). Root cause: blind-spot regions re-seal duplicates every visit because
reloc RECALL is too sparse there to mark coverage (the known P-B bottleneck), and each
duplicate is added to the prior. When reloc fires well (visit 3) growth nearly stops (1
sealed, 7 suppressed). **Ceiling = reloc recall, not the suppression logic.**

**Maturation V1** (`provider_fusion_node.cpp` `finalizeMaturation`): when a revisit segment
is SUPPRESSED as a duplicate of a PRIOR submap, its landmarks (in the prior's global frame)
are buffered keyed by that submap; at shutdown the prior archive is reloaded and each
covered submap's EXISTING landmarks are REFINED toward the revisit observations (voxel
nearest-match, position nudged by `mature_alpha`=0.2 toward the multi-session consensus —
structure-only, descriptors/kf_obs untouched). Matured archive → `mature_out_dir`. Params
`mature_enabled`, `mature_voxel_m`=0.06, `mature_alpha`=0.2.

**Validated** (CASA1_Suave revisit, prior = pass-1 map): refined **1805 landmarks** across
the **2 covered submaps** (3: 1356, 7: 449), mean shift **0.8-1.0 cm** (max 3.5 cm — sane
nudge, not corruption); **all 9 submap sizes byte-for-byte identical landmark counts** (map
matures, does NOT grow). Viz `results/r01/lifelong/maturation.png`.

**Honest limit (V1 vs V2):** V1 refines existing positions only → it does NOT add the
revisit's new descriptored landmarks, so it does NOT fix blind-spot reloc recall (the
growth driver in the 5-visit study). V2 = merge revisit descriptors/landmarks into the
prior submap (needs descriptor-block + kf_obs consistency — risk of corrupting the submap
if done naively) to raise recall and converge growth to 0. That is the real bounded-growth
closer (slamko_mapping P-C′+).

## 2026-06-19 — GAP-2 immortality brick: "don't re-map what you already see" (task #4)

**The problem (measured, the immortality blocker):** replaying CASA1_Suave with its
OWN pass-1 map as prior, the system RELOCALIZED PERFECTLY — 28 re-anchors, inliers
280-301, sub-cm — yet still sealed **9 duplicate submaps (+12.9k lm)** of the same
house. The merge info was served by every reloc match but UNUSED. 10× replay → 90
submaps, ~128k lm, all one house (architecture audit, `docs/architecture_is_vs_should.dot`).

**What landed** (`provider_fusion_node.cpp`): a coverage tally + duplicate-seal
suppression. Each confident match to an EXISTING submap (`markCoverage`, inliers ≥
`dup_min_inliers`=60) extends a "covered" window (`dup_cover_window_s`=4 s); a KF that
arrives inside it counts as covered. At the seal trigger, if ≥ `dup_cover_frac`=0.6 of
the segment's KFs were covered AND a prior map exists, the seal is **SUPPRESSED** (the
pending KFs are dropped, no submap persisted, no relocalizer registration) instead of
baking a duplicate. `dup_suppress`=false restores old behavior. The live trajectory /
TF are untouched (graph nodes stay; only the redundant submap is not persisted).

**Validated** (CASA1_Suave revisit, prior = own pass-1 map, rate 0.5):

| revisit | new submaps | new landmarks | reloc |
|---|---|---|---|
| WITHOUT suppression | 9 | ~12,870 | 28 re-anchors |
| WITH suppression | **4** (4 suppressed) | **~5,870** | 28 re-anchors |

**−54% map growth on revisited ground, reloc & trajectory intact** (fused max 0.15 m;
the P-A fused-vs-provider "FAIL" is by-design with VPR on). The 4 suppressed segments
were recognized as already covered by prior submaps 2/3/7. Viz `scripts/plot_immortal.py`
(`results/r01/lifelong/immortal.png`).

**Honest limit (first brick, not the whole package):** suppression is conservative —
it only fires where re-anchor confidence is SUSTAINED (≥60% of the segment). The 4
sealed submaps are where matches went sparse (coverage lapsed) — kept rather than risk
dropping genuinely-new territory. Full bounded growth needs the richer step: **merge**
new observations into the matched prior submap (maturation), which also handles partial
coverage. That is the `slamko_mapping` summarization work (P-C′+).

## 2026-06-19 — R0.1 DR-gate instrument: OKVIS across-gap motion vs independent DR (task #10)

**The task #10 reframe (load-bearing):** the soft chain edge is NOT identity+huge-cov
during a loss — OKVIS internally bridges the gap with its own IMU (measured: never
resets, holds warm state). So an independent dead-reckoning source's job is not to
*fill* the soft edge but to **GATE** it: a second, independent opinion on the
across-gap motion. Disagreement ⇒ OKVIS's bridge is suspect ⇒ that is the R0
"never ingest garbage" gate (PLAN_ROBUSTNESS_01 ordered R0 gates **before** R1.1
anchor edges). This pass builds the instrument and runs R0.1 (observe, don't gate yet).

**What landed** (`provider_fusion_node.cpp`): an independent DR channel — `onImu`
integrates **gyro only** (world←body SO3; accel is DOUBLED on bno_ab bags, never
touched) from `/camera/camera/imu`; translation comes from **coasting** the last
trustworthy OKVIS body twist. At every stale-gap the across-gap OKVIS relative
motion is compared to the DR estimate and logged to `<out>/dr_gate.csv`
(`rel_t,gap_s,d_rot_deg,d_trans_m,okvis_trans_m,coast_trans_m`). Pure instrumentation
— zero behavior change. Viz `scripts/plot_dr_gate.py` (`results/r01/dr_gate.png`).

**Validated** (CASA1_Suave, rate 0.5, FORCE_LOSS sweep + natural gaps):

| gap | OKVIS trans | coast trans | **d_rot** | d_trans |
|---|---|---|---|---|
| 1.05 s (natural) | 0.49 m | 0.91 m | **2.5°** | 0.45 m |
| 3.03 s (forced)  | 1.39 m | 1.85 m | **4.2°** | 1.07 m |
| 6.02 s (forced)  | 1.53 m | 3.29 m | **1.3°** | 2.05 m |

**Finding:** rotation is the **trustworthy gate channel** — gyro-vs-OKVIS disagrees
only **1.3–4.2°** even over a **6 s** loss ⇒ OKVIS's IMU-bridge rotation is sound;
a provisional R0 gate at ~15° has wide margin (correctly PASSes all three). The
**translation coast is too crude to gate on** — constant-velocity over-shoots on a
curving path (3.29 vs 1.53 m @6 s is the model, not a broken bridge). Upper gate
threshold still needs a **negative** (a genuinely broken bridge) — OKVIS won't
produce one naturally; next R0.1 step is an adversarial injection. **NOT a regression:**
the bench's P-A fused-vs-provider gate "FAILs" by design with VPR on (loops correct
the fused trajectory away from raw provider).

## 2026-06-19 — R0.1 map cleanup: ORB-SLAM dedup+cull adopted (task #9)

Adopted ORB-SLAM3's map-quality techniques into `sealSubmap()`, adapted to
slamko's loose-over-OKVIS poses-FIXED architecture (structure-only, NO joint BA —
the metric estimation stays in the provider). The R0.1 campaign had shown
slamko's per-KF one-shot stereo triangulation produces **2-4x duplicates** (same
feature triangulated independently each KF) + **"ray" artifacts** (far,
depth-uncertain points smeared along the camera ray, σ_z ∝ z²).

**What landed** (`provider_fusion_node.cpp` sealSubmap; params `lm_dedup_voxel_m`
=0.06, `lm_min_obs`=2): voxel-hash dedup of submap-local landmarks — the same
physical feature from N keyframes lands in ONE voxel → merged to its **centroid**
(multi-view refine) with obs count N; a "ray" has its per-KF depth noise SPREAD
across voxels → 1 hit each → **culled** by `lm_min_obs`. One O(N) pass at seal
(per-KF path untouched → speed unchanged). Provider-agnostic (uses `graph_.pose()`,
not OKVIS internals → works for any future provider).

**Validated** (CASA1_Suave golden, rate 0.5): per-submap ~10k→~1.5k lm (**~6.2x
dedup**), run total **79,945 → 12,855**. Reloc recall NOT hurt — **improved**:
6 loops closed (vs 2), inliers 75-123 (vs 59-63) — cleaner unique points match
better in PnP. Rays visually gone (`results/r0/dedup_before_after.png`). vs
ORB-SLAM3+XFeat on the same bag (44k, but stereo-only / not gravity-aligned).

**Next (task #9 phase 2):** optional DLT multi-view re-triangulation per merged
point + σ_z weighting (robust far-landmarks indoor AND outdoor, unlike a depth
gate). Tune `lm_min_obs` if coverage on short-visibility features drops.

## 2026-06-19 — R1 inter-map anchor edges (soft + hard) recorded + visualized

The "federation of islands" connections (PLAN_ROBUSTNESS_01 R1.1/R1.3), the user's
two-edge design: *don't propagate a bad pose into the map, but propagate the
dead-reckoning APART as a SOFT inter-map connection; HARD when there are good
visual matches.* `provider_fusion_node` now records `AnchorEdge`s between submap
anchors and persists them to `<map>/anchor_edges.csv`:
- **CHAIN-ODOM** (type 0) — consecutive submaps, good odometry (tight σ 0.05/0.02).
- **SOFT** (type 1) — consecutive submaps where the segment was visually degraded
  (raw landmark yield < `anchor_soft_lm`=7000, or images missing) → the odom across
  it is less trustworthy → high σ (1.0/0.3): approximate placement only, don't trust
  its geometry.
- **HARD** (type 2) — verified weld (reloc passed consensus), σ = loop sigma.

`scripts/plot_multimap.py` draws the Atlas: islands (colored) + anchors + gray/odom,
orange-dashed/SOFT, green/HARD edges. Validated: CASA1_Suave_blackout → 8 islands,
5 odom + 2 soft (4→5 is the real degraded blackout segment, 6→7 the trailing
partial) + 1 hard (7→0 reconnect). Additive — does not touch the live map→odom path.

**Caveat / next:** the edges are RECORDED + visualized; they are NOT yet fed into an
anchor-graph optimization (corrections still propagate via the existing keyframe
graph). Next (R1.1/R1.2): optimize the submap-anchor graph from these edges
(covariance-weighted → soft barely moves geometry, hard pins it) + reversibility
(drop a bad edge). Soft-edge trigger is a landmark-yield proxy; the principled
signal is the provider covariance/tracking-quality (Marginal/Lost) — wire when the
health probe is plumbed. A true odom-stale branch (cam+IMU both out) hasn't been
exercised yet (the bno_ab blackouts are ~1.15s, OKVIS IMU-bridges them).

## 2026-06-19 — never-lost branch supervisor (loss detection -> seal+branch+soft)

provider_fusion_node now detects tracking loss and branches (R-C, the user's
jump-scenario). In onOdometry: an odom **stale-gap** > `stale_gap_s` (0.5) since the
last accepted sample = a loss -> seal the current submap early (the loss sits at a
branch boundary) -> flag the next chain edge **SOFT** (its placement across the gap
is dead-reckoned). A `force_loss_start/end` test window drops odom to simulate it
(bench: `FORCE_LOSS="30,33"`; launch args forwarded).

**Validated** (CASA1_Suave golden, rate 0.5): fired on REAL OKVIS stale-gaps
(1.22s, 0.53s, 1.29s, 1.20s — the bag has natural dropouts) -> 4 SOFT edges at those
branch points + 5 odom + 3 HARD reconnect welds (`results/r0/multimap_branch.png`).
**Real-time x1**: runs WITHOUT diverging (extent ~10 m, not >>30 m) but drops ~half
the frames (2195 vs 4340 poses) on the 8 GB GPU (OKVIS-CNN + XFeat + VPR contention)
— the 60 fps recording gives enough redundancy to stay coherent; full-rate fidelity
needs more GPU / INT8 / two-pass.

**Caveat / next (task #10):** the soft edge's relative pose is still the
(degraded) OKVIS odom across the gap; a TRUE odom-stale branch needs a separate DR
source (raw IMU / wheel / GPS) — the soft edge would otherwise be identity+huge-cov.
Loss signals now: stale-gap + low-landmark + **OKVIS covariance trace > `cov_soft_thresh`
(0.01)** = Marginal/Lost (the principled probe). [compile+logic validated; empirical
re-run on a degraded bag pending — deferred for context budget.]

## 2026-06-12 — P-A shipped: provider_fusion_node + loose chain over OKVIS2-X

First real content of the composition root (MASTER_PLAN v2 §8 P-A — the
smallest end-to-end loop of the loose-fusion-over-external-odometry pivot).

**What landed**
- `slamko_core/odometry_provider.hpp` — `ProviderChain`: provider pose stream →
  keyframe decimation (0.10 m / 0.10 rad / 1 s) → relative edges, information
  from the provider's reported covariance (diag sum of the two endpoint poses,
  floored at 5 mm/2 mrad; PoseGraph default sigmas when the provider reports
  zero). OKVIS's quality-scaled covariance (10× Marginal / 100× Lost) flows
  straight into edge weight — Hard Rule #3, no `if(sensor_ok)`.
- `nodes/provider_fusion_node.cpp` — subscribes `/okvis/okvis_odometry`
  (param; reliable default, `provider_best_effort` for BE providers), feeds
  `slamko_loop::PoseGraph` (chain-composes in the fused frame so a future
  optimize() propagates), publishes `~/fused_odometry` (map frame, exact
  correction) + slewed `slamko_map→slamko_odom` TF (0.5 m/s, 0.5 rad/s caps) +
  `slamko_odom→slamko_base` raw provider TF (param-gated). TUM dumps for bench.
- `tools/provider_chain_offline.cpp` — the no-ROS reproducible gate: recorded
  provider TUM → same chain+graph path → optimize() → compare vs input.
- `launch/pa_okvis_bag.launch.py` — OKVIS2-X **pure VIO** (config
  rsD455_odom848: `do_loop_closures=false`, the double-LC rule) on a D455 bag +
  the fusion node. `scripts/bench_pa.sh` = pre-flight zombie ABORT (never
  blind-pkill — the okvis process may be the user's), run, teardown only our
  PIDs, gate compare.

**Numbers (gate: fused must track the provider — no global constraints yet)**

| Gate | Input | KFs/samples | Result |
|---|---|---|---|
| offline | okvis_Suave.tum (2475 poses) | 376 KF / 375 edges | max 2.9e-14 m, PASS |
| offline | okvis_Escaleras.tum (5184 poses) | 747 KF / 746 edges | max 4.8e-14 m, PASS |
| live | CASA1_Suave bag (80 s, OKVIS live) | 2315/2315 samples | max 0.000000 m, PASS |
| live | CASA1_Escaleras bag (165 s, multi-floor) | 6079/6079 samples | max 0.000000 m, PASS |

The live comparison is fused-vs-provider **within the same run**, so it is
invariant to OKVIS's own run-to-run nondeterminism — what it proves is the
chain math + plumbing, exactly P-A's scope. (Run 2 of Suave repeated the PASS —
see results/pa/.)

**Where to look when it misbehaves:** `results/pa/<bag>/launch.log` (OKVIS +
node output), `provider.tum` empty → topic/QoS mismatch (is OKVIS publishing?
`ros2 topic hz /okvis/okvis_odometry`), `fused.tum` diverging → edge math or a
global constraint landed when none should.

**Next (P-B):** EigenPlaces reloc as the first global-constraint source into
this same pose-graph; then optimize() moves off the callback thread.

---

## 2026-06-12 — P-B step 2a: KF images + EigenPlaces + sealed VPR submaps in the provider chain

`provider_fusion_node` grew the P-B capture path (param-gated, `image_topic`
empty = pure P-A): infra1 image ring buffer (0.6 s) → nearest image per chain
keyframe (±60 ms) → **EigenPlaces TRT** (wrapper reused from slamko_vio_core —
composition-root privilege; `slamko_vio` now exports the vendored
tensorrtbuffer headers) → per-KF `global_descriptor` → **sealed SMP submaps**
(`kf_per_submap`=50, anchor = first KF of segment, manifest maintained,
trailing partial sealed in the destructor on clean SIGINT).

**Live validation (CASA1_Suave bag, `vpr:=true`):** 5 submaps / 250 KF /
**VPR coverage 100%** (`smap_info` hard gate OK), sane anchors. Caveats found
and handled: (a) first run pays ~30 s building the EigenPlaces TRT engine
(GPU contention delays OKVIS init) — one-time, cache at
`/tmp/slamko_vio_eigenplaces_512.engine`; (b) bench teardown now SIGINTs
before killing so the trailing submap seals (bench_pa.sh).

**The architectural point:** keyframe capture starts seconds after bag start
(OKVIS init), so the 133 s start-room data hole that killed the old
magistrale bridge cannot recur. **Next (P-B step 2b):** XFeat features per KF
→ retrieval top-10 (per the step-1 verdict) + LighterGlue/PnP verify →
reversible gated anchor edge → optimize off-thread.

---

## 2026-06-12 — P-B step 2b: LIVE LOOP CLOSURE in the provider chain — 5.02 m → 0.057 m

The full reloc chain runs live in `provider_fusion_node`: XFeat per KF (left+
right, static-752 pad/crop handling for 640/848-wide sensors) → stereo NN match
+ triangulation (~200 lm/KF) → submaps sealed WITH landmarks+descriptors →
XFeatRelocalizer (EigenPlaces per-KF top-10 retrieval + PnP-RANSAC verify,
BoW off) on every KF against AGED submaps (older than min_loop_gap_s=25) →
**PCM-lite consensus gate** → robust loop edge → synchronous `optimize()` →
map→odom correction.

**Validated on CASA1_Suave with REAL drift** (GPU contention from our own
TRT inference degraded this OKVIS run to 5-10 m closure error — an accidental
but realistic stress test):

| Run | Gate | Result |
|---|---|---|
| loop2 | none | 14 true loops accepted BUT cost stayed ~2e4, fused frame torn (max 1274 m) — earlier in-run edges + no gating |
| loop3/4 | absolute disagree ≤2 m | ALL true return loops REJECTED (the graph is wrong BY the drift the loop corrects — chicken-and-egg), fused = provider exactly |
| **loop5** | **PCM-lite: 3 consecutive same-submap candidates pairwise-consistent under provider relative odometry (tol 0.30 m/0.15 rad) + 2 s cooldown + 30 m teleport bound** | **6 loops accepted, optimize converged: provider closure 5.020 m → fused closure 0.057 m (88×)** |

**Lessons (load-bearing):**
1. **Absolute disagree gates are structurally wrong for loop closure** — they
   reject precisely when correction is needed. Consensus (matches consistent
   with EACH OTHER under odometry) is drift-magnitude-agnostic.
2. **Our TRT inference contends with OKVIS for the GPU** and can degrade the
   provider 4 cm → 10 m on the same bag. System-level budget issue for the
   robot (mitigations: KF-rate throttle, INT8, second GPU). Meanwhile it
   conveniently generates realistic drift for testing the loop layer.
3. provider odometry latency under load starved 235/1300 KFs of their image at
   a 0.6 s ring buffer → 2.5 s default.

**Remaining for the full P-B gate:** cross-session (load a prior map at
startup, reloc against it → anchor into the prior frame) on the casa bags;
optimize() off the callback thread (P-C′ iSAM2).

---

## 2026-06-12 — P-B step 2c: CROSS-SESSION relocalization + LoopConsensusGate extracted & unit-tested

- **`slamko_core/loop_consensus.hpp`** — the PCM-lite gate extracted as a pure,
  unit-tested class (`test_loop_consensus.cpp`, **8/8 PASS**: true-streak,
  drift-agnostic (the 6 m lesson), aliasing-jitter-never-accepts,
  streak-reset, rotation-inconsistency, cooldown-burst, per-target
  independence, stationary-robot). The node is now a shell around it.
- **Cross-session**: `prior_map_dir` loads a prior smap archive at startup,
  registers it into the relocalizer; consensus-accepted matches into PRIOR
  submaps re-anchor the session (`T_global_map` estimated, `slamko_global`→map
  TF) — anchor-don't-weld, session graph untouched.

**Validated (CASA1_Suave bag vs the loop5 prior map):** prior loaded (9
submaps) → **LOCALIZED at kf 2 (~2 s), 200 PnP inliers, T_global_map
translation = 2 mm** (same physical start point → ~identity expected ✓);
continuous re-anchors stable at mm-cm. Cross-DAY/cross-bag (Escaleras vs Suave
prior, casa1↔casa2) = the remaining P-B gate matrix entry.

**Supereight2 study (user question, agent-verified against the OKVIS2-X code):**
OKVIS's dense submap alignment (`SubmapIcpError`: point-to-occupancy-field,
σ-weighted) plays NO role in its loop-closure DECISION (that's DBoW2 + RANSAC +
drift heuristic) — it refines poses AFTER acceptance, and only in the dense
configs (the sparse 3.22 cm baseline has none). For slamko's loose layer the
right analogue is **landmark-cloud overlap verification after an anchor**
(Bosch-style reversible merge check, ~no new deps) — planned for P-C; copying
supereight2 would couple us to the provider's dense backend for marginal gain.

---

## 2026-06-12 — P-C first pass: S1 BLACKOUT gate — auto-recovery works on both protocol bags

The kidnap/blackout protocol bags through the unmodified P-B stack (no
dedicated never-lost code yet — recovery emerges from continuous per-KF reloc +
consensus loops):

| Bag (49.4 s, lens covered mid-run) | Provider closure | Fused closure | Crashes | Loops |
|---|---|---|---|---|
| CASA1_Suave_blackout | 3.12 m (18.9 m/s IMU-only spike at t+34) | **0.16 m** | 0 | 2 |
| CASA1_Suave_blackout4 | 3.69 m | **0.09 m** | 0 | 2 |

S1 gate criteria (PLAN_STRESS_SUITE): zero crashes ✓, clean recovery ✓,
un-aligned divergence bounded ✓ (single-run; reproducibility pass pending).
Remaining P-C: explicit seal-on-loss/branch state machine (today the chain just
keeps consuming OKVIS's quality-inflated covariance — Hard Rule #3 doing the
work), Atlas multi-prior-map + A↔B bridging, landmark-overlap merge check.

Also this block: klt_vo confirmed drop-in second provider (offline contract
gate PASS 5.3e-14 on its MH_01 est.tum; its node already publishes
nav_msgs/Odometry on /klt_vo/odometry) — P-E is a remap away. bench_pa.sh:
PRIOR_MAP env + empty-arg fix.

---

## 2026-06-12 (night) — CROSS-BAG FUSION: Suave localized inside the Escaleras map (LighterGlue)

The épico the user asked for: two different walks sharing a start point, fused.

1. **Escaleras map built**: 17 submaps, 164k landmarks, 10 in-session loops;
   provider drifted 9.24 m (stairs + GPU contention) → fused closure **0.07 m**.
2. **Cross-bag fusion (Suave over the Escaleras prior)**: LOCALIZED at kf 2,
   **53 LighterGlue inliers, T_global_map ≈ 3 cm** (shared start point);
   14 re-anchors through the run; `global.tum` dumps the fused-frame trail.

**Root-cause chain that got here (3 failed runs first):**
- A single best-of-all relocalizer NEVER surfaces a prior map once own submaps
  exist — same-session imagery always out-scores a different walk in inliers.
  → **dual relocalizers** (session / prior), both feeding the same consensus gate.
- Even then zero prior candidates: XFeat NN-brute verify can't cross walks
  (VPR retrieval was FINE — cross-map diag `--map2`: Suave submap 0 → Escaleras
  submap 0 at cos 0.712). → **LighterGlue ON for the prior relocalizer**
  (slamko_loop now built with `-DSLAMKO_LOOP_WITH_TORCH=ON`), prior inlier bar
  15 (consensus is the precision defense).
- `vpr_recall_diag --map2 A --map2-queries B` = the cross-map retrieval diag.

**Build note:** LighterGlue requires `colcon build --packages-select slamko_loop
--cmake-args -DSLAMKO_LOOP_WITH_TORCH=ON` (libtorch at ~/libtorch); without it
the prior verify silently falls back to NN and cross-bag fusion won't fire.

---

## 2026-06-12 (late) — fusion v2: cross-floor false re-anchor caught (RTABmap zgate lesson, 2-vote correction-consensus)

User spotted it in the viz: Suave sat BELOW the floor. Diagnosis (fusion3 log):
re-anchors 1-13 stable at cm; #14 jumped 3.7 m (prior submap 2 = upper floor,
aliased) and #15 +2.6 m in z — and T_global_map was a hard replace. This is
EXACTLY RTABmap's documented Suave+Escaleras failure (MULTISESSION_FUSION.md:
cross-floor false loops are SELF-consistent; zgate at |ΔZ|>1.5 m was their fix;
their clean reference run = casa1_fused_clean_zgate.db).

Fix: **correction-consensus on re-anchor updates** (`reanchor_jump_m`=0.5):
small refinements apply directly; a BIG T_global_map change needs TWO
consecutive accepted re-anchors agreeing on it (real drift repeats; an aliased
floor jumps elsewhere next). fusion4: the 3.62 m jump to submap 2 was HELD (no
2nd vote ever came — confirmed alias), 12 clean re-anchors, global z end
-0.06 m (was +2.65). Suave landmarks now plotted fused into the Escaleras map.

**Known remaining (the P-C′ headline):** mid-run z dip (-2.3 m) = provider
drift leaking BETWEEN re-anchors — the output-transform re-anchor only corrects
where the prior covers. The continuous fix is prior ANCHOR EDGES in the
pose-graph (+ refreshing sealed submap anchors post-optimize), so corrections
distribute through the whole trajectory.

---

## 2026-06-12 (final) — GPU contention CONFIRMED dose-response; rate 0.5 = clean maps

User: "OKVIS verde está perfecta, algo hacemos mal" — correct. Controlled test,
same bag, same code:

| Bag rate | Provider closure | z min | Fused closure |
|---|---|---|---|
| 1.0 (contended) | 8.25 m | -2.23 | 0.01 m |
| **0.5** | **0.75 m** | -0.69 | **0.022 m** |
| (reference, no slamko inference at all) | 0.04 m | ~0 | — |

Our TRT inference (XFeat×2 + EigenPlaces per KF) starves OKVIS's GPU.
**Operational recipe:** bag map-building at rate ≤0.5; online robot needs a GPU
budget (reloc inference throttle `reloc_every_n_kf` — next session, INT8
engines, or a second GPU). Sealed-anchor refresh after optimize() remains the
map-straightening fix for whatever residual drift the provider has.

---

## 2026-06-12 (close) — P-E first light: klt_vo AS PROVIDER, full stack, both routes

`pa_kltvo_bag.launch.py` + `PROVIDER=kltvo` in bench_pa.sh: klt_vo (the user's
own 190 fps XFeat/KLT VIO, ~/ros2_ws workspace) drives the SAME
provider_fusion_node — odom_topic:=/klt_vo/odometry, body_T_cam=IDENTITY
(klt_vo publishes the CAMERA pose, klt_vo_node.cpp:767), bag QoS override for
its RELIABLE IMU sub. Zero fusion-code changes — the loose contract delivered.

| Route (rate 0.5) | klt_vo raw closure | slamko fused | loops | submaps/landmarks |
|---|---|---|---|---|
| CASA1_Suave | 0.489 m | **0.080 m** | 6 | 8 / 79k |
| CASA1_Escaleras | 2.477 m | **0.163 m** | 11 | 16 / 155k |

Note: klt_vo's Escaleras z span (-2.2..3.5) underestimates the climb vs OKVIS
(0..8.2) — the known stairs-bias signature; the loop layer still bounds it.

**Addendum (klt_vo docs check):** klt_vo's CLAUDE.md platform rule says
`feature_detector:=xfeat` is THE robot/D455 config (wins both axes on the real
casa bag, 190 fps) — the d455 launch default (shitomasi) is the blur-bench
default, not the robot one. With xfeat: Escaleras raw 2.477→1.628 m, fused
0.163→**0.070 m** (12 loops). Now the launch default in pa_kltvo_bag. The z
compression on the climb (±3 vs true 8 m) is klt_vo's documented open
stairs-bias — its fix belongs in the klt_vo repo (bias carry-forward + gravity
gate per the slamko stairs memory); the loop layer bounds it meanwhile.

---

## 2026-06-12 (FINAL) — ROOT CAUSE of the inflated maps: WRONG OKVIS CALIB CONFIG (848 on 640 bags)

The user's "no es normal tanto desvío / parece doble más grande" was right, and
it was NOT (only) GPU contention. The chain of elimination:
1. Two-pass run on a verified-clean machine STILL gave provider scale 0.56 →
   contention theory dead for scale.
2. A live klt_vo sprint bench was found sharing the GPU all day (explains the
   run-to-run variance and the 673 m outlier, but not the systematic scale).
3. The launch hardcoded `rsD455_odom848` (848x480, fx=426.15) — but the
   CASA1_*_BNO bags are **640x480 (fx=385.95)**. Wrong calibration -> ~1.7x
   trajectory scale inflation, episodic drift, bent maps. ALL of today's casa
   runs carried it.
4. `rsD455_bno` is for the BNO055 external IMU (diverges with the camera IMU).
   **`rsD455_map_odom` is the correct config**: 640x480, camera IMU, odom-only
   (loops off — RTAB-Map/slamko does LC).

**With rsD455_map_odom + the two-pass recipe (clean GPU):**
provider Sim3 ATE **3.5 cm, scale 1.000**; slamko graph.tum **4.2 cm, scale
1.000**; flat floor; inter-submap walls 42 cm median (residual = normal cm
drift, no longer pathology). The map finally looks like the house.

**RULES learned (hard):** (a) verify `config image_dimension == bag resolution`
BEFORE any OKVIS run — it fails silently and tracks plausibly at the wrong
scale; (b) evaluate on graph.tum with Umeyama SCALE printed — closure and even
SE3-ATE can hide a scale error; (c) check `nvidia-smi`/pgrep for OTHER GPU
tenants before blaming algorithms. Defaults fixed: pa_okvis_bag + map_two_pass
use rsD455_map_odom; d455_setup launch grew a config_dir arg (default
unchanged).

---

## 2026-06-13 — THE CLEAN FUSION: second root cause (2x camera-IMU accel) + image-driven replay

Escaleras kept diverging (z→km, then bad_alloc from the runaway map) even with
the right calib config — **second root cause: the bno_ab bags' /camera/camera/imu
accel is DOUBLED** (unite_imu_method:=2). The PROVEN launch for these bags is
`~/coding/BNO055/ab/okvis_ab_c1_d455imu.launch.py` (imu_relay --accel-scale 0.5
-> /okvis/imu0; OKVIS_CFG env; 80 Hz propagated odometry). Suave had survived
the 2x accel by luck (gentle motion); the stairs' vertical accelerations
couldn't. map_two_pass.sh pass 1 now uses it (OKVIS_CFG=rsD455_map_odom).

Also: pass-2 replay re-synced — two independent bag players skew by the pass-1
pre-roll; `scripts/odom_player.py` now replays the recorded odometry
IMAGE-DRIVEN (publishes each message when the image stream reaches its header
stamp — synced by construction).

**Clean results (two-pass, correct config + accel relay):**
| Run | provider | slamko graph | loops |
|---|---|---|---|
| Suave | 3.5 cm / scale 1.000 | 4.2 cm | 1 |
| Escaleras | 10.1 cm / scale 1.008 | **8.1 cm (improves the provider)** | 9 |
| **Fusion (Suave over Escaleras prior)** | — | LOCALIZED kf 3, 53 inliers, **T_global = 4 cm**, 14 re-anchors, **0 held jumps** | 6 |

With clean data the consensus/jump gates sit idle (0 interventions) — they are
armour, not crutches. Map: 151k landmarks, real floors 0→6.7 m, flat ground.
