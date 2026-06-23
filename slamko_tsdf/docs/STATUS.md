# slamko_tsdf — STATUS

Validated milestones (dated, with numbers). Newest first. See
[`../README.md`](../README.md) + [`../../docs/PLAN_SLAMKO_TSDF_01.md`](../../docs/PLAN_SLAMKO_TSDF_01.md).

## 2026-06-23 — LIVE driver (policy) + slamko_ros node wiring (D455 HW depth) ✅

Increment 2: the live POLICY over the engine + the ROS composition-root wiring, so a
running slamko fuses a live volumetric map from the D455 hardware depth stream.

- **`VolumetricLiveDriver`** (`live_driver.hpp/.cpp`, pure C++, +6 gtests) — the policy
  the node drives: `addKeyframe(frame, world_pose)` forward-fuses + tracks; `applyCorrection(
  world_poses)` DIFFS the latest corrected poses against where each kf was last fused and
  windows ONLY the moved set (decoupled from the optimizer — a loop just shows up as moved
  poses on the next snapshot); `enforceBudget()` seals oldest frames outside `keep_recent`
  once the store passes a byte budget. KEY correctness: the move-diff baseline is the last
  INTEGRATION (not the last snapshot) so a slow sub-threshold creep accumulates and fires
  (test `SubThresholdDriftAccumulatesNotResets`). Suite: **19 tests, 0 failures** CUDA-free.
- **Live depth source = D455 HW depth topic** (`/camera/camera/depth/image_rect_raw`, 16UC1
  mm → float m). NO SGBM/HITNet, NO GPU for depth (on-ASIC). Intrinsics 848×480
  fx=fy=426.1532 cx=423.6672 cy=240.5506, extrinsic from make_depth_skdf_frombag — all params.
- **provider_fusion_node wiring** (gated by `volumetric:=true`): subscribes the depth topic
  (timestamp buffer like img_buf_), `volumetricOnKeyframe` picks the nearest depth → DepthFrame
  → `addKeyframe` at `worldPose(id)=T_global_map ∘ graph.pose(id)`; on a cadence
  (`volumetric_correct_every`, default 10 kf) snapshots `graph_.poses()` → `applyCorrection`
  (the bend) → `enforceBudget` → publishes a Nav2 `OccupancyGrid` on `~/volumetric_costmap`
  (transient_local). Destructor does a final bend at the optimized graph + mesh export.
- **Decoupling preserved:** nvblox stays hidden behind slamko_tsdf's PIMPL — slamko_ros links
  only `libslamko_tsdf.so` (which pulls nvblox transitively when built `-DSLAMKO_WITH_NVBLOX`);
  the node's own compile is CUDA-free. Verified: CUDA-free node build green; **GPU build green +
  linked** (`provider_fusion_node` → `libslamko_tsdf.so` → `libnvblox_lib.so`). With nvblox OFF
  the driver runs a no-op backend (empty costmap) — harmless. Hard-rule #4 intact (OKVIS never
  sees depth; depth feeds only the map layer).

**LIVE GPU RUN — VALIDATED on the casa flashbag** (`scripts/run_slamko_casa_volumetric_live.sh`,
2026-06-23): full 77 s bag @ rate 0.5, `volumetric:=true`, nvblox-GPU backend. **581 keyframes
fused LIVE** (581 re-poseable, only 2 kf no-depth), 12 submaps, **LOOP CLOSED** (kf 581→submap 0,
inliers 77, optimizer converged) → the pose-graph bent → the volumetric layer window-re-integrated
the moved keyframes on the cadence corrections → **1 FUSED component** (coherent house) → final
bend + **25 MB mesh PLY**. nvblox as a 3rd GPU consumer @rate 0.5 did NOT starve OKVIS (engines
pre-built + bag gated on node `reloc ready` — the fixed 20 s warmup raced the TRT build the 1st try,
caught only 35 kf; the readiness gate fixed it). Matches the offline map, but built LIVE + incrementally.
- **The depth-store bound is now MEASURED:** 581 frames @848×480 unbounded = **946 MB** (the 751 MB
  problem, confirmed). `enforceBudget` is the lever; the bound A/B (budget on/off vs map coherence)
  is the next gate.

**BOUND-THE-STORE A/B — VALIDATED** (2026-06-23, same casa flashbag): `volumetric_store_budget_mb:=200`,
`keep_recent:=60` vs the unbounded baseline.

| | unbounded | bounded 200 MB |
|---|---|---|
| store RAM at end | **946 MB** | **213 MB** (sealed 448 frames, kept 131) |
| keyframes / loop closed / fused comp | 581 / ✅ / 1 | 579 / ✅ / 1 |
| mesh verts | 284 892 | **124 216 (−56%)** |

**Verdict:** the bound WORKS — RAM 946→213 MB (4.4×), the map stays topologically COHERENT (loop still
closed, 1 fused component), and at voxel/costmap resolution (0.05–0.125 m) the house structure is intact
(navigable map preserved). The COST is −56% fine-mesh density: a late loop closure moves the OLD frames
(submap 0), but those are SEALED → can't re-pose, and a neighbouring unsealed frame's `clearRegion` can
wipe baked voxels that only a sealed frame covered → thinned reconstruction in early-explored regions.
**The seal-vs-late-correction tension is the real lifelong tradeoff, now measured.** Refinements (next):
(a) larger budget / keep_recent → less density loss; (b) smarter seal — don't seal frames in submaps that
are still plausible loop targets (seal only settled, already-loop-closed regions); (c) guard `clearRegion`
to NOT clear blocks only sealed frames cover (leave stale-but-present geometry vs holing it).

**Next:** the seal-policy refinement (b/c above); the depth source is now the splitter's emitter-ON
`/nvblox/depth` (94% coverage, decoupled — agent-verified safe for OKVIS/XFeat); the geometric loop
channel; live RViz/Rerun costmap viz. **Reliability reframe (2-agent architecture refresh):** the
volumetric map's fidelity is bounded by POSE quality (OKVIS odom + XFeat reloc), not depth quality — a
0.1 m pose error dominates the ±2% depth error at indoor range. Invest reliability in odometry/reloc first.

## 2026-06-23 — LIVE path foundation: incremental fuse + touched-window bend + store bound ✅

The engine the slamko_ros volumetric node will drive (increment 1: pure C++, CUDA-free
gtested + nvblox GPU path compile-verified). The offline `reintegrate()` (reset + re-fuse
ALL, ~2.6 s/483 frames) was the only bend; LIVE needs forward fusion + a cheap loop bend +
a bounded store. Added to `VolumetricMapper`:

- **`integrateLive(frame, pose)`** — forward incremental fusion as keyframes arrive (no
  reset, the map grows); the frame is KEPT re-poseable with its world pose + footprint.
- **`reintegrateWindow(moved_kfs, poses)`** — the **touched-window bend**. A loop moves a few
  keyframes → clear ONLY the region they touch (old ∪ new footprint) + re-fuse the stored,
  unsealed frames overlapping it → **O(frames touching the window), not O(whole map)**. Falls
  back to a full `reintegrate()` past `kWindowFullFraction`=0.6 moved (clearing a near-global
  region is wasteful). The frame-selection correctness (only overlap re-fused, non-overlap
  untouched, moved pose updated) is the load-bearing logic — fully unit-tested.
- **`clearRegion(Aabb)`** added to the `VolumetricBackend` contract (default no-op so the
  fake/stub + CUDA-free build are unaffected). nvblox impl =
  `getBlockIndicesTouchedByBoundingBox(block_size, aabb)` → `TsdfLayer::clearBlocks(idx)`
  (ESDF/mesh are derived → recomputed at export). **Compile-verified + linked** against the
  installed nvblox (`~/ros2_ws/install/nvblox_ros`, CUDA 12.6): `libslamko_tsdf.so` carries
  both `NvbloxBackend::clearRegion` and the base virtual, links `libnvblox_lib.so`+`libcudart`.
- **Depth-store bounding (the 751 MB lever):** `sealFrame(kf)` drops a stable frame's depth
  payload — geometry stays baked, frame no longer re-poseable (window bends skip it).
  `storeBytes()` / `numLiveFrames()` accessors. The MECHANISM lives here; the seal POLICY
  (which frames are outside the active/loop window) will live in the ROS node.
- **Footprint** = world AABB of the depth frustum (camera origin + 4 image-corner rays cast to
  `max_integration_distance_m`) — the unit of "the touched window" and the overlap test.
- **Store model refactor:** `frames_` is now `StoredFrame{frame, world_pose, footprint, sealed,
  fused}`; offline `reintegrate()` now also records each frame's integrated pose+footprint, so
  the LIVE window path can continue seamlessly from a full build.
- **GATE — +6 gtests** (live fuses-without-reset + skips invalid; window clears-and-refuses-only-
  overlap; window full-fallback past the fraction; seal drops-payload-and-is-skipped; window
  noop on empty/unknown-moved). Suite: **13 tests, 0 failures** CUDA-free.

**Next (increment 2):** the ROS node in slamko_ros (composition root) — subscribe the live
keyframe/depth/pose-graph stream, drive integrateLive on new kf + reintegrateWindow on a
correction (moved-kf set from the graph delta), seal frames leaving the active window, publish
the costmap. Needs a live depth source (the splitter/HITNet path, today offline → .skdf).

## 2026-06-22 — package spine: contract + bend policy + nvblox backend (gated) ✅

New opt-in package. The slamko-native volumetric layer scaffolded and the **bend
policy unit-tested CUDA-free**.

- **Contract** (`slamko_core/volumetric_map.hpp`): `DepthFrame` (per-kf re-poseable
  depth SOURCE — pose deliberately not stored), `CostmapSlice` (2D ESDF/occupancy in
  map frame), `VolumetricBackend` (integrate-at-pose / reset / exportCostmap / mesh),
  `VolumetricParams` + `CostmapParams`. Honest-unknowns + decay-OFF encoded as invariants.
- **`VolumetricMapper`** — owns the per-kf depth store + a backend. `reintegrate(
  kf_world_pose)` IS the bend: reset → fuse every VALID frame at its keyframe's
  CORRECTED world pose; skip frames whose kf is unanchored (dangling island). This is
  the PLVS/RTAB on-demand re-integration, slamko-native (NOT nvblox_submap's rigid
  per-submap anchor).
- **`NvbloxBackend`** (PIMPL, gated `-DSLAMKO_WITH_NVBLOX`): real GPU path written
  against the nvblox `Mapper`/`EsdfSlicer` API (decay never called, max_weight 100,
  max_dist 8 m = clean static map); no-op stub (`available()=false`) when off so the
  workspace builds CUDA-free.
- **GATE — 4 gtests** with a fake backend (reset-once-then-fuse, corrected poses passed
  through, dangling-kf skipped, invalid-frame skipped, available() accessor). Suite
  green: **5 tests, 0 failures**. Build: slamko_core + slamko_tsdf finished, no-op path.
- **nvblox GPU path COMPILE-VERIFIED** ✅ — `-DSLAMKO_WITH_NVBLOX=ON` builds clean
  against the installed nvblox (`~/ros2_ws/install/nvblox_ros`, CUDA 12.6): the whole
  `nvblox_backend.cpp` (Mapper/MapperParams/DepthImage/Camera/integrateDepth/EsdfSlicer)
  matches the real API, zero drift. `libslamko_tsdf.so` links `libnvblox_lib.so` +
  `libcudart`. CMake needs `find_package(glog/gflags)` BEFORE `find_package(nvblox)` —
  nvbloxConfig doesn't pull its own imported targets (fixed).

## 2026-06-22 — FIRST REAL MAP: casa40 nvblox TSDF, corrected poses ✅🎉

End-to-end on the user's house. slamko (VPR on, rsD455_map848, rate 0.5) → 11 submaps,
483 keyframes. `scripts/make_depth_skdf.py` (SGBM + **WLS edge-aware filter** +
confidence mask, baseline 0.0950564 m, T_SC0 extrinsic) → 483 `.skdf` depth files.
`slamko_tsdf_export -DSLAMKO_WITH_NVBLOX=ON` → **483/483 frames integrated at the
CORRECTED poses** in ~2.6 s → mesh 291k verts (post-WLS; was 992k raw SGBM = noise) +
Nav2 costmap 280×408 @ 5 cm. `scripts/viz_tsdf.py`: the **mid-height horizontal cut shows
the house floorplan** (walls as outline) with the trajectory through it; floor→ceiling
slices; 3D. z robust range [-1.57, 2.38] m.

- **Bugfix (segfault):** `EsdfSlicer::sliceLayerToDistanceImage` fills the output Image in
  **device** memory regardless of the requested type → a host `memcpy` from
  `dataConstPtr()` crashed. Fixed: allocate the slice `kDevice` and `Image::copyTo(host)`.
- **Depth quality:** raw SGBM floods the TSDF with speckle (992k noisy verts); WLS
  (lambda 8000, sigma 1.5) + confidence ≥110 + max-depth 5 m → 291k clean verts, crisp
  walls. SGBM is the v0 source; HITNet/ESS is the quality upgrade behind the same .skdf.

## 2026-06-22 — HITNet (GPU TRT) depth vs SGBM ✅

`make_depth_skdf.py --engine hitnet` runs the **proven d455_hitnet_adapter path** on GPU:
the eth3d 480×640 fp16 TRT engine via `SingleEngineTrtRunner` (FFS venv torch+TRT),
resize src→model, disp@model → depth (`fx_model = fx·640/848`), resize depth back to src
→ FULL src intrinsics (same as SGBM). **483 frames in ~10 s on GPU** (vs CPU onnxruntime
~6 min — system ort is CPU-only; the FFS venv has the GPU TRT path). Run with
`~/coding/FFS/.venv/bin/python` (has torch+TRT+rosbag2). casa40: 333k mesh verts.
- **vs SGBM+WLS (291k, honest holes):** HITNet **fills the textureless walls** SGBM left
  as gaps → continuous walls / more complete floorplan (denser, can hallucinate). Matches
  the RTAB-Map insight (`RTABmap/STEREO_DISPARITY_RTABMAP.md`): the clean-sparse look =
  conservative filtering (holes not guesses); HITNet trades that honesty for density.
- GOTCHA: the FFS venv's cv2 lacks `ximgproc.createRightMatcher` → the SGBM/WLS setup is
  now guarded behind `--engine sgbm`. Only one engine on disk (480×640); the adapter
  rect_848.npz supplies fx/baseline (P0[0,0]=426.15, baseline=0.095056), HW-rectified so
  no remap needed.

## 2026-06-22 — CROSS-SESSION combine + REVISIT voxel behaviour (HONEST) ⚠️

Ran casa100 with casa40 as prior (88 reloc/anchor lines). **CORRECTION (earlier over-claim
fixed):** measured casa100→casa40 NN distance is **~0.38 m median for BOTH corrected and
raw** (centroid offset 2.01 vs 2.11 m) — i.e. the cross-session re-anchoring **barely moved
casa100**. Two reasons: (1) the bno_ab bags share a START (salon) → the two OKVIS frames are
already ~aligned (both at origin), so raw casa100 is already roughly on casa40; (2) slamko
applies cross-session as a **soft PRIOR factor**, not a rigid re-base → gentle, leaving the
~0.38 m residual. So blue (casa40) and green (casa100) do NOT cleanly overlap — the user
caught this. The within-session bend stays cm-scale (OKVIS accurate).

**REVISIT / voxel question (the real finding).** `slamko_tsdf_export --map2/--depth2/
--raw-tum2` fuses a 2nd session INTO ONE TSDF (kf_ids offset by 1e9 → no collision; 957
frames). A revisited voxel gets the **weighted average** of both sessions' depth — so the
outcome depends on reloc residual vs nvblox truncation (~0.2 m at 5 cm voxels):
- residual < truncation → surfaces fuse → revisit **REFINES** (one wall, more weight).
- residual > truncation (our 0.38 m) → surfaces are too far → **DOUBLED walls** (two parallel
  surfaces ~0.4 m apart), NOT fused. `revisit_zoom.png` shows exactly this.
**Implication:** clean lifelong fusion needs reloc residual < voxel truncation, OR the
persistent-MapPoint cross-session dedup (separate mechanism), OR a stiffer cross-session
correction. The volumetric layer alone fuses-or-doubles by that threshold — a load-bearing
design fact for the lifelong map.

**CORRECTION 2 (the NN metric fooled me; user pushed back twice — right both times).**
Per-keyframe corrected(anchor∘T_WB)-vs-raw(provider): translation diff **median 0.34 m,
max 0.81 m**, yaw median 1.4°. So the cross-session DID move casa100 ~0.34 m — corrected ≠
raw (my "barely moved" was wrong; NN-to-casa40 is a weak overlap metric, not alignment).
But the two sessions STILL don't coincide: best rigid ICP of casa100→casa40 only reaches
**0.18 m median** (from 0.38 m) — a ~0.18 m floor that is NOT a rigid offset:
reconstruction-level difference (40 cm vs 100 cm camera heights → different views/HITNet
depth + per-session non-rigid drift). **Does the voxel count grow per revisit?** Not
inherently — nvblox is a fixed spatial grid, an aligned revisit REUSES voxels (bounded by
AREA). With misalignment it grows by the doubled fraction: combined 500 k verts vs 333 k
(full-fuse) vs 601 k (full-double) ⇒ ~60% reused, ~40% extra. So **clean lifelong needs
alignment < truncation (~0.2 m); slamko's soft cross-session leaves 0.38 m → doubles →
grows.** Tighter correction reaches the 0.18 m floor (borderline fuse); below that needs
non-rigid per-submap deformation or the MapPoint dedup.

**WHY they're not aligned (diagnosed, not hand-waved).** Decomposed the residual:
scale = 1.0004 (Sim3 vs rigid same → NOT scale); residual-vs-distance-from-start
correlation = **+0.56**, residual heatmap dark at the salon/start and bright (~0.5 m)
at the extremities ⇒ **accumulated per-session VIO drift (NON-RIGID)**, not a fixable
rigid offset or the camera heights. Each session drifts independently as it leaves the
start; near the salon they coincide, far away they diverge — so a single rigid transform
can't fit (0.18 m floor, growing to 0.5 m at the edges). slamko's cross-session is a SOFT
GLOBAL prior → pins them roughly but doesn't co-optimize the non-rigid warp. **Fix: a
per-submap cross-session correction** (re-anchor each session-2 submap onto its session-1
match, distributing the correction along the trajectory) → pulls the bright edges to the
reconstruction floor (~5–10 cm) → below truncation → clean fusion + bounded voxels.

## 2026-06-22 — total volumetric: Suave + Escaleras (multi-level, fix on 640) ✅

The cross-session rotation fix validated on 640 + MULTI-LEVEL. Ran Escaleras (640) with
Suave (640) as prior → 39 between-edges over 26 distinct kfs; the stairs climb z = -0.07 → 6.61 m
(6.7 m vertical, Z NOT collapsed). Combined Suave+Escaleras into ONE TSDF (1130 frames,
679k verts): the house (Suave, ground) + the staircase zig-zag climbing 6.7 m. Ground-floor
salon overlap fuses: median NN 0.05 m, 62% within nvblox truncation (0.2 m) → the shared
salon walls coincide; the stairs are NEW structure extending the map vertically.
`scripts/viz_tsdf.py` (Suave standalone) + an inline 3D + elevation render
(total_volumetric.png/.html). Confirms the fix is not casa-848-specific.

## 2026-06-22 — the BEND A/B: corrected vs raw poses ✅

`slamko_tsdf_export --raw-tum=<provider.tum>` integrates the SAME depth at the provider's
UNCORRECTED odometry poses (nearest TUM sample to each kf timestamp, 50 ms window) instead
of `anchor∘T_WB` — what a raw-odometry nvblox (rtabmap-style) would build. CASA1_Suave
(640, HITNet, 369 kf): two TSDFs exported, overlaid (raw=red, corrected=blue). The walls
separate by the loop-correction magnitude where odometry drifted; slamko pulls them to the
loop-consistent position. **HONEST magnitude: ~0.12–0.22 m here** — OKVIS is *accurate* on
these bags at rate 0.5–1.0 (a good sign), so the within-session bend is cm-scale, not the
metre-scale doubling of a high-drift/contention run. The mechanism is proven; the dramatic
doubling needs genuine large drift or a CROSS-SESSION combined map (two sessions in
different frames → house appears twice raw, once corrected) — the next, deterministic demo.
GOTCHA: Suave is 640×480 (fx=385.95, cx=319.70), NOT 848 — pass the 640 intrinsics; the
HITNet `fx_model` now uses the actual src width (was hardcoded /848).

## 2026-06-22 — A→B routing over the floor-anchored costmap ✅🎯

`scripts/route_costmap.py`: floor-anchored 2D nav costmap from the HITNet TSDF mesh
(obstacles = vertices in the robot height band [floor+0.1, floor+1.6] flattened + inflated
by robot radius; free = explored trajectory tube ∪ near-obstacle, minus inflation) + **A\*
A→B** (8-connected, goal = farthest free cell in A's connected component → guarantees a
route). casa40: grid 292×179, 3098 free cells, **route FOUND (145 steps)** — the planned
path is straighter/more direct than the recorded wander. The navigation goal ("ruta de A
a B") works end-to-end over the slamko-built map. GOTCHA: goal must be in the SAME
connected component as the start (an isolated free speck is unreachable → NONE).

**Next:** (1) the bend A/B (corrected vs raw provider poses → doubling) — needs a
bigger-drift / cross-session bag (casa40's loop correction is only ~7 cm); (2) wire the
PGM/YAML costmap into Nav2 proper; (3) semantic layer (phase 2).

## 2026-06-22 — offline driver + depth-IO + submap→pose glue ✅

The v0 offline pipeline is wired end-to-end (everything but the GPU fusion + the
HITNet producer).

- **`submap_poses.hpp`** — `keyframeWorldPoses(submaps)` = the bend's input: each kf's
  corrected world pose = `submap.anchor ∘ kf.T_WB` (later submap wins on a dup id).
- **`depth_io.hpp`** — `SKDF` binary per-kf DepthFrame format (kf_id, w/h, intrinsics,
  T_body_cam, float32 depth). The handoff from the depth producer (HITNet, Python) to
  the C++ driver. Pose never stored (comes from the corrected archive → the bend).
- **`slamko_tsdf_export`** — driver: `loadSubMaps` → `keyframeWorldPoses` → load `*.skdf`
  → `reintegrate(poses)` → Nav2 costmap (PGM + YAML, rows bottom-up, origin bottom-left)
  + mesh PLY. No-op CUDA-free (empty costmap) → real with `-DSLAMKO_WITH_NVBLOX`.
- **GATE — +4 gtests** (depth-IO round-trip incl. extrinsic binary-exact + bad-magic
  reject; pose glue composes anchor∘local + later-submap-wins). Suite: **10 tests, 0
  failures**. **Smoke:** the tool on a 3-submap archive → "loaded 3 submaps, 4 keyframe
  poses … exit 0" (load + glue + reintegrate path verified, no GPU).

**Next (needs a GPU run):** (1) HITNet/ESS depth producer (Python) — run on the casa-bag
keyframe stereo, write `*.skdf`; needs slamko to dump kf stereo + ids alongside the
submap archive. (2) Run `slamko_tsdf_export` with `-DSLAMKO_WITH_NVBLOX=ON` on
casa40/100/Suave → first real costmap. (3) Validate: a global planner routes A→B + the
revisit bend is correct (no doubling) vs. integrating at raw provider poses (the A/B that
proves slamko's value over rtabmap-nvblox).
