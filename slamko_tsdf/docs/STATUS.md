# slamko_tsdf — STATUS

Validated milestones (dated, with numbers). Newest first. See
[`../README.md`](../README.md) + [`../../docs/PLAN_SLAMKO_TSDF_01.md`](../../docs/PLAN_SLAMKO_TSDF_01.md).

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
