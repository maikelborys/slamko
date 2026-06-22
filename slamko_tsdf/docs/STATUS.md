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
