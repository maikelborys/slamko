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
