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

**Next:** (1) compile-verify the nvblox backend with `-DSLAMKO_WITH_NVBLOX=ON` against
the installed nvblox (catch API drift); (2) depth producer — HITNet/ESS on casa-bag kf
stereo → `DepthFrame`; (3) slamko-side glue — build `kf_world_pose` from sealed submaps
(`anchor * kf.T_WB`); (4) offline driver → export costmap on casa40/100/Suave → confirm
a global planner routes A→B + the revisit bend is correct (no doubling).
