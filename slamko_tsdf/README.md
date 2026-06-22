# slamko_tsdf — the volumetric layer that bends with the graph

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/PLAN_SLAMKO_TSDF_01.md`](../docs/PLAN_SLAMKO_TSDF_01.md) first.

**Role:** turn slamko's corrected pose-graph into a **dense map that bends with the
graph** + export a **2D costmap/ESDF for global A→B planning**. The local controller is
already fine — this feeds the **global** plan only. Opt-in (`-DSLAMKO_WITH_NVBLOX`).

**The load-bearing idea (why it bends):** a fused TSDF is *fuse-and-forget* — a single
volume does NOT bend on loop closure. So the bend is an **architecture property, not a
backend feature**: keep the **per-keyframe depth SOURCE** (re-poseable); the TSDF is a
**derived** product re-integrated from the keyframe's **corrected** pose. That is PLVS
`OnMapChange` / RTAB "ensamblado on-demand". Offline-first runs it **once** after the
graph is final → perfect smooth bend by construction. This is slamko-native and
deliberately **NOT** `nvblox_submap`'s rigid per-submap anchor (piecewise, no
re-integration) — we re-integrate per keyframe → smooth.

**Pieces:**
- `slamko_core/volumetric_map.hpp` — the contract (`DepthFrame`, `CostmapSlice`,
  `VolumetricBackend`). The only coupling (Hard Rule #2).
- `VolumetricMapper` (`volumetric_mapper.hpp`) — owns the per-kf depth store + a
  backend; `reintegrate(kf_world_pose)` **is the bend** (reset → fuse each frame at its
  corrected pose). Pure C++/Eigen, unit-tested with a fake backend (no CUDA).
- `NvbloxBackend` (`nvblox_backend.hpp`) — GPU TSDF→ESDF, PIMPL. No-op stub unless
  `-DSLAMKO_WITH_NVBLOX` (workspace builds CUDA-free; `available()=false`).

**Build:**
```bash
# CUDA-free (default): contract + bend policy + tests
colcon build --packages-select slamko_tsdf
colcon test --packages-select slamko_tsdf      # 4 gtests, fake backend

# with the GPU backend (needs nvblox install on CMAKE_PREFIX_PATH + CUDA)
colcon build --packages-select slamko_tsdf --cmake-args -DSLAMKO_WITH_NVBLOX=ON
```

**Status / numbers:** [`docs/STATUS.md`](docs/STATUS.md). **Plan:**
[`../docs/PLAN_SLAMKO_TSDF_01.md`](../docs/PLAN_SLAMKO_TSDF_01.md).

**Depth source (v0):** HITNet/ESS stereo-net → per-kf depth (bags are stereo-IR, no
depth topic). Backend + depth source both behind contracts → swappable (libSGM, the
PLVS choice, is a lighter alternative).
