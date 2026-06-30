# PLAN_NAV2_01 — Nav2 over slamko's own costmaps (self-contained, no external nvblox)

<!-- validated: PENDING · plan authored 2026-06-29 · status: IN PROGRESS (step 1 = local timer) -->

**Goal (the "make it drive" phase):** drive Nav2 goals over slamko's two costmaps —
a deformable GLOBAL planning map + a reactive LOCAL control map — with **everything
implemented inside the slamko monorepo**. No dependency on the external `nvblox_ros`
node nor the `nvblox_nav2` Nav2 plugin (those live in `~/coding/nvblox_ros2/` and serve
the RTABmap stack; left untouched). The never-jump gate (`gate_live_pose`, default ON)
is the prerequisite and already landed — a planner on a jumping TF would corrupt.

## The decision: why self-contained, not the upstream nvblox path

The upstream nvblox path exists and works: the `nvblox_ros` node natively publishes an
ESDF `nvblox_msgs/DistanceMapSlice` (`~/static_map_slice`, frame `odom`), and the
official `nvblox_carter_navigation.launch.py` + `carter_nav2.yaml` wire it into Nav2 via
the `NvbloxCostmapLayer` plugin. We deliberately do **not** use it because:

1. **slamko already runs its own nvblox** inside `slamko_tsdf` (PIMPL'd, `slamko_ros`
   stays CUDA-free). That instance already produces the same ESDF slice
   (`NvbloxBackend::exportCostmap` → `updateEsdf` → `sliceLayerToDistanceImage`,
   `slamko_tsdf/src/nvblox_backend.cpp:139-178`). The costmap nvblox "generates of the
   box" is **already running here** — `occupancy=false` returns the raw distance ESDF,
   `occupancy=true` returns the thresholded OccupancyGrid.
2. **slamko's instance DEFORMS with the pose-graph** (re-integrates moved keyframes on
   loop closure via `VolumetricMapper::applyCorrection`); the upstream `nvblox_ros` is
   **rigid in `odom`** and never re-integrates on loop closure (confirmed: zero
   `deform/reintegrate/pose_graph` hooks in `nvblox_core`). For a lifelong consistent
   GLOBAL map, the deformation is the whole point.
3. Monorepo / contract discipline (CLAUDE.md Hard Rule #2): a self-contained costmap
   path depends only on `slamko_core` contracts, swappable, no external Nav2 plugin.

## Architecture — two costmaps, one nvblox instance

```
slamko_tsdf nvblox (in-loop, DEFORMS) ── ESDF slice (whole map AABB)
        │
        ├─► GLOBAL  ~/volumetric_costmap  (OccupancyGrid, frame slamko_map, LATCHED)
        │       → Nav2 global_costmap StaticLayer  → global planner
        │
        └─► LOCAL   ~/local_costmap       (OccupancyGrid, rolling window, fixed-rate)
                → Nav2 local_costmap        → local controller (DWB/MPPI)

slamko provides slamko_map → slamko_odom → slamko_base (map→odom slewed + gated; never-jump)
    → remap to map / odom / base_link for Nav2
```

**Key insight (the local doesn't need deformation):** the GLOBAL is the deformable
lifelong map (slamko's value-add). The LOCAL is recent, around-the-robot, reactive — it
does NOT need to bend on loop closure. So slamko's single in-loop instance feeds both:
the global is the latched whole slice; the local is a rolling window of the **same
slice**, re-cropped around the live (gated) robot pose at a fixed rate.

## The 4 gaps in today's LOCAL crop, and the fixes (in order)

Today (`provider_fusion_node.cpp:1357-1408`) the local is a crop of the global slice,
published only on the keyframe-correction cadence (`volumetric_correct_every`, default
10 kf) in the `map` frame, occupancy-binary, no decay. The gaps:

| # | Gap | Fix | Status |
|---|---|---|---|
| 1 | Cadence = per-keyframe → **freezes when stationary**; not Nav2-grade rate | **Fixed-rate wall timer** re-crops a cached slice around the live pose (~10 Hz) | **STEP 1 (this change)** |
| 2 | Local in `map` frame, not `odom` | Window follows live `slamko_map`-frame pose at high rate; with the gate ON `map` is smooth, so `map`-frame local is acceptable. Revisit `odom`-frame if a controller needs it. | step 1 (accepted) |
| 3 | No decay → dynamic obstacle never clears | The in-loop map is deliberately non-decaying (lifelong honest-unknowns). Dynamic clearing needs a decaying live layer (nvblox `decayTsdf`/`decayOccupancy` exist in the wrapped core). | step 3 / v2 |
| 4 | Occupancy-binary, no ESDF distance / inflation | Expose the local in ESDF-distance mode (`CostmapParams.occupancy=false`); Nav2 inflation layer or a slamko cost-conversion | step 2 |

## Steps

1. **LOCAL fixed-rate timer (this change).** Add `local_costmap_rate_hz` param
   (default 10). Cache the last full global slice/grid; a wall timer re-crops the window
   around `T_map_odom_pub_ * live_TOB_` and republishes `~/local_costmap` at the fixed
   rate, decoupled from the keyframe cadence. Single-threaded executor → no locking
   (the node already shares subscription+timer on one thread). The GLOBAL stays on the
   keyframe cadence + shutdown flush (it's the latched static map; no need for high
   rate). **Validate: `capture_costmaps.py` shows the local window tracking the robot
   smoothly, including while stationary; rate ≈ 10 Hz on `ros2 topic hz`.**
2. **ESDF/distance mode for the local.** Add the local in distance mode so Nav2 gets a
   gradient (reactive) rather than binary occupancy. Either publish a distance grid +
   inflation layer, or a small in-repo cost conversion.
3. **Decay for dynamic obstacles (v2).** A decaying live layer so a moved-away obstacle
   clears — without touching the non-decaying lifelong global.
4. **`nav2_params.yaml` + bringup in `slamko_ros`.** global_costmap StaticLayer ←
   `~/volumetric_costmap` (frame `map`); local_costmap ← `~/local_costmap`. TF remap
   `slamko_map/odom/base` → `map/odom/base_link`. Planner + controller + bt_navigator +
   lifecycle_manager.
5. **Lifecycle gate on `localized`** (the `okvis_nav2_bridge` pattern): don't ACTIVATE
   Nav2 / accept goals until slamko reports `localized=true` (or a `/initialpose`
   cold-start override).
6. **Validate bag → sim → real.** Send `/goal_pose` in bag-replay; confirm the global
   planner uses slamko's map and the controller respects the local obstacles; spawn a
   dynamic obstacle in sim (`cerebro_robot_sim`) → confirm reroute (needs step 3).
   Never real-robot first.

## Open questions (resolve while doing it)
- Local frame `map` vs `odom`: start `map` (gate makes it smooth); revisit if a
  controller misbehaves on a map correction.
- Local-window resolution/range: reuse the global voxel (0.05 m) + a 4 m window first;
  tune for controller lookahead.
- Does the global need to be re-exposed as `/map` (true OccupancyGrid map) for the
  StaticLayer, or point the StaticLayer topic straight at `~/volumetric_costmap`? Latter
  first.

## Validation gates (don't advance until the prior is reproducible)
- G1 (step 1): local window tracks the robot at fixed rate, including stationary.
- G2 (step 4): Nav2 reaches ACTIVE, global plan drawn over slamko's map in bag-replay.
- G3 (step 6): a `/goal_pose` is followed; in sim a dynamic obstacle reroutes.
