# RESEARCH_D455_CLEAN_MAP_01 — how to make a CLEAN map with a RealSense D455

<!-- authored 2026-06-29 · 4-agent internet swarm (RealSense GitHub issues, RTAB-Map/nvblox/voxblox
docs, Reddit/ROS-Discourse practitioner threads, TSDF papers). All cited in the agent transcripts.
Motivation: slamko's D455 TSDF map comes out noisy/thick-walled with stray fragments + deformed at
loop close. The user (correct): "people map houses with D455 weekly, it can't be impossible." -->

**Verdict: it is NOT impossible — it's a known, fixable failure mode.** Every working open-source
D455 mapping system (RTAB-Map, nvblox, voxblox, Open3D, OctoMap) converges on the SAME recipe, and
slamko is missing most of it. Two physical facts cause everything:
1. **Stereo depth noise grows with distance² (to cubic)** → far surfaces smear into thick/double walls.
2. **Flying pixels** at every depth discontinuity (edges) → the map "spray" / stray fragments.

## The universal recipe (all 4 research angles converge — ranked by impact)

1. **HARD depth-range cap ~3–4 m.** THE single biggest win for thin walls. D455 Z-error is <2% only to
   ~4 m, then quadratic. Every system has this knob: RTAB-Map `Grid/RangeMax` (default 0=∞ → set 3.0),
   nvblox `max_integration_distance_m` (default 7 → ~3.5), voxblox `max_ray_length_m`, Open3D `depth_max`.
2. **TSDF / log-odds multi-ray averaging + voxel downsample.** A wall must be confirmed by MANY agreeing
   rays before it's solid → kills single-ray noise / double walls. Inherent to TSDF; tune voxel size ≥
   the depth-noise floor at working range (0.05 m OK to ~3–4 m).
3. **Distance/angle-aware integration WEIGHT** (`w ∝ 1/z²`, the sensor-error model) + **anti-grazing**
   (down-weight oblique/grazing pixels) + truncation = a small multiple of the voxel. Far/oblique depth
   contributes LESS → thin walls. nvblox supports a sensor-error weight (we use CONSTANT).
4. **Outlier removal / free-space CARVING.** Require ≥N observations before a voxel is "occupied"
   (Open3D `weight_threshold` default 3; OctoMap log-odds hit/miss + clamping) AND ray-trace/carve the
   free space (deletes floating fragments — "great at removing outlier structures like floating noise
   pixels and bumps along edges"). RTAB-Map `Grid/NoiseFilteringRadius`=0.1 + `MinNeighbors`=5,
   `Grid/RayTracing`=true.
5. **Depth PRE-FILTERING before integration** (librealsense pipeline, in the DISPARITY domain because
   noise scales with z²): **Decimation(2) → Spatial(α0.6/δ8/mag2) → Temporal(α0.5/δ20) → mild Hole-fill**,
   + explicit flying-pixel/edge removal. GOTCHA: the temporal filter causes "spray" while MOVING — use
   High-Accuracy preset + a statistical-outlier-removal final pass instead, or temporal only when slow.
6. **Clean POSES (the deformation fix — the user's TSDF point).** "A map that deforms at loop closure is
   a POSE problem, not a depth problem — the map jumps because the loop is yanking out accumulated
   drift." A deformable TSDF is only as clean as the pose-graph behind it: keep **submap-local TSDFs**,
   **converge the pose-graph/loop-closures BEFORE the final re-integration**, never integrate across an
   unconverged/fragmented trajectory (Voxgraph, RTAB-Map OnMapChange). Reduce drift → smaller jump →
   cleaner map.
7. **2D occupancy cleanup**: normal-based ground/obstacle segmentation + ray-traced free-space clearing
   + morphological speckle removal (RTAB-Map `OccupancyGrid`). Don't dump the raw cloud into a costmap.

## Sensor-side checklist (recording / driver — for future bags)
- **Range clip 0.4–3.5 m** (Threshold filter). · **Preset = High Accuracy** (confidence threshold drops
  flying pixels). · **848×480 @ 30 fps** (NEVER 720p — it's extrapolated, equal-or-worse depth). ·
  **Emitter ON** for depth (we already alternate on/off for VIO/depth — correct). · **GAIN=16**, tune
  exposure, **disable auto_exposure_priority** (locks FPS + IMU↔cam time-sync), **enable Global Time**,
  `enable_motion_correction:=true`, `unite_imu_method=linear_interpolation`. · **On-Chip Self-Cal + Tare**
  against a known wall (fixes bent/scaled/doubled maps). Post-2022 D455 (BMI085) ships well-calibrated.
- Pair with **infra/stereo odometry, NOT color** (smoother odom); don't point at textureless walls (drift).

## What slamko is MISSING (the gap → the fix)
Our nvblox config (`slamko_tsdf/src/nvblox_backend.cpp`, `VolumetricParams`):
| Lever | Today | Fix |
|---|---|---|
| `max_integration_distance_m` | **8.0** (run didn't even cap → 8 m) | **→ 3.5 m** — the #1 win |
| integration weight | **constant** | **sensor-error `1/z²`** (nvblox supports it) |
| max_weight | 100 (high) | keep (clean static) ✅ |
| depth pre-filtering | **none** (raw HW depth → nvblox) | add Decimation/Spatial/Temporal OR statistical-outlier removal |
| occupancy extraction | ESDF dist < thresh, **no weight gate** | **require observation weight ≥ N** (kills speckle/fragments) |
| free-space carving | TSDF-implicit only | ensure ray-carving; add 2D noise-filter radius |
| poses before re-integration | re-integrate live + final bend | **converge the graph + ONE clean re-integration at export** (Voxgraph pattern) |
| 2D cleanup | raw TSDF slice | ground-seg + ray-trace clear + morphological |

## The plan (ordered by impact, cheapest first)
1. **`max_integration_distance_m` 8 → 3.5 m** + always pass `volumetric_max_range_m:=3.5`. One number,
   biggest win. (Re-run, slice, compare wall thickness.)
2. **Weight-gate the occupancy export** in `exportCostmap`/`queryDistanceField` consumers: require the
   ESDF voxel `weight ≥ N` (nvblox `getVoxels` exposes it) before marking occupied → removes the stray
   fragments + speckle. (We already read `weight` in `queryDistanceField`.)
3. **Sensor-error `1/z²` integration weight** in `nvblox_backend` (nvblox `weighting_function`).
4. **Depth pre-filter** stage (decimation + spatial + temporal, or a statistical-outlier pass) before
   `integrateDepth`. Biggest code change.
5. **2D occupancy cleanup** (noise-filter radius + morphological) on the published costmap.
6. **Export-time clean map**: a final global pose-graph solve + ONE re-integration of the whole TSDF at
   converged poses (the OKVIS-clean equivalent), keeping the live islands for never-lost.

**Bottom line:** slamko already runs nvblox, which has almost every lever — they're just not turned on.
The map isn't deformed because the D455 can't map; it's far-range depth (8 m) integrated unfiltered with
a constant weight, no observation-confidence gate, over a fragmented/drifted trajectory. Fix those and
the D455 produces the same clean map everyone else gets.

## Sources (per-agent transcripts, all URL-cited)
Intel post-processing + presets + calibration; RealSense-ros #2906 (the canonical bad→good recipe),
#3321; rtabmap_ros #357/#1221, rtabmap #614; nvblox Isaac-ROS parameters reference; voxblox IROS-2017;
VDBFusion; Open3D TSDF (`weight_threshold`); OctoMap log-odds; Voxgraph (submap pose-graph);
arXiv 1803.03932 (cubic range-error), 2311.00626 (nvblox), 2410.08084 (flying-pixel correction).
