<!-- validated: research 2026-06-23, backend decision for v2 -->
# Live + lifelong volumetric backend — decision (v2)

> Question (user): everything live? realtime costmap + future long-range/lifelong? how much
> memory does nvblox TSDF use? is it adequate for lifelong? something like Bonxai?

## Decision: TWO backends (local GPU + global sparse out-of-core). Not one.

The mature pattern, converged on by every system: **bounded local volumetric (GPU, disposable,
follows odom) for the realtime costmap** + **per-submap sparse tiles pinned to the pose-graph,
re-posed (not re-integrated) on loop closure, paged by region** for the lifelong global map.

## Why nvblox can't be the global store (measured)

nvblox = dense **8³ voxel BLOCKS** × a LayerCake (TSDF 8 B + ESDF ~20 B + Color ~8 B + Mesh).
~**100 KB/m²** of surface at 5 cm (our casa40 ≈ 28 MB; independent study: **2.55 GB @ 20 mm,
11.87 GB @ 10 mm** for a workspace). **Grows ~linearly with explored AREA → a km neighborhood
= GB-to-tens-of-GB in GPU RAM.** Hard limits: **no out-of-core / no disk paging** (whole map
GPU-resident), **whole-map blocking save** (`.nvblx`), **load OVERWRITES** (no multi-session
merge), ceiling = GPU RAM. Bounded only by `map_clearing_radius_m` (default 5 m rolling) +
decay+dealloc. → **RIGHT for the local rolling Nav2 costmap; WRONG for the global lifelong map.**

## The backends

| | nvblox | Bonxai | OpenVDB / **VDBFusion** |
|---|---|---|---|
| structure | dense 8³ blocks, GPU | sparse hashed VDB-like, CPU | VDB tree (tiles collapse), CPU |
| provides | TSDF+ESDF+Mesh, fast | **occupancy** only (+serialize) | **TSDF** (VDBFusion), .vdb out-of-core |
| lifelong km | ❌ GPU ceiling, no paging | ✅ sparse, Rolling-Bonxai chunks | ✅ **KITTI km in 847 MB vs 30.6 GB dense (~36×)** |
| SDF for planning | ✅ ESDF | ❌ (occupancy, no distance) | ✅ TSDF→SDF |
| use | **local live costmap** | global occupancy (no SDF) | **global lifelong TSDF tiles** |

VDBFusion: 19.57 fps CPU single-core, 2–3 cm accuracy, **no env-size assumption**. Caveat: it's
a fusion utility (no loop closure) → needs an external pose-graph driving **per-submap** tiles —
which **slamko already is**.

## Named precedents (read these)
- **supereight2 + OKVIS2** ([arxiv 2403.09596](https://arxiv.org/html/2403.09596v2)) — octree TSDF
  submaps **rigidly anchored to OKVIS2 keyframes, re-posed on loop closure, frozen at seal**.
  **slamko's EXACT odometry provider — the canonical precedent.**
- **voxgraph** ([2004.13154](https://arxiv.org/abs/2004.13154)) — TSDF submaps pinned to pose-graph
  nodes; the published GLOBAL half of the **nvblox(local) + voxgraph(global)** pair (same ETH/ASL
  lineage). c-blox / voxblox = the submap-TSDF + incremental-ESDF ancestry.
- **nvblox + Isaac** — nvblox IS the official Nav2 local costmap provider: 2D ESDF slice
  (`~/map_slice`, `nvblox_msgs/DistanceMapSlice`) → `nav2_costmap_2d::Layer` plugin; LiDAR-free
  **stereo→ESS→nvblox→Nav2** documented (HITNet drops in where ESS sits).
- Hydra / Khronos (deformation graph, spatio-temporal), maplab (multi-session).

## The insight that makes v2-live feasible
The **local costmap does NOT need HITNet** — the D455 has **hardware depth** (on-ASIC, zero GPU).
Live local costmap = **D455 HW-depth → nvblox (5 m bounded) → 2D ESDF slice → Nav2** with NO
HITNet GPU contention (only nvblox + OKVIS). HITNet stays for the high-quality offline/global map.

## v2 plan (two phases)

**v2a — live local costmap (NOW):** a node consuming slamko's corrected `odom→base` + D455
HW-depth → nvblox bounded rolling (5 m, decay on) → publish the native 2D ESDF slice into a
Nav2 `nvblox_costmap_layer` + stream to Rerun. Memory: sub-GB GPU. The realtime nav surface.

**v2b — lifelong global store (LATER):** swap the global nvblox export for **per-submap
VDBFusion TSDF tiles**, each rigidly attached to its slamko pose-graph anchor (supereight2 /
voxgraph model), **re-posed on loop closure (never re-integrated)**, persisted as `.vdb` in the
`.smap` archive, **paged in/out by region** around the robot. Kills the 751 MB depth-store + the
linear-growth problem → km-scale in single-digit GB on disk, hundreds of MB resident.

**One-line architecture:**
`D455 HW-depth → nvblox (5 m bounded, GPU) → 2D ESDF slice → Nav2 local costmap`  **‖**
`per-submap VDBFusion TSDF tiles ↔ slamko anchors (re-posed on loop) → paged .vdb store → in-region composite → global plan`.

## Hard rules carried in
decay OFF for global (ON for the local rolling); honest-unknowns (no generative fill); Z-gate
multi-floor; re-pose tiles, never re-integrate (the bend); license: nvblox Apache, OpenVDB
MPL-2.0, Bonxai MPL-2.0 — all OK.
