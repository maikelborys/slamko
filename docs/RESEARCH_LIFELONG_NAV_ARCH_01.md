<!-- validated: 3-agent research + casa A/B + wavemap measurement, 2026-06-23 -->
# Lifelong navigation architecture — frozen decision (volumetric + loops + outdoor)

> Decided after the casa pipeline run, the depth-to-OKVIS A/B, a wavemap memory
> measurement, and 3 parallel research agents (slamko landmark bounding · nvblox
> deformability/lifecycle · wavemap-vs-nvblox at scale). Supersedes the global-store
> half of [`RESEARCH_LIFELONG_VOLUMETRIC_BACKEND_01.md`](RESEARCH_LIFELONG_VOLUMETRIC_BACKEND_01.md)
> (VDBFusion was a guess; the real answer is **nvblox_submap (already built) now,
> wavemap deferred to scale**).

## The frozen stack

```
PROVIDERS (odometry):  OKVIS-VI (pure VIO, NO depth)  [+ future: LiDAR KISS-ICP/GLIM, klt_vo hi-fps]
                              | slamko contract: relative SE3 edges + covariance
slamko GLOBAL GRAPH:   loops = XFeat(visual) ⊕ ScanContext(geometric) ⊕ proximity-E
                       + GPS / compass anchors (outdoor)
VOLUMETRIC:  slamko_tsdf  (RE-INTEGRATE / PLVS-style; offline today → live next, nvblox backend)  ← KEEP
             XFeat landmarks  (bounded-by-area, ~+3%/visit plateau)
             [ wavemap = GLOBAL compact tier — DEFERRED to building/campus scale ]
PLANNER:  route on global map  +  reactive on nvblox local ESDF
```

## Decision 1 — Volumetric = slamko_tsdf (RE-INTEGRATE). nvblox_submap DROPPED.

**User decision (2026-06-23): keep `slamko_tsdf`, drop `nvblox_submap`.** One model, one codebase,
slamko-owned. The two are the two deformation philosophies:
- **slamko_tsdf = RE-INTEGRATE** (PLVS OnMapChange): keep per-kf depth, re-fuse at corrected poses →
  **seamless bend** (no submap seams). Offline today (`export_tool`, no ROS node — gave the casa map).
  Owns the `slamko_core::VolumetricBackend` contract. The header already flags the live hook:
  "the same call is the live hook later (re-integrate the touched window on a loop)".
- **nvblox_submap = RE-POSE** (Voxgraph, `~/coding/nvblox_ros2/nvblox_submap/`, Phases A–E built):
  freeze submaps, move anchors on loop, zero re-integration, µs, VRAM-bounded, piecewise-rigid.
  **NOT adopted** — would fork the live path into a second codebase/model (RTAB-anchored).

**Tradeoff accepted:** re-integrate keeps the seamless quality + a single slamko codebase, at the
cost that LIVE re-integration is the harder/expensive path (the original plan's "v2 RISK"). The live
build must solve: (a) bound the per-kf depth store (the 751 MB problem — compress/disparity/re-derive),
(b) re-integrate only the TOUCHED window on a loop (not the whole map) on the GPU. nvblox stays the
backend (TSDF/ESDF engine); slamko_tsdf drives it. nvblox_submap's re-pose stays on the shelf as the
fallback if live re-integration proves too costly.

→ "Does the volumetric deform with keyframes like Voxblox+PLVS?" **Yes — via slamko_tsdf re-integrate
(the PLVS model itself), offline now, live next.**

## Decision 2 — wavemap DEFERRED to scale (not worth it for a house)

Measured casa global map: **wavemap 5.3 MB (.wvmp) / 10 MB RAM** vs **nvblox ~28 MB / mesh 16 MB**
vs **OKVIS native 106 MB**. The 3× gap for ONE house is below the complexity-justification line —
user was right. wavemap pays off **at scale**: Newer College (large outdoor) = wavemap 242 MB vs
dense-Voxblox 2362 MB → **~2.1 GB saved** (ratio ~9–10× vs dense, ~4× vs Octomap, widening in
absolute terms). **The deciding factor is ESDF:** nvblox emits a TRUE Euclidean SDF (reactive
collision + the official Nav2 local costmap provider, GPU, 31× faster ESDF than Voxblox); wavemap
is **occupancy-only** (no ESDF; wavestar RSS-2025 adds a CPU quasi-ESDF + multi-res planner).
→ **House today = nvblox-only (simpler, right). Building/campus/lifelong = 2-tier nvblox-LOCAL
(GPU ESDF reactive) + wavemap-GLOBAL (compact, route planning, disk-paging basis).** Not a
replacement — opposite sides of the live-ESDF / compact-global tradeoff.

## Decision 3 — Bounding (does a robot-all-day inflate the map?)

**XFeat landmarks: NO, bounded-by-area, plateau.** Bounding ON by default (`cull_enabled`,
`cull_viewpoint_aware`, `occ_refresh` in provider_fusion_node.cpp): voxel dedup at seal (6.2×) +
cross-submap occupancy cull (main bound) + viewpoint-aware keep (omni-directional) + submap-cull
backstop (>70% redundant → cull whole submap). **Measured: +100%/visit baseline → +3.1%/visit**
(drift-tolerant 0.15 m voxel); finite house → finite voxels → asymptotic PLATEAU. Residual +3% is
run-to-run frame drift, NOT unbounded growth.

**nvblox TSDF: NO inflation on revisits — bounded by AREA, not visit count.** A voxel grid updates
the SAME 8³ blocks on revisit (weighted avg), allocates only for NEW area. The ONLY inflation =
pose DRIFT (same wall → different voxels → ghosting) — exactly what nvblox_submap's re-pose fixes.

## Decision 4 — OKVIS stays PURE VIO (don't feed it depth)

casa A/B (same front-end ± depth, loops OFF): odometry start-end drift over a 42 m loop = **0.12 m
without depth vs 0.32 m WITH depth** (worse), + OKVIS native map bloats to 106 MB. Single run each
(OKVIS GPU-contention nondeterminism → not statistically hard), but clearly no improvement. The
depth-submap-align (`SubmapIcpError`, field/TSDF-gradient, enabled in map848/se2) helps where VIO
DRIFTS (long/outdoor) — on a clean 42 m indoor loop the VIO is already 0.28% drift so there's no
room. → **Keep OKVIS pure VIO odometry; depth lives in the MAP layer (slamko_tsdf), not the
provider.** Empirically validates slamko hard-rule #4 (provider disposable, global map separate).

## Decision 5 — Loops & outdoor (the geometry/absolute-reference theme)

> Geometry is viewpoint-invariant where appearance (XFeat) fails; absolute references
> (GPS/compass) bound drift where loops can't.

- **Loops:** XFeat(visual, same-heading) ⊕ ScanContext-style(geometric, viewpoint-robust → opposite-
  facing) ⊕ proximity-E (pose-triggered). ScanContext is strong with 360° LiDAR, partial with a
  ~90° depth-cam FOV. Disjunctive gate: accept if visual OR geometric passes. Plugs the documented
  0% opposite-facing recall ceiling.
- **OKVIS depth submap-align (`SubmapIcpError`)** is OKVIS's existing field-based geometric channel —
  the model to port into slamko's loop verification (field-gradient > naive point-ICP).
- **Outdoor/descampado:** depth dies >6 m, open field has weak/repetitive landmarks → VIO drifts +
  loops unreliable → **NOT well-behaved without GPS+compass**. Rocks → no planes (plane-SLAM is an
  indoor/urban specialization) → use generic geometric registration (point/field), and GPS at range.
- **Magnetometer (`/bno055/mag`, recorded):** gated by field-norm (|B|≈25–65 µT; indoor metal = garbage).
  Uses: absolute-yaw prior (correct heading drift), independent yaw witness (measure drift → inflate
  cov/flag), most valuable OUTDOORS (where it's reliable AND most needed). slamko has `compass_yaw_prior`.

## Next (in order)
1. **LIVE nvblox TSDF in slamko_tsdf** (the chosen path): add the ROS node + live nvblox backend;
   re-integrate the TOUCHED window on each loop (not whole map); bound the per-kf depth store
   (751 MB → compress/disparity/re-derive). nvblox = backend engine, slamko_tsdf drives.
2. Geometric loop channel (ScanContext-style / field-align) as a disjunctive gate, proximity-E-triggered.
3. wavemap global tier — only when a multi-floor/building bag exists.

> nvblox_submap (re-pose) is DROPPED but kept on the shelf as the fallback if live re-integration
> proves too costly (GPU/depth-store). It is NOT to be developed in parallel.
