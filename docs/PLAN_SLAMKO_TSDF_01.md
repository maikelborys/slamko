<!-- validated: <pending — design doc, no code yet> -->
# PLAN — `slamko_tsdf`: deformable volumetric map for A→B navigation (v0)

> **Status:** DESIGN (rumbo fixed with the user 2026-06-22; no code yet).
> **Reading order:** [`../CLAUDE.md`](../CLAUDE.md) → [`../MASTER_PLAN.md`](../MASTER_PLAN.md) → this.
> **Research provenance:** 4 parallel agents (2026-06-22) digesting `~/coding/plvs/`
> (volumetric deformation), `~/coding/RTABmap/` (validated nav stack + hard-won
> warnings), the two existing semantic packages, and the 2024–26 semantic-mapping SOTA.
> Findings are summarized inline below; this doc is the operational plan.

## Why this exists

slamko owns the hard part (poses, landmarks, loop closure, lifelong reloc). It does
**not** yet produce a map a planner can route over. `slamko_tsdf` is the **opt-in**
volumetric layer that turns the corrected pose-graph into a **dense map that BENDS
with the graph** and exports a **2D costmap / ESDF for global A→B planning**. The
local controller is already fine (user: *"navegación local ya planifica como debería"*)
— this feeds the **global** plan only.

It realizes the deferred `slamko_mapping` role for the volumetric piece. Named
`slamko_tsdf` per the user. Pluggable behind a `slamko_core` contract like every
other slamko capability — *"un módulo que podría utilizarse o no."*

## Decisions locked (the rumbo)

| Decision | Choice | Why |
|---|---|---|
| Home | **Native `slamko_tsdf`** module, opt-in | self-sufficient, decoupled; not bound to the external RTABmap stack |
| Backend | **nvblox only** (GPU TSDF→ESDF) | the **bet is all-live**; nvblox's GPU speed wins at the live destination → avoid a backend swap later |
| Deformation granularity | **per-keyframe (PLVS-grade)** | smooth bend, no seams, even on a 1 km loop |
| Depth source | **HITNet / ESS stereo-net** (GPU) | bags are stereo-IR + RGB, **no depth topic**; dense quality for a clean costmap |
| Scope v0 | **offline-first, single floor** | de-risk: measure the A→B costmap quality before wiring live |
| Order | **volumetric/nav first**, semantic later | user marked semantic as "otra tema" |

## The load-bearing insight (why it bends at all)

From all four research streams + slamko's own architecture: **never store voxels in
global coordinates.** A fused TSDF is *fuse-and-forget* — once depth is integrated it
blends with neighbors and is **not re-poseable**. A single monolithic nvblox grid
**does NOT bend** on loop closure (same lesson as the `slamko-lifelong-fusion-ABE`
memory: rigid-SE3 can't absorb path-growing drift).

**The bend is an ARCHITECTURE property, not an nvblox feature:**
> Keep the **per-keyframe depth source** (re-poseable). The TSDF is a **DERIVED**
> product, re-integrated from the **corrected keyframe poses**. PLVS does exactly this
> (`PointCloudMapping::OnMapChange` → group points by `kfid` → correct by
> `Twnwo = TwcNew·TwcIntegration⁻¹` → clear + re-insert). RTAB-Map does the same as
> "ensamblado on-demand" (nothing in world coords; global map = assembled from graph
> poses each render). Hydra: derive semantics from the corrected geometry, never as
> independent free variables.

**Offline-first makes this nearly free:** optimize the graph to the end → re-integrate
the TSDF **once** from the final corrected poses at export. Perfect smooth bend **by
construction** (it's derived from the final graph), no incremental deformation needed.

## The honest hard part (the risk in the all-live bet)

Offline, re-integrate-once is cheap and exact. **Live is the open problem:**
re-integrating a per-keyframe TSDF on *every* loop closure is too expensive at scale.
The live fallbacks (to be researched in the live phase, NOT v0):
- **submap-tiled nvblox** (per-submap volumes, re-anchored rigidly) — piecewise bend,
  bounded cost (Voxgraph/c-blox/maplab pattern); seams ≈ 1 voxel at small submaps.
- **periodic / lazy re-integration** — re-fuse only the region a loop touched, only
  every N closures, or only the active window.
- nvblox's own decay + re-integration primitives as the live mechanism.

v0 deliberately sidesteps this by being offline. The plan keeps the **per-keyframe
depth source as the contract** so the live strategy can change without touching the
front-end.

## Pipeline (v0, offline)

```
slamko pose-graph (optimized, corrected anchors)
  + per-keyframe stereo (infra1/infra2)
        │
        ▼
[1] HITNet/ESS stereo-net (GPU) ── per-keyframe DEPTH image  ──┐  (kept as source)
        │                                                       │
[2] store (kf_id, depth, K, T_cam_kf)  in/alongside the SubMap  │
        │                                                       │
[3] EXPORT (once, after graph is final):                        │
      for each kf: pose = optimized graph pose of kf            │
      integrate depth into nvblox at that pose  ◄───────────────┘
        │
[4] nvblox → TSDF → ESDF → 2D ground-projected slice
        │
        ▼
   global 2D costmap / ESDF  (map frame)  ──►  global planner (A→B)
```

Module shape (pluggable):
- `slamko_core` contract: `VolumetricMap` consuming `(kf_id, depth, intrinsics,
  pose)` and emitting a costmap/ESDF export. Front-end (HITNet) and backend (nvblox)
  both behind interfaces so either can be swapped (voxblox/SGBM remain future options).
- Lives off the critical odometry path (Hard rule #4: the global graph is disposable;
  fast odometry never depends on it). `slamko_tsdf` is downstream of the graph, opt-in.

## Baked-in warnings (already paid for in `~/coding/RTABmap/` — do NOT re-learn)

- **`decay` OFF** for the global map — with decay on, only the final region survives.
- **Honest unknowns only** — no Poisson/NKSR/generative surface fill: they hallucinate
  free-space → ghost obstacles-free zones → unsafe nav. Sparse→TSDF keeps unknown=unknown.
- **Z-gate for multi-floor** (deferred past v0): casa+escaleras collapses 6.6 m→1.7 m
  without rejecting cross-floor loops (`|odom ΔZ| > 1.5 m`).
- **nvblox contends for GPU** with OKVIS + XFeat + HITNet — slamko's #1 nondeterminism
  gotcha. Offline-first dodges it; live phase must budget GPU (stagger depth rate,
  or run depth at rate ≤ 0.5 like the existing bench discipline).
- **GT-2D Y-mirror** (Isaac) — validate nav on real VIO, not synthetic GT.

## Phasing

- **v0 (this plan): offline, single floor.** HITNet depth on the casa bags
  (40cm / 100cm / Suave), re-integrate nvblox once from corrected poses, export 2D
  costmap, confirm a global planner routes A→B sanely. Measure: costmap connectivity
  (free-space carved, no ghost walls), bend correctness (revisit overlaps, no doubling),
  export time.
- **v1: multi-floor.** Add Z-gate; casa+escaleras as a multi-level map.
- **v2: live (the bet).** Solve the live re-integration problem (submap-tiled / lazy
  re-integration / nvblox decay). nvblox as the live local layer + the deformable
  global. This is where the nvblox-over-voxblox choice pays off.

## Reuse map (don't reinvent)

- **From `~/coding/RTABmap/`:** `TSDF_EXTRACTION.md` (Open3D TSDF-from-keyframes, honest
  unknowns, 0.71 s/session — the offline export recipe), `NVBLOX_COSTMAP.md` (nvblox→ESDF
  slice→Nav2 wiring, validated E2E in Isaac), `costmap_esdf_slice.py` (per-column
  floor-anchored 2D slice, robust to z-drift), `GLOBAL_COSTMAP_NAV2.md` (ray-carved grid
  beats raw slab as the planner surface). **Reuse the recipes + the warnings.**
- **From `~/coding/plvs/`:** the **idea** of `OnMapChange` per-`kfid` re-integration
  (`PointCloudMapOctreePointCloud::OnMapChange`, `PointCloudKeyFrame::TwcIntegration`).
  **PLVS core is GPL (ORB-SLAM3) — reuse the approach, not the code** (Hard rule #1).
- **License:** nvblox (Apache-2.0), HITNet/ESS (check the specific port), all clean for
  slamko's Apache/BSD constraint.

## Open questions for v0 kickoff

1. HITNet/ESS exact port + TRT engine (the RTABmap stack already has one wired — reuse?).
2. Where the per-kf depth is stored — extend `.smap` (SMP7 block) or a sidecar depth
   store? (Depth images are heavy; sidecar likely.)
3. Costmap export format — `.pgm`+`.yaml` (nav2_map_server) vs live OccupancyGrid topic.
4. Which global planner consumes it for the A→B validation (Nav2 NavFn on the exported grid?).

## Semantic layer (deferred — captured so it isn't re-researched)

SOTA synthesis (2026-06-22) is logged for phase 2. Verdict in one line: **build
object-instances carrying one CLIP vector, anchored to keyframes** (ConceptGraphs/HOV-SG
regime, ~75% lighter than dense per-voxel CLIP, loop-closure-safe). Live open-vocab
detector = YOLO-World / NanoOWL; offline auto-label = Grounded-SAM; stable labels =
multi-view weighted aggregation at the instance level + dual geometric+CLIP association
(= slamko's existing persistent-MapPoint dedup). You already have two working packages
(`semantic_mapping`, `rtabmap_semantic_mapping`) to mine. **Not touched until A→B ships.**
