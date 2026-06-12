# slamko — Master Plan (v2: lifelong loose-fusion SLAM over external odometry)

<!-- validated: adopted 2026-06-12 · migrated from docs/REBUILD_PROPOSAL_01.md (full
research provenance + §11 addendum live there) · previous own-VIO plan archived at
docs/archive/MASTER_PLAN_OWNVIO_01.md -->

**What slamko is now:** slamko **does not implement odometry**. It is the
**lifelong map + multi-session relocalization + multi-sensor loose-fusion layer**
on top of a mature external odometry provider (**OKVIS2-X** default; klt_vo as a
future high-fps second provider via the same contract). The DigiForest/VILENS idea
done at the **map/fusion altitude**, not the raw-measurement altitude.

**Decision criterion (user, verbatim):** *"A mí me importa el resultado. Y que sea
bien estable sin romperse."* — result + stability outrank paper novelty. The pains
that matter — multi-floor drift, never-lost, day/night relocalization — are NOT
odometry-ATE problems; they live in the map/reloc layer, which is agnostic to the
odometry underneath. (OKVIS magistrale: 5.91 cm vs slamko-own-VIO 10.06 m.)

## 0. Locked decisions

1. **Coupling = LOOSE.** Consume the provider's keyframe-relative odometry +
   covariance into slamko's own pose-graph. Tight would buy ~50–62% ATE — a metric
   that is not the bottleneck — at the cost of marrying provider internals.
2. **Default provider = OKVIS2-X** (BSD-3; VI+depth+LiDAR+GNSS; in-workspace;
   tight internally so its output is high quality even consumed loose).
   Alternatives: Basalt (BSD-3), cuVSLAM (runtime-only, proprietary), klt_vo
   (future, same contract). VINS-Fusion / OpenVINS = GPL, blocked.
3. **The Atlas / multi-map lives IN slamko** (ideas from ORB-SLAM3/maplab/
   experience-maps; no GPL code).
4. **Anchor, don't weld** (2026-06-12 swarm, multiply confirmed): map merges are
   **covariance-gated, inertially sanity-checked, REVERSIBLE 4-DoF anchor edges**
   between submap frames — never landmark fusion. The Atlas bottleneck is
   **place-recognition RECALL**, not merge machinery (CudaSIFT-SLAM: +70% mapped
   frames purely from recall).
5. **Global backend = iSAM2 over poses/anchors ONLY — no landmarks ever.** Metric
   fixed-lag smoothing stays inside the provider. Pin GTSAM 4.2.1 (or GLIM's
   4.3a0), benchmark with TBB off. (This also explains the 2026-05-29 finding that
   the GTSAM *local* smoother was 15× worse than Ceres: wrong altitude for GTSAM,
   not a GTSAM defect.)
6. **Double-loop-closure avoidance:** consume the provider's *relative KF deltas*
   (they absorb its internal loop closures locally) — never its re-corrected
   global pose. slamko's relocalization supplies the global constraints. Two
   systems never both inject global corrections. *(Design inference — validate
   against OKVIS2-X's actual output API during P-A.)*
7. **Degradation = covariance inflation, never an `if(sensor_ok)` branch**
   (Hard Rule #3, VILENS-confirmed).
8. Apache-2.0 / BSD-3 only (Hard Rule #1). Harvest *ideas* from GPL systems,
   never code.

## 1. Vision → the eight must-not-foreclose constraints

Vision: the robot lives 1–50 years; the map can grow to world scale anchored to
global coordinates; very robust; later gains semantic layers (sidewalks, roads,
lanes). None of it ships in v1 — but the data model must not foreclose it:

1. **Tiled & out-of-core from day 1** — never a monolithic in-RAM global graph;
   `SubMap` carries a spatial tile key.
2. **Every submap carries a global-frame anchor** `T_global_submap` + datum id +
   covariance — even uninitialized; GNSS *observes* it later.
3. **Multi-appearance per place** — a place stores a *set* of VPR descriptors
   accumulated over time/condition; a night revisit *appends*, never overwrites.
4. **Bounded growth / summarization is first-class** (maplab-style ILP landmark
   sparsification + KF culling as a designed operation).
5. **Layered, extensible schema** — semantic layers attach to the same submaps
   later via the `custom_data`/layer escape hatch (Hydra-style target).
6. **Disposable graph, durable map** (GLIM) — the optimization graph is rebuilt
   on the fly; the persisted, versioned MAP is the durable artifact.
7. **Versioning & provenance** — session provenance per submap; roll back a bad
   merge (merge = reversible transaction, Bosch-lifelong pattern).
8. **Global vs relative split** — odometry → relative edges; GNSS/cross-session
   reloc → global constraints on submap anchors.

## 2. The fusion core — loose chain-pose-graph

Template: *chain-pose-graph fusion* (Merfels & Stachniss, IROS 2016) — fuse
pose+covariance outputs of independent providers in a sliding-window second graph;
handles multi-rate, out-of-sequence, latency; distinguishes relative vs global
constraints.

**Provider contract (slamko_core):** each provider emits, behind one interface:
- relative KF-to-KF transform `T_{k-1,k}` + covariance (the odometry edge), and
- optionally landmarks/covariance and a global anchor observation.

## 3. The never-lost spine (unchanged philosophy)

Three safety nets, three timescales:
1. **Short gap (ms–0.5 s):** the provider's own IMU dead-reckoning.
2. **Medium loss → multi-map:** OK→RECENTLY_LOST→LOST state machine; **archive,
   don't discard** — seal the active submap, branch fresh, keep producing
   odometry (reuse the validated decoupled supervisor).
3. **Long loss / kidnap / new session:** relocalize (EigenPlaces retrieval +
   XFeat/LighterGlue verify) → **reversible gated anchor** constraint.

Disposable global graph + catch-damp-rebuild. The graph never crashes the
odometry; the odometry never depends on the graph.

Documented Atlas failure modes to design against: wrong merges under aliasing;
recall-too-low → permanent fragmentation (the dominant one); immature-map scale
error (don't anchor a branch younger than ~15 s); merge-time compute spikes;
unbounded multi-session graph growth.

## 4. The map — Atlas, lifelong, georeferenced

- **Data model:** a set of **submaps** (local pose-graph of KFs + landmarks +
  optional occupancy + global anchor + appearance-descriptor set + provenance +
  tile key) + a **global pose-graph over submap anchors** (relative edges between
  adjacent submaps; global constraints from GNSS + cross-session reloc).
- **maplab 2.0 = the working Apache-2.0 reference** of this exact design
  (external-odometry interface, multi-session submapping, RTK anchoring, ILP
  sparsification; cloned at `~/coding/maplab`). Adopt-vs-reimplement its
  map-server is open question #1 — answer DURING P-A/P-C with data, not before.
- ORB-SLAM3 Atlas mechanics (spawn / one-active-map / shared PR database / loop
  vs merge = same detector different target) are **reference-only**; its welding
  BA is explicitly NOT the plan (anchor-don't-weld, §0.4).

## 5. Day/night relocalization (provider-agnostic, already de-risked)

Two-stage, from `docs/PLAN_VPR_RELOC.md`: **EigenPlaces** (ResNet18, 512-D, MIT)
retrieval → **XFeat + LighterGlue** local match + geometric verification (PnP /
3D-3D with depth) → relocalization factor. De-risked on real magistrale1:
R@1=0.85, R@5=R@10=1.0, negative control 0/30. Multi-appearance storage (§1.3)
makes night revisits append rather than fail. **This module is the actual fix for
the day/night and multi-floor pains.**

## 6. Future sensors (P-D/P-E, designed-for now)

- **GNSS/RTK:** GTSAM `GPSFactor`/`GPSFactorArm`, unary in ENU, per-fix
  covariance. Gauge-correct pattern: do NOT keep the VIO↔ENU 4-DoF transform as a
  state — initialize over a window, then **re-anchor into ENU**. OKVIS2-X ships
  yaw-observability-gated 4-DoF GNSS alignment (arXiv:2510.04612) — read first.
- **Magnetometer:** raw `/bno055/mag` (already in the CASA1 BNO bags @~66 Hz) +
  Kok-Schön ellipsoid calibration with motors running + unary yaw factor
  (`MagPoseFactor` pattern, WMM/IGRF) + norm/dip gating + Cauchy. BNO055 *fused*
  orientation is NOT usable (silent ~180° yaw re-snaps). Indoor: gated OFF.
- **Optional stereo-depth submap-alignment factor** (covariance-weighted, OKVIS2-X
  pattern `W = 1/(σ²_map + σ²_depth)`) — for free-space mapping + geometric loop
  verification, not ATE.

## 7. Module mapping

| Package | Is / becomes | Notes |
|---|---|---|
| **slamko_core** | Provider contract (relative+global+covariance), tiled/anchored/multi-appearance `SubMap` schema, datum/tile types, map-versioning contract, health ifaces | Keep + extend |
| **slamko_vio** | **Thin adapters** wrapping external providers (OKVIS2-X first) | **DELETE** the own VIO (klt_vo HEAD is strictly better — the fork has zero unique value; klt_vo itself lives on in its repo as a future provider) |
| **slamko_fusion** | The loose chain-pose-graph fixed-lag fuser (relative edges + global constraints) | Repurpose; drop local-VIO machinery |
| **slamko_loop** | Atlas multi-map + lifelong map mgmt + EigenPlaces reloc + never-lost supervisor + GNSS anchoring | Reuse EigenPlaces, supervisor, submap IO, LighterGlue |
| **slamko_mapping** (split early) | Lifelong tiled map-server: out-of-core store, summarization, versioning, georeferencing, semantic hooks | Promoted from deferred — now core |
| **slamko_msgs / slamko_ros** | Provider topics, map-server API, map→odom slewed correction, Nav2 bridge, viz | Reuse okvis_nav2_bridge patterns |

## 8. Roadmap (stability-first; gates on the REAL bags, reproducible not single-run)

| Phase | What | Gate |
|---|---|---|
| **P-A** | OKVIS2-X adapter → relative KF edges + cov into the loose fuser; map→odom slew | Live pose tracks OKVIS on CASA1_Suave + Escaleras bags |
| **P-B** | Reloc recall: EigenPlaces per-KF diagnostic on magistrale return (per-KF cosines + attempt counts); SALAD/CosPlace fallback if needed | Cross-session reloc on casa bags; magistrale start↔end bridge closes (today: 14%) |
| **P-C** | Never-lost end-to-end: stale-gap → seal → branch → reloc → **reversible gated anchor** | `CASA1_Suave_blackout` + `blackout4`: clean recovery, zero crashes, un-aligned divergence bounded |
| **P-C′** | Swap slamko_loop's batch Ceres pose-graph → iSAM2 poses-only incremental | Escaleras multi-floor with real-time incremental global correction |
| **P-D** | Georeferencing: GNSS anchors submaps to a global datum (init-then-re-anchor) | — |
| **P-E** | Extra providers (klt_vo!, Basalt/cuVSLAM) + optional depth-submap factor | — |
| **P-F** | Semantic layers: sidewalks/roads/lanes atop the metric map | — |

**Validation data inventory:** EuRoC (full GT; median-of-3 on V1 — single runs
±50% noise); `/mnt/data/bno_ab/CASA1_{Suave,Escaleras,extremeFinal}_Stereo60_RGB30_BNO`
(+`_strim`/`_trim`; raw BNO mag included); `CASA1_Suave_blackout{,4}`
(kidnap/recovery protocol); c1/c2/c3a/c3b series; TUM-VI magistrale (**CAUTION:
GT is room-only — ATE on it is meaningless; use masked segments or the
bridge-closure criterion**). OKVIS reference TUMs:
`~/coding/klt_vo/results/d455/okvis_{Suave,Escaleras}.tum`.

## 9. Open questions

1. **adopt-vs-reimplement maplab 2.0's map-server** — answer during P-A/P-C
   design, data first.
2. **Incremental online GNSS anchoring** (vs maplab's offline-console flow).
3. **Atlas reconciliation** with OKVIS2-X's `ATLAS_DESIGN_01.md`.
4. **Online/continuous summarization policy** + change-detection/forgetting rule.
5. **Tiling scheme** — geohash vs H3 vs UTM (slamko-net-new; maplab doesn't tile).
6. **Double-loop-closure avoidance** — validate relative-deltas against OKVIS2-X's
   actual output API (P-A).

## 10. Paper door (preserved, deprioritized)

Three verified-empty novelty claims stay reachable later by swapping the provider
to klt_vo + adding P5 sensors: (a) >100 fps embedded front-end + incremental
factor-graph backend + never-lost multi-map; (b) 4-modality cam+IMU+wheel+mag in
one graph; (c) recovery-first framing in stereo-inertial 2024–2026. No structural
cost now — the provider contract IS the door.
