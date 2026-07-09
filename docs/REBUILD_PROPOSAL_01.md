# slamko — Rebuild proposal: lifelong loose-fusion SLAM over external odometry

<!-- status: ADOPTED 2026-06-12 — user decision, criterion verbatim: "A mí me importa el
resultado. Y que sea bien estable sin romperse." Result + stability first; the paper door
stays open via the provider contract (see §11 amendments). §11 also folds in the
2026-06-12 research refresh (5-agent swarm, cross-verified) that re-validates §4's
direction and corrects two points. MIGRATED 2026-06-12: this doc is now the
research-provenance record; the operational plan lives in MASTER_PLAN.md (v2).
P-A started same session. -->
<!-- history: forks locked 2026-05-29; §4 originally folded from a research run whose
verifier crashed (not independently re-verified then). The 2026-06-12 swarm re-verified
the §4 claims — see §11.3. -->

**What this is:** the design proposal for the strategic pivot decided in the
2026-05-29 session. It supersedes the "slamko implements its own VIO" framing of
`MASTER_PLAN.md`. Read it, redline it; on approval it becomes the new `MASTER_PLAN`
and `CLAUDE.md` focus.

---

## 0. The pivot (what changed and why)

**Old slamko:** a from-scratch modular SLAM that *implements its own VIO*
(XFeat/KLT/IMU + GTSAM fusion + loop/reloc). Validated to ~10 cm ATE on MH_01,
healthy VIO (scale 1.01 EuRoC), recall fix designed (EigenPlaces R@5=1.0).

**New slamko:** slamko **stops implementing odometry** and becomes the
**lifelong map + multi-session relocalization + multi-sensor loose-fusion layer**
*on top of a mature external odometry provider*. It is the DigiForest/VILENS idea
done at the **map/fusion altitude**, not the raw-measurement altitude.

**Why (the user's diagnosis):** slamko's own VIO drifts on stairs (bias absorbs
gravity error without excitation) and across floors. Rather than spend the big
effort to close that (global landmark BA), stand on a provider that already
handles it (OKVIS2-X). The remaining pains — **multi-floor drift, never-lost,
day/night relocalization** — are *not* odometry-ATE problems; they live in the
map/reloc layer, which is **agnostic to the odometry underneath**.

### Decisions locked this session
1. **Coupling = LOOSE.** Consume the provider's keyframe-relative odometry +
   covariance into slamko's own pose-graph. (Tight would buy ~50–62% ATE — a
   metric that is *not* the bottleneck — at the cost of marrying OKVIS2-X's
   internals and losing provider-swappability.)
2. **Default provider = OKVIS2-X** (BSD-3, verified; VI + depth + LiDAR + GNSS;
   already in the workspace; tight *internally* so its output is high quality even
   consumed loose).
3. **The Atlas / multi-map lives IN slamko**, harvesting best practices from
   ORB-SLAM3 Atlas, maplab, experience-maps — built clean (no GPL code).
4. **Target = lifelong, world-scale, georeferenced, semantically-extensible.**
   See §1.

### Why NOT the alternatives (license + coupling reality)
| Provider | License | Verdict |
|---|---|---|
| **OKVIS2-X** | **BSD-3** ✅ | Default provider. |
| cuVSLAM | NVIDIA proprietary *(2026-07-09: now OPEN SOURCE v16 — fork extendable; adapter shipped)* | Optional *runtime-only* loose provider (binary; GPU-locked; cannot ship/extend). *(claim superseded — see RESEARCH_CUVSLAM_OPENSOURCE_01.md)* |
| VINS-Fusion | **GPLv3** ❌ | Blocked by Hard Rule #1 — cannot link into slamko. |
| OpenVINS | GPLv3 ❌ | Blocked. |
| Basalt | BSD-3 ✅ | Clean alternative loose provider. |
| RTAB-Map | BSD-3 ✅ | Already cloned; clean. |
| maplab | Apache-2.0 ✅ | Already cloned; harvest map-layer best practices. |

---

## 1. The vision → the constraints it imposes NOW

**Vision (user, verbatim intent):** the robot lives 1–50 years; the map can grow to
a **world/planetary scale anchored to global coordinates**; it stays **very
robust**; later it gains **semantic layers** (sidewalks, roads, lanes).

This is *lifelong, planet-scale, georeferenced, multi-layer SLAM*. None of it ships
in v1 — but the data model and contracts must not foreclose it. The eight
**must-not-foreclose constraints**:

1. **Map is tiled & out-of-core from day 1.** Never a monolithic in-RAM global
   graph. The map is a spatially-indexed set of submaps/tiles, lazily
   loaded/unloaded. (A planet does not fit in RAM.) → `SubMap` carries a spatial
   tile key.
2. **Every submap carries a global-frame anchor — even uninitialized.** Reserve
   `T_global_submap` + datum id + covariance in the schema now; GNSS *observes* it
   later. This is how "georeferenced in the future" stays a cheap add, not a rewrite.
3. **Multi-appearance per place.** A place stores a *set* of VPR descriptors
   accumulated over time/condition (day/night/season), not one. This is what makes
   day/night recall and 50-year robustness work. EigenPlaces vectors append on revisit.
4. **Bounded growth / summarization is first-class.** Over years you cannot keep
   every keyframe. Landmark sparsification + keyframe culling (maplab-style) must be
   a designed operation, not an afterthought.
5. **Layered, extensible map schema.** Base = pose-graph + submaps (geometry/
   occupancy). Semantic layers (sidewalks/roads) attach to the same submaps later
   *without touching the base contract* (the `custom_data`/layer escape hatch;
   Hydra-style scene graph as the conceptual target).
6. **Disposable graph, durable map (GLIM principle).** Separate "the optimization
   graph we rebuild on the fly" from "the persisted, versioned MAP." The graph is
   throwaway; the map is the durable artifact. Critical for lifelong.
7. **Versioning & provenance.** A 50-year map needs session provenance per submap,
   version history, and the ability to roll back a bad merge.
8. **Global vs relative split (loose-fusion-native).** Odometry (OKVIS2-X relative
   KF deltas) → **relative edges**. GNSS / cross-session relocalization → **global
   constraints** on submap anchors. (Validated by chain-pose-graph, §2.)

---

## 2. The fusion core — loose chain-pose-graph over external odometry

**Template (research-verified):** *chain-pose-graph fusion* (Merfels & Stachniss,
IROS 2016) — fuse the **pose+covariance outputs** of independent providers in a
sliding-window second graph, explicitly to *"incorporate third-party localization
modules for which source code is unavailable."* Handles multi-rate, out-of-sequence,
latency. Distinguishes **relative** constraints (odometry → edges between pose
nodes) from **global** constraints (GPS/reloc → observed nodes). This is exactly
slamko's heterogeneous-provider problem.

**Provider contract (slamko_core):** each provider emits, behind one interface:
- relative KF-to-KF transform `T_{k-1,k}` + covariance (the odometry edge), and
- optionally landmarks/covariance and a global anchor observation.

**Double-loop-closure avoidance:** consume OKVIS2-X's *relative keyframe deltas*
(which already absorb its internal loop closures locally) as relative edges — **never
its re-corrected global pose**. slamko's own relocalization supplies the global
constraints. Two systems never both inject global loop corrections. *(This is a
design inference from the relative/global split, not a directly verified result —
flagged for validation.)*

**Coupling cost, eyes open:** VILENS measured tight at ~62%/51% better than a loose
baseline — but that is **ATE**, which is not slamko's bottleneck. The loose tax
falls on the metric the user does not care about; robustness/recall (what the user
*does* care about) lives in the map layer and is coupling-agnostic.

**Degradation = covariance, never a branch** (Hard Rule #3, VILENS-confirmed): a
degraded provider inflates its covariance; the graph naturally down-weights it. No
`if(sensor_ok)` in the backend.

---

## 3. The never-lost spine (unchanged philosophy)

Three safety nets, three timescales (from `MASTER_PLAN` §3 — kept):
1. **Short gap (ms–0.5 s):** the provider's own IMU dead-reckoning.
2. **Medium loss → multi-map:** OK→RECENTLY_LOST→LOST state machine; **archive,
   don't discard** — seal the active submap, branch a fresh one, keep producing
   odometry. (Reuse slamko's validated decoupled supervisor.)
3. **Long loss / kidnap / new session:** relocalize (EigenPlaces retrieval + XFeat/
   LighterGlue verify) and weld with a relative-pose constraint.

Disposable global graph + catch-damp-rebuild (GLIM). The graph never crashes the
odometry; the odometry never depends on the graph.

---

## 4. The map — Atlas, lifelong, georeferenced

The differentiating layer. **Sourcing caveat:** the items below are primary-sourced
(ORB-SLAM3 paper `2007.11898` + `Atlas.cc`; maplab 2.0 paper `2212.00654` + wiki;
Hydra/Spark-DSG) and consistent with established knowledge, **but the dedicated
research run (w3o3dbfyd) hit a verifier-tooling failure (all verifiers abstained,
not refuted), so they were not independently re-verified by that run.** Treat as
design reference; confirm at implementation.

### Headline: maplab 2.0 is a working Apache-2.0 reference of slamko's exact design
maplab 2.0 (ETH, arXiv `2212.00654`, Apache-2.0) **consumes external 6-DoF odometry
through a method-agnostic interface** (no own estimator/IMU) and is driven in the
paper by **OKVIS, ROVIO, and FAST-LIO2 as interchangeable sources** — i.e.
loose-over-black-box-odometry, *exactly* slamko's plan. It does multi-session
submapping, cross-session loop closure, **RTK-GPS anchoring**, and **ILP landmark
sparsification** for bounded growth, validated at 23 runs / ~10 km / 2+ h. Already
cloned (`~/coding/maplab`). → **Primary harvest target; evaluate adopt-vs-
reimplement for the map-server before writing one from scratch.**

### Atlas multi-map mechanics (from ORB-SLAM3 — ideas only, it is GPL)
- **Spawn:** on tracking loss, first try to relocalize against ALL maps; only if
  that fails for a time, store the active map as non-active and start a fresh one.
  (slamko's never-lost seal→branch is the same idea.)
- **One active map** (gets new KFs) + a set of non-active maps; **one shared
  place-recognition DB** serves relocalization, loop closure, and merge.
- **Loop vs merge = same detector, different target:** a place match *within* the
  active map → loop closure; a match *into a different map* → multi-map data
  association → **merge**.
- **Merge = welding:** welding window (matched KF + covisibles both sides + their
  points, transformed by the relative `Tma`) → **local welding BA** →
  **essential-graph pose-graph optimization** propagating the correction while
  keeping the welding area fixed.

### maplab map-server blueprint (Apache-2.0, portable)
- **Submapping:** split into submaps at intervals → server preprocesses each (local
  BA + intra-map loop closure) → concatenate into a globally consistent map →
  global inter-mission loop closures merged afterward.
- **Multi-session anchoring:** designate one base frame as known, align all missions
  to it (vs each map floating in its own arbitrary frame).
- **Cross-session pipeline:** `relax` (pose-graph) → `lc` (loop closure) → `optvi`
  (VI bundle adjustment), iterated to convergence.
- **Bounded lifelong growth:** landmark sparsification as an ILP that keeps the
  most-observed landmarks while preserving spatial coverage (not random)
  — `lsparsify --num_landmarks_to_keep N`.

### Lifelong / multi-appearance
Store **multiple appearance descriptors per place** (experience-maps idea); a night
revisit *appends* rather than overwrites — the basis for day/night/seasonal
robustness over years. Change detection + a forgetting/pruning policy = open design.

### Layered semantics (future)
**Hydra / Spark-DSG** (MIT-SPARK) hierarchical 3D scene graph is the conceptual
target for sidewalks/roads/lanes as layers atop the metric map (check its license
before any code reuse).

### Map data model (the durable artifact)
- A set of **submaps**, each = local pose-graph of keyframes + landmarks +
  (optional) occupancy volume + **a global anchor** `T_global_submap` + **a set of
  appearance descriptors** + session/version provenance + a **spatial tile key**.
- A **global pose-graph** over submap anchors (relative edges between adjacent
  submaps; global constraints from GNSS + cross-session reloc).
- The optimization graph is rebuilt from this; the map is persisted/versioned
  independently (GLIM principle).

**Reconcile-first:** OKVIS2-X already has `ATLAS_DESIGN_01.md` +
`ROBUSTNESS_ARCHITECTURE.md`. Before building, decide what (if anything) transfers
from that design so the Atlas isn't built twice.

---

## 5. Day/night relocalization — reuse what's already de-risked

Keep the existing two-stage design (`docs/PLAN_VPR_RELOC.md`):
**EigenPlaces** (ResNet18, 512-D, MIT) global descriptor for retrieval →
**XFeat + LighterGlue** local match + geometric verification (PnP / 3D-3D with
depth) → relocalization factor. De-risked on real magistrale1: **R@1=0.85,
R@5=R@10=1.0**, negative control 0/30. Per-keyframe descriptors; multi-appearance
storage (§1.3) lets a night revisit *append* its descriptor rather than fail.

This module is **provider-agnostic** — it is slamko's, regardless of OKVIS vs
cuVSLAM underneath. It is the actual fix for the day/night and multi-floor pains.

---

## 6. Optional: stereo-depth submap-alignment factor (covariance-weighted)

The user's "ICP-from-stereo instead of LiDAR" idea, done the principled way.
Research-verified: OKVIS2-X couples the estimator to dense occupancy submaps via
**map-alignment factors built from depth images**, and the uncertainty-aware
variant weights the residual by depth uncertainty `W = 1/(σ²_map + σ²_depth)` —
**not a hard ICP constraint** (matches Hard Rule #3).

Scope honestly: this is for **free-space mapping (Nav2) + geometric loop
verification + robustness**, *not* the ATE fix (the user's own OKVIS investigation
showed submap-ICP is off-by-default and not what gets OKVIS to 3.22 cm). Add it as
an optional factor, not a core dependency.

---

## 7. Module mapping — what each package becomes

| Package | Becomes | Delete / Reuse |
|---|---|---|
| **slamko_core** | Provider contract (relative+global+covariance), tiled/anchored/multi-appearance `SubMap` schema, datum/tile types, map-versioning contract, health ifaces | Keep + extend the schema |
| **slamko_vio** | **Thin adapters** wrapping external providers (OKVIS2-X first; cuVSLAM/Basalt later). **No own VIO.** | **DELETE** the XFeat+KLT+IMU VIO + the broken gtsam path |
| **slamko_fusion** | The loose chain-pose-graph fixed-lag fuser (relative edges + global constraints), GTSAM | Repurpose; drop the local-VIO-specific machinery |
| **slamko_loop** | Atlas multi-map + lifelong map mgmt + EigenPlaces reloc + never-lost supervisor + GNSS anchoring | **Reuse** EigenPlaces, supervisor, submap IO, LighterGlue |
| **slamko_mapping** (split early now) | Lifelong tiled map-server: out-of-core store, summarization, versioning, georeferencing, semantic-layer hooks | New — promoted from deferred because it is now core |
| **slamko_msgs / slamko_ros** | Provider topics, map-server API, map→odom slewed correction, Nav2 bridge, viz | Reuse bridge patterns from okvis_nav2_bridge |

**Net:** delete the own-VIO + broken-smoother code; reuse the reloc/supervisor/
submap-IO investments; promote the map-server to a first-class lifelong subsystem.

---

## 8. Licensing
Apache-2.0 / BSD-3 only (Hard Rule #1). Provider matrix in §0. Harvest *ideas* from
ORB-SLAM3 (GPL) — never code. maplab (Apache), supereight2/OKVIS2-X (BSD-3),
EigenPlaces (MIT), GTSAM (BSD-3) are all clean.

---

## 9. Open questions
1. **adopt-vs-reimplement maplab 2.0** for the map-server — it already does
   loose-over-external-odom + multi-session + RTK-GPS anchoring + ILP
   sparsification (Apache-2.0, cloned). The single biggest scoping decision: build
   the Atlas/map-server from scratch, or build slamko's reloc/never-lost layer on
   top of maplab's map structure? *(New — surfaced by research.)*
2. **GNSS anchoring** — maplab's pattern is absolute RTK-GPS pose constraints +
   a designated known base frame; open part is *incremental online* anchoring (vs
   maplab's offline-console flow).
3. **Atlas reconciliation** with OKVIS2-X's existing `ATLAS_DESIGN_01.md`.
4. **Map summarization** — direction is maplab's ILP landmark selection (keep
   most-observed + spatial coverage); open part is the *online/continuous* policy
   and a change-detection/forgetting rule for lifelong.
5. **Tiling scheme** — geohash vs H3 vs UTM zones for the spatial index (maplab
   does not do planet-scale tiling/out-of-core — this is slamko-net-new).
6. **Double-loop-closure avoidance** — validate the relative-deltas approach
   against OKVIS2-X's actual output API.

---

## 10. Phased roadmap (proposed)
- **P-A — Provider adapter + loose fuser:** OKVIS2-X adapter → relative KF edges +
  covariance into slamko_fusion's pose-graph; map→odom slewed correction; validate
  the live pose tracks OKVIS2-X on a bag. *(Smallest end-to-end loop.)*
- **P-B — Reloc reattach:** wire EigenPlaces (finish the C++ from PLAN_VPR_RELOC) as
  the global-constraint source; validate day/night reattach + multi-floor snap.
- **P-C — Atlas + lifelong map-server:** multi-map spawn/merge/weld; tiled out-of-
  core store; multi-appearance; summarization; versioning. *(The big build —
  research-informed.)*
- **P-D — Georeferencing:** GNSS anchors submaps to a global datum.
- **P-E — Extra providers + depth-submap factor:** cuVSLAM/Basalt loose adapters;
  optional stereo-depth submap-alignment.
- **P-F (future) — Semantic layers:** sidewalks/roads/lanes as map layers.

---

## 11. Addendum 2026-06-12 — adoption, amendments, research refresh

### 11.1 The decision and its criterion

Adopted as written, with the amendments below. The user's criterion: **result +
stability ("que sea bien estable sin romperse")** outranks paper novelty. OKVIS2-X
is the default provider because it is the mature, already-validated choice (user's
own magistrale run: OKVIS 5.91 cm vs slamko-own-VIO 10.06 m; stairs/multi-floor
solved; BSD-3). Every hour not spent hardening an own VIO goes into the layer that
actually delivers "never breaks".

### 11.2 Amendments

1. **klt_vo = second provider via the SAME contract (future, not now).** Do not
   read §7's "DELETE the own VIO" as abandoning klt_vo — it lives and improves in
   its own repo (`~/coding/klt_vo`). Its 2026-06-12 watershed state: FB-check
   (+30% mean ATE), LocalBA revived (the dead-prune bug slamko_vio still carries
   was FIXED there), Schur+FEJ marginalization working (MH_05 0.092 median),
   clean-room XFeat TRT detector (MH_01 0.040 median, beats local cuVSLAM;
   190 fps @640×480 on the real D455 casa bag), three median-of-3-validated
   configs. When the high-fps robot regime needs it, klt_vo plugs in as a thin
   adapter — slamko_vio's stale fork of it should still be deleted as §7 says.
2. **slamko_vio's diagnosis is partially superseded.** The "own VIO drifts on
   stairs" motivation (§0) predates the klt_vo 2026-06-12 fixes (part of that
   drift was the dead-BA bug). This does NOT change the decision — OKVIS2-X is
   still the stable choice — but it removes any urgency to salvage slamko_vio's
   VIO code: klt_vo HEAD is strictly better, so the fork has zero unique value.
3. **Paper framing preserved, deprioritized.** The 2026-06-12 novelty check found
   three EMPTY claims: (a) no system combines >100 fps embedded-GPU front-end +
   incremental factor-graph backend + never-lost multi-map; (b) no published
   4-modality fusion (cam+IMU+wheel+magnetometer) in one graph; (c) no
   recovery-first framing in 2024–2026 stereo-inertial literature. All three are
   reachable LATER by swapping the provider to klt_vo + P5 sensors. cuVSLAM
   (arXiv:2506.04359) has speed + LC but no multi-map, no appearance reloc, no
   extra sensors, restrictive NVIDIA license.

### 11.3 Research refresh (2026-06-12 swarm — supersedes the crashed-verifier caveat on §4)

**Anchor-don't-weld is now multiply confirmed; §4's ORB-SLAM3 welding mechanics
are reference-only, NOT the plan:**
- Hydra-Multi (arXiv:2304.13487) has the ablation: removing anchoring is
  catastrophic; removing node merging costs only modestly.
- maplab 2.0 (arXiv:2212.00654): anchoring-first staged design (solve `T_G_M`
  only; landmark merge = offline refinement). Kimera-Multi (arXiv:2106.14386) +
  RTAB-Map (arXiv:2301.00050): never fuse landmarks at all, anchor-only works
  for 10+ km / 11 sessions.
- CudaSIFT-SLAM (arXiv:2405.16932, Tardós 2024): +70% mapped frames purely from
  better place-recognition recall → **the Atlas bottleneck is RECALL, not merge
  machinery**. Directly validates spending P-B on the VPR/reloc layer.
- **Merge as reversible transaction** (Bosch lifelong, arXiv:2211.03423): merge →
  post-hoc map-agreement check → undo if bad. ROVER (arXiv:2508.13488):
  trajectory/inertial-consistency gating of candidates. → slamko's anchor edges
  must be **covariance-gated, inertially sanity-checked, and reversible**.
  4-DoF anchors suffice (both maps gravity-aligned).
- Documented Atlas failure modes to design against: wrong merges under aliasing;
  recall-too-low → permanent fragmentation (the dominant one); immature-map VI
  scale error (~5% at 2 s, ~1% at 15 s — don't anchor a too-young branch);
  merge-time compute spikes; unbounded multi-session graph growth.

**GTSAM/iSAM2 — keep, but poses-only and pinned:**
- iSAM2 remains the only production incremental backend (no GPU incremental
  solver exists anywhere; MegBA/DeepLM/Graphite/SymForce are all batch).
- Known structural pitfalls: unbounded memory with landmarks in long sessions;
  `marginalizeLeaves` fragility; smart factors break inside
  IncrementalFixedLagSmoother; TBB can be slower (parallelizes linearization
  only); relinearization spikes on big loops.
- → The GLIM/Kimera split: metric fixed-lag smoothing stays INSIDE the provider;
  slamko's global graph = **iSAM2 over poses/anchors only, no landmarks ever**.
  Pin GTSAM 4.2.1 (or GLIM's tested 4.3a0), benchmark with TBB off. GPU effort,
  if any, goes to factor evaluation (gtsam_points pattern, Jetson-validated).
- This also explains slamko_fusion's own 2026-05-29 finding (GTSAM local smoother
  15× worse than Ceres): wrong altitude for GTSAM, not a GTSAM defect.

**Magnetometer (P5, the casa bags already carry the data):**
- BNO055 **fused orientation is NOT usable as a factor**: ~180° heading re-snaps,
  calibration collapses in motion, no calibration persistence, silent yaw
  re-snaps poison a smooth-measurement factor.
- Correct path: **raw `/bno055/mag`** (already recorded @~66 Hz in
  CASA1_*_BNO bags) + Kok-Schön ellipsoid calibration (arXiv:1601.05257) done
  on-robot WITH MOTORS RUNNING + unary yaw factor (GTSAM `MagPoseFactor` pattern,
  WMM/IGRF field vector for true north) + norm/dip gating + Cauchy kernel.
- Indoor: **gated OFF by default** — 6.9–25.9° yaw errors documented near
  steel/equipment; both reference mag-VIO systems restrict to outdoor. No
  published quadruped-actuator mag study exists (gap AND warning).
- Hardware note: BNO085/086 (calibration persistence, better dynamics) is a
  drop-in robustness upgrade.

**GNSS/RTK (P-D):**
- Loosely-coupled is fully covered by GTSAM `GPSFactor`/`GPSFactorArm` (lever arm
  makes attitude observable), unary in ENU with per-fix covariance.
- **Gauge-correct pattern (Lee et al., ICRA 2020): do NOT keep the VIO↔ENU 4-DoF
  transform as a state (stays unobservable) — initialize it over a window, then
  re-anchor the estimator into ENU** and use plain unary factors.
- Robustness ladder: Huber/Cauchy for RTK-fixed; switchable constraints / GNC
  (arXiv:2109.00667) only if ingesting float/SPP multipath fixes.
- OKVIS2-X itself ships yaw-observability-gated 4-DoF GNSS alignment
  (arXiv:2510.04612) — read it before building anything.

### 11.4 Roadmap, reprioritized for stability-first (gates on the user's REAL bags)

| Phase | What | Gate (must be reproducible, not single-run) |
|---|---|---|
| **P-A** | OKVIS2-X adapter → relative KF edges + cov into the loose fuser; map→odom slew | Live pose tracks OKVIS on CASA1_Suave + Escaleras bags |
| **P-B** | Reloc recall: EigenPlaces per-KF diagnostic on magistrale return (dump per-KF cosines + attempt counts); SALAD/CosPlace fallback if needed | Cross-session reloc on casa bags; magistrale start↔end bridge closes (today: 14%) |
| **P-C** | Never-lost end-to-end: stale-gap → seal → branch → reloc → **reversible gated anchor** | `CASA1_Suave_blackout` + `blackout4`: clean recovery, zero crashes, un-aligned divergence bounded |
| **P-C′** | Swap slamko_loop's batch Ceres pose-graph → iSAM2 poses-only incremental | Escaleras multi-floor with real-time incremental global correction |
| **P-D/E/F** | Georef (init-then-re-anchor) / extra providers (klt_vo!) / semantics | unchanged |

Validation data inventory: EuRoC (full GT, median-of-3 on V1 — single runs are
±50% noise, klt_vo-proven), `/mnt/data/bno_ab/CASA1_{Suave,Escaleras,extremeFinal}_
Stereo60_RGB30_BNO` (+`_strim`/`_trim` variants, raw BNO mag included),
`CASA1_Suave_blackout{,4}` (the kidnap/recovery protocol bags), c1/c2/c3a/c3b
series, TUM-VI magistrale (CAUTION: GT is room-only — ATE-gated evals on it are
meaningless; use masked segments or the bridge-closure criterion). OKVIS reference
TUMs: `~/coding/klt_vo/results/d455/okvis_{Suave,Escaleras}.tum`.

### 11.5 What the next session should do, in order

1. Migrate this doc → `MASTER_PLAN.md` (+ CLAUDE.md focus update); archive the
   old own-VIO MASTER_PLAN framing.
2. Resolve the uncommitted slamko_vio work-in-progress (health-trace edits +
   `vio_d455.launch.py`, present in the tree on 2026-06-12) — commit, harvest,
   or drop; don't leave it dirty.
3. Start P-A (smallest end-to-end loop). Open question §9.1 (adopt-vs-reimplement
   maplab's map-server) should be answered DURING P-A/P-C design, not before —
   data first.
