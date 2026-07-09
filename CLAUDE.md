# slamko — central orientation & instructions (the one authoritative doc)

**slamko** is the **lifelong map + multi-session relocalization + multi-sensor
loose-fusion layer over an EXTERNAL odometry provider** (OKVIS2-X default; klt_vo
as future high-fps second provider via the same contract). slamko does NOT
implement odometry. Top priority: **never gets lost, never fails,
super-recoverable — while staying SIMPLE and STABLE.** Every sensor / capability
is a plugin behind a `slamko_core` contract, not a rewrite. User criterion
(verbatim): *"A mí me importa el resultado. Y que sea bien estable sin romperse"*
— result + stability over novelty.

This is the **single source of truth** (always loaded — you work from this repo
root). Each package has only a short `README.md` (orientation) + `docs/`
(validated detail). The authoritative plan: [`MASTER_PLAN.md`](MASTER_PLAN.md)
(v2, migrated 2026-06-12 from the adopted
[`docs/REBUILD_PROPOSAL_01.md`](docs/REBUILD_PROPOSAL_01.md), which keeps the full
research provenance — anchor-don't-weld, iSAM2-poses-only, raw-mag-not-BNO-fused,
GNSS init-then-re-anchor). Old own-VIO plan: `docs/archive/MASTER_PLAN_OWNVIO_01.md`.
Also: [`docs/DECOUPLING.md`](docs/DECOUPLING.md), [`docs/DOC_PROCESS.md`](docs/DOC_PROCESS.md).

> **Current focus (2026-07-09): cuVSLAM OPEN-SOURCE → 2nd PROVIDER SHIPPED + A/B + DEPLOY PLAN;
> NEXT = GAZEBO closed-loop (wheel-EKF + referee cinemático + Nav2 + velocity governor).**
> Cold-start → [`docs/PIPELINE_STATUS_01.md`](docs/PIPELINE_STATUS_01.md) **§0 (2026-07-09)** +
> [`docs/RESEARCH_CUVSLAM_OPENSOURCE_01.md`](docs/RESEARCH_CUVSLAM_OPENSOURCE_01.md) +
> [`docs/PLAN_ROBOT_DEPLOY_01.md`](docs/PLAN_ROBOT_DEPLOY_01.md) + memory `slamko-cuvslam-opensource`.
> cuVSLAM v16 is FULL open source (`~/coding/cuVSLAM_src`, fork `slamko/trusted-health` exposes
> per-frame pnp_health — inliers/residual/H-condition; gates `inliers<10||cond>1e6` validated on the
> wall bag: teleports = 1 inlier + H singular). Adapter shipped (`slamko_vio` `cuvslam_provider_node`
> → `/cuvslam/odometry`+`/cuvslam/health`, odometry-only, P-A PASS 0.0000 m). **Provider policy:
> cuVSLAM-Inertial = recommended for NEW runs (A/B: Suave 12.8cm/0.8%, Escaleras 49cm/1.7% — the IMU
> fixed the stereo-only 6.6% stairs under-scale); OKVIS = existing launches/battery default, offline
> baseline, fallback, AND the flash-bag provider (cuVSLAM degraded there: 147 teleports → 35 honest
> islands, no false weld).** Bench: `scripts/bench_cuvslam_provider.{py,sh}` (4-channel; cuVSLAM cov
> 30–350× overconfident → cov_scale 80). SIM = GAZEBO (Isaac starves the 8 GB GPU; PLAN_ISAACSIM_01
> shelved). GOTCHAS: IMU must be BUFFERED ≤ frame_t ("Timestamps are non-monotonic" else); ROS shell
> shadows the wheel's libcuvslam (RPATH/wrapper); duplicate-publisher zombies = duplicated stamps in
> provider.tum. (Prior focus below kept for provenance.)
>
> **Prior focus (2026-06-30): IMMORTAL GATES + DEPTH GEOMETRIC LOOP + DYNAMIC LOCAL COSTMAP +
> D455 CLEAN-MAP shipped (commit 220c130); then-NEXT = Isaac Sim (superseded → Gazebo).**
> Cold-start → [`docs/PIPELINE_STATUS_01.md`](docs/PIPELINE_STATUS_01.md)
> **§0b (2026-06-30)** + [`docs/PLAN_ISAACSIM_01.md`](docs/PLAN_ISAACSIM_01.md) + memories
> `slamko-isaacsim-next`, `slamko-immortal-seal-on-doubt`, `slamko-d455-clean-map`. Shipped: A live
> IMU referee (`imu_referee`, recall 0→0.67) + B HOLD (`hold_on_loss`) seal-on-doubt; depth geometric
> loop weld (`depth_loop_refine`, 7.2cm, point-to-SDF ICP on nvblox ESDF); DYNAMIC LOCAL costmap (2nd
> DECAYING nvblox mapper @45Hz per-frame@live, `local_dynamic` ON — static+dynamic split); Nav2 local
> timer; D455 fixes (range 5→3.5, speckle filter). Validated vs OKVIS-full-SLAM GT (ATE 7.5cm median).
> Open levers: nvblox 1/z² weight, depth pre-filter, push. (Prior focus below kept for provenance.)
>
> **Prior focus (2026-06-26): UNIVERSAL EVALUATOR + NEVER-JUMP GATE shipped.**
> Cold-start → [`docs/PIPELINE_STATUS_01.md`](docs/PIPELINE_STATUS_01.md) **§0c (2026-06-26)** +
> [`docs/EVAL_SYSTEM_01.md`](docs/EVAL_SYSTEM_01.md) + memory `slamko-universal-evaluator`. Built
> `scripts/slamko_eval.py` = the **provider-agnostic 7-channel ideology scorecard** (3 independent
> witnesses: IMU inertial · depth→SDF geometric · XFeat recognition) — judges the slamko LAYER, not
> the provider (untrusted by design). It MEASURED that cuVSLAM/OKVIS teleports are 100% caught
> (sealed in LOST→RECOVERED), then FOUND + FIXED a real **never-jump defect**: the live robot pose
> jumped 17.5 m/s with the provider (the `map→odom` slew bounds only the correction leg; the teleport
> rode `odom→base` raw) → **`gate_live_pose`** absorbs it into `T_gate_` (A/B: 0 jumps/2.19 m/s PASS,
> opt-in default OFF). Also `scripts/tsdf_slice.py` (TSDF floor-plan cut). The 45 fps "flash" bag
> volumetric map (D455 HW depth → nvblox TSDF) is coherent (605 kf, loop closed, path in free space).
> **GTSAM/iSAM2 pose-graph backend SHIPPED (MASTER_PLAN P-C′) and is now the LIVE DEFAULT**
> (`pose_graph_backend=isam2`, GTSAM auto-detected; `PoseGraphBackend::{Ceres,GtsamLM,GtsamISAM2}`):
> native GTSAM yaw factor, EuRoC ATE Ceres≈iSAM2 (19.1 mm), **6× faster live** (O(touched)), brutal-bag
> validated. GTSAM is ONLY the pose-graph solver — volumetric nvblox / provider / reloc unchanged.
> **The GLOBAL + LOCAL costmaps are SHIPPED** (2026-06-26): `~/volumetric_costmap` (global, whole TSDF
> slice, latched) + `~/local_costmap` (local, rolling 4 m window of the live nvblox slice). **NEXT =
> the Nav2 planner+controller** over the two costmaps, lifecycle gated on `localized` (the never-jump
> gate is the prerequisite, already default-ON). GOTCHAS: capture the GLOBAL costmap at END-of-run (it
> grows; mid-run = one room = "solo salón"); the flash-bag splitter chain is flaky on repeated runs;
> cuVSLAM 0-odometry
> "body_tf" blocker was a missing LD_LIBRARY_PATH (libnvblox) — FIXED; gate threshold =
> robot-max-speed+margin; tsdf_slice MUST cut at navigable height (else projection artefact looks like
> a wall-crossing); rosbags in `/tmp/rerunvenv` not system python; build with `-DSLAMKO_LOOP_WITH_GTSAM`
> auto-on when GTSAM present.
>
> **Prior focus (2026-06-20): COHERENT CROSS-SESSION FUSION shipped (A+B+E).** Cold-start →
> [`docs/RESEARCH_LIFELONG_FUSION_01.md`](docs/RESEARCH_LIFELONG_FUSION_01.md) +
> [`docs/PLAN_BRUTAL_RUNS_VIZ_01.md`](docs/PLAN_BRUTAL_RUNS_VIZ_01.md) + memory
> `slamko-lifelong-fusion-ABE`. The revisit "doubling" = rigid-SE3 re-base can't absorb
> path-growing drift (5-agent research). FIX: **A** cross-session = weighted PRIOR FACTOR in the
> pose-graph (`PoseGraph::addPriorFactor`), not a rigid re-base; **B** stiff-chain (soft only on a
> TRUE odom stale-gap, fixed the all-soft img-miss bug → 15 stiff/0 soft/2 hard); **E** proximity
> detection (`XFeatRelocalizer::relocalizeNear` by anchor-distance, VPR-independent → fills the
> recall-dead gap). Validated A-vs-A+E @rate0.5: matches 15→45, p90 1.41→1.01 m (−28%), certainty
> 24→31%. **HONEST DANGLING model:** overlap→connect+align, no-overlap→hang (never a fake-coherent
> double). Brutal bags recorded (`/mnt/data/bno_ab/BRUTAL_BAGS.md`). **GOTCHA: OKVIS GPU-contention
> nondeterminism → run VPR-on rate≤0.5 + check provider.tum y-span before any ATE.**
> **LIVE VIZ shipped (2026-06-20, 633a06d):** `slamko_ros` `VizSink` = live Rerun (rerun.io) viewer —
> window A image+XFeat-keypoints+HUD, window B 3D map building live + frustums + edges BY TYPE
> (chain/soft/loop/x-prior/proximity-candidate), Atlas-coherent (session Transform3D). NO-OP unless
> `-DSLAMKO_WITH_RERUN` (FetchContents SDK 0.33); `connect_grpc` to a SEPARATE viewer (never spawn(),
> GPU isolation) or `.rrd` offline-rewindable. `viz_selftest`→345 KB .rrd; both builds green. Plus
> **3-tier candidate→soft→weld** for the VPR-independent proximity path (reversible
> `proximity_three_tier`). "Soft edges on another plane" → per-class COLOUR+entity-LAYER, not z-offset
> (research). Next: VALIDATE live brutal-revisit capture + 3-tier A/B (no-regress) · #12
> loss-edge magnitude · C suppress duplicate-submap sealing (cross-session bounding) · D cull backstop.
> **Prior milestone (2026-06-19, IMMORTALITY CORE):** [`docs/PIPELINE_STATUS_01.md`](docs/PIPELINE_STATUS_01.md) §0
> + memory `slamko-immortality-push`. ~26 commits: **map BOUNDED-by-AREA**
> (ORB data-association: per-landmark cross-submap cull + drift-tolerant 0.15 m voxel +
> viewpoint-aware → revisiting plateaus, landmark growth +100%→+3%/visit); **R0
> "never ingest garbage" gates** (seal-quality + DR-informed bar degraded submaps as reloc
> targets); **I2 never-false-merge VALIDATED** (162 welds, 0 teleports, `scripts/audit_i2.py`);
> compass instrument; and the big **RECALL REFRAME** — the cosine "cliff" is VIEWPOINT
> coverage, NOT descriptor quality (same-heading revisits match; opposite-facing is no-overlap,
> unmatchable by any model/dense-matcher) → CANCELLED the SALAD-swap + LoFTR C++ builds after
> offline A/B (`scripts/vpr_ab_casa.py`, venv `/tmp/vprvenv`).
> Scorecard: never-lost ✅ map-bounded ✅ never-garbage ✅ never-false-merge ✅ recall ✅.
> Remaining = scope-expansion: GPS/compass-yaw 🟡 · out-of-core map 🔴 · real-robot stress 🟡.
> **NEXT TASK (user-chosen): RECORD NEW BRUTAL STRESS BAGS** (IMU-blackout, wall-pointing,
> a DIFFERENT place, multi-DIRECTION revisit, rough-terrain/flip) — the extreme modes can't be
> stressed without data (c2/c3 dirs are OUTPUT folders, not bags; only CASA1_* are valid).
> **Method that paid off: de-risk by MEASURING OFFLINE before building.** Older plan
> [`docs/PLAN_ROBUSTNESS_01.md`](docs/PLAN_ROBUSTNESS_01.md) (R0 gates now partly shipped).
> **Hard gotchas: bno_ab bags are 640×480 (config rsD455_map_odom, NOT odom848) with DOUBLED
> camera-IMU accel (use ~/coding/BNO055/ab launch); evaluate graph.tum with Umeyama scale, never
> the closure number; NEVER pip into system Python (PEP668; ROS depends on it) — use the venv.**

## Orientation (cold start — human or LLM)

**Reading order:** this file → [`MASTER_PLAN.md`](MASTER_PLAN.md) →
[`docs/SYSTEM.md`](docs/SYSTEM.md) (**the cold-start map**: status-at-a-glance table +
how the modules interact + where every note lives) →
[`docs/DECOUPLING.md`](docs/DECOUPLING.md) (the contracts) → the package's
`README.md` → its `docs/STATUS.md` (what's validated + the numbers) → its
`docs/PLAN_<phase>.md` (the active plan).

**Where each kind of knowledge lives:** contracts → `slamko_core` headers +
`DECOUPLING.md` · how-it-works + the math → the **inline header "why" blocks** +
`SYSTEM.md` · what's validated / the numbers / what was tried-and-reverted → each
package's `docs/STATUS.md` · the rules → this file.

**Doc discipline (lean — don't contaminate):** the **inline header block** (why this
exists + how it fits + the load-bearing decision) and the **per-tuning rationale**
(reason + what was tried) are the *primary* doc surface — they can't drift from the
code. Standalone prose is reserved for the **one** system map (`SYSTEM.md`) + the
per-package `STATUS.md` logs. No parallel/overlapping prose docs.

## This is ONE git repo, worked from the root

`slamko/` is a single git monorepo; you edit all packages from here and build
them as a colcon workspace (like GLIM = glim+glim_ros2+glim_ext, OKVIS = one repo
of ~13 libs). Don't split packages into separate repos.

**Remote:** https://github.com/maikelborys/slamko — commit locally as you go;
**push when a milestone lands** (a phase closes, a package validated green).

## Claude Code's role here: ENGINEER + ORCHESTRATOR (self-sufficient system)

You (Claude Code) operate on slamko as both the implementing **engineer** AND the
session **orchestrator** — the goal is a **self-sufficient SLAM system that watches
its own runs and improves itself.** The operating loop:

> **When SLAM runs** (bag replay / sim / real robot): **monitor everything** —
> logs, topics, `health`/localization status, ATE & un-aligned divergence, FPS,
> graph conditioning. **Diagnose by root cause** (degeneracy? init? sensor
> dropout? bad loop closure?), **make the fix**, **re-run**, **validate** —
> iterate until green. Maintain the docs (STATUS + validated stamps) as you go.

Principles: spawn research / parallel agents for breadth; **benchmark-driven**
(every change measured, regress ≥5% → revert); **diagnose before retrying**;
surface trade-offs in one sentence; leave the trail (commands, safety caveats,
where to look) the next session needs. This mirrors the `~/coding/CLAUDE.md`
developer+orchestrator role, scoped to slamko.

**Zombie/orphan check (HARD — learned the hard way).** Before AND after every
SLAM run (`scripts/bench_ate.sh` or a direct `ros2 launch`): verify no stale
`slamko_vio_node` / `euroc_player` are alive (`pgrep -af 'slamko_vio_node|euroc_player'`).
A leftover from a crashed/killed prior run is a **duplicate publisher** on
`/slamko_vio/odometry` + `/tf` that **silently corrupts the next run** — the bag
recorder locks onto the wrong node (empty bag), the GPU is contended, the launch
dies early (SIGKILL -9). `ros2 launch` children routinely escape the process group,
so `kill -- -$PGID` alone is not enough — reap by name (`pkill -KILL -f`) and
re-verify. `bench_ate.sh` now does this pre-flight + on teardown. **Run benches
SERIALLY** — concurrent background runs reap each other (global `pkill` races).
These process names are bench-owned, never the user's stack, so reaping them is safe.

## Module map (6 packages — pluggable, decoupled)

Hard rule: **a package depends only on `slamko_core` contracts, never on another
package's internals.** That is what makes them swappable and gives the
DigiForest/VILENS decoupling.

| Package | Role | Depends on |
|---|---|---|
| **slamko_core** | Contracts (`Factor`, `SensorFrontend`, `FactorGraphBackend`, `Relocalizer`) + types (SE3/manifold, `SubMap`, `EstimationFrame`, `NodeKey`, `RobustKernel`) + **cross-cutting infra**: time-sync/buffering (TimeKeeper, trajectory buffer, thread-safe queues), config + per-platform presets, structured logging, map serialization schema, frame conventions, **health-signal interfaces**. Thin on algorithms, rich on infra (GLIM `common/`+`util/` model). | — |
| **slamko_vio** | **Thin provider adapters** wrapping external odometry (OKVIS2-X first; klt_vo/Basalt/cuVSLAM later) behind the `slamko_core` provider contract. The legacy own-VIO (XFeat/KLT/IMU, seeded by klt_vo) is **deprecated — slated for deletion** (klt_vo HEAD is strictly better; the fork has zero unique value). | core |
| **slamko_fusion** | The **loose chain-pose-graph fixed-lag fuser**: relative provider edges + global constraints (reloc/GNSS), covariance-weighted. Global backend = **iSAM2 over poses/anchors ONLY — no landmarks ever** (metric smoothing stays inside the provider). Emits health probes. | core |
| **slamko_loop** | **Atlas multi-map** + lifelong map mgmt + **relocalization** (EigenPlaces retrieval → XFeat+LighterGlue verify; libtorch isolated as an optional build target) + the **never-lost supervisor / health POLICY** (state machine, watchdogs; seal→branch→relocalize→**reversible gated anchor** — anchor-don't-weld) + GNSS anchoring + defensive numerics (catch-damp-rebuild). | core |
| **slamko_msgs** | ROS 2 interface defs (map-server API, correspondences, status/lifecycle). | — |
| **slamko_ros** | ROS 2 integration (composition root — the only package that knows all others): nodes, the `map→odom→base` bridge, launch, **and visualization** (rviz panels; offline Plotly lives in `scripts/`). | all + msgs |

`scripts/` holds the benchmark harness (EuRoC ATE + un-aligned divergence, FPS
A/B, feature compare-all) and offline viz — **not a package** (OKVIS `eval/`,
klt_vo `scripts/` pattern).

**Health** is designed in from day 1, not retrofitted: signal *probes* live in
`core` (interfaces) and are emitted by `vio`/`fusion`; the decision *policy* lives
in `slamko_loop` (it IS the never-lost supervisor). Split into its own package
only if that policy outgrows loop.

**Deferred packages** (split out when their phase's code crosses real boundaries —
not pre-drawn): **slamko_mapping** (promoted — the lifelong tiled map-server:
out-of-core store, summarization, versioning, georeferencing; split during P-C),
**slamko_sensors** (LiDAR/GPS/wheel/mag frontends; split at the 2nd sensor),
**slamko_semantic** (P-F: semantic map layers). Until then `SubMap` lives in
`core`, the global graph in `loop`.

## Dependency graph
```
slamko_core ◄── vio, fusion, loop          slamko_msgs ◄── ros
slamko_ros  ── integrates everything (composition root; only it knows all packages)
scripts/    ── standalone harness (bench + offline viz)
```

## Documentation discipline (ENFORCED — see [`docs/DOC_PROCESS.md`](docs/DOC_PROCESS.md))
> implement → run the package's tests / bench → **on green**, update its
> `docs/STATUS.md` (dated entry + numbers) and bump the
> `<!-- validated: <commit> <date> · tests: <result> -->` stamp on any design doc
> you changed → **commit code + docs together.** Never commit code without its
> STATUS entry. `scripts/check_doc_freshness.sh` flags packages whose code moved
> ahead of their docs.

## Hard rules
1. Apache-2.0 / BSD-3 only. GTSAM/Ceres (BSD-3), XFeat/LiftFeat/LightGlue (Apache),
   DBoW2 (BSD) OK. **No ORB-SLAM3 GPL code shipped** — reuse the approach, not code.
2. Packages depend only on `slamko_core` contracts. No cross-package internal deps.
3. Degradation = covariance inflation, never an `if(sensor_ok)` branch.
4. The global graph is disposable; the fast odometry never depends on it.
5. A never-lost system reports BOTH Sim3-aligned ATE/RPE AND an un-aligned
   divergence/health metric (Sim3 alignment hides catastrophic divergence).
