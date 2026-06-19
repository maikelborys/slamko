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

> **Current focus (2026-06-19): the IMMORTALITY CORE is BUILT + validated on casa1.
> Cold-start → [`docs/PIPELINE_STATUS_01.md`](docs/PIPELINE_STATUS_01.md) §0** (refreshed)
> + memory `slamko-immortality-push`. Done today (~26 commits): **map BOUNDED-by-AREA**
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
