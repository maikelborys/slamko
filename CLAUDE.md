# slamko — central orientation & instructions (the one authoritative doc)

**slamko** is a modular, pluggable SLAM framework. Top priority: **never gets
lost, never fails, super-recoverable — while staying SIMPLE and STABLE.** Strong
like ORB-SLAM3 / OKVIS2-X / VILENS, but built from a small core where **every
sensor / capability is a plugin behind a `slamko_core` contract, not a rewrite.**

This is the **single source of truth** (always loaded — you work from this repo
root). Each package has only a short `README.md` (orientation) + `docs/`
(validated detail). Details: [`MASTER_PLAN.md`](MASTER_PLAN.md),
[`docs/DECOUPLING.md`](docs/DECOUPLING.md), [`docs/DOC_PROCESS.md`](docs/DOC_PROCESS.md).

## Orientation (cold start — human or LLM)

**Reading order:** this file → [`MASTER_PLAN.md`](MASTER_PLAN.md) →
[`docs/SYSTEM.md`](docs/SYSTEM.md) (how the modules interact) →
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

## Module map (6 packages — pluggable, decoupled)

Hard rule: **a package depends only on `slamko_core` contracts, never on another
package's internals.** That is what makes them swappable and gives the
DigiForest/VILENS decoupling.

| Package | Role | Depends on |
|---|---|---|
| **slamko_core** | Contracts (`Factor`, `SensorFrontend`, `FactorGraphBackend`, `Relocalizer`) + types (SE3/manifold, `SubMap`, `EstimationFrame`, `NodeKey`, `RobustKernel`) + **cross-cutting infra**: time-sync/buffering (TimeKeeper, trajectory buffer, thread-safe queues), config + per-platform presets, structured logging, map serialization schema, frame conventions, **health-signal interfaces**. Thin on algorithms, rich on infra (GLIM `common/`+`util/` model). | — |
| **slamko_vio** | Visual-inertial odometry S1 (XFeat / LiftFeat-m1 / Shi-Tomasi + KLT + IMU + dead-reckoning). **Re-entrant initialization** inside (NOT a one-shot latch — that's why OKVIS can't self-recover). Seeded by `~/coding/klt_vo`. | core |
| **slamko_fusion** | Heterogeneous fixed-lag smoother: **GTSAM iSAM2** + **marginalization** (Schur+FEJ) + `FactorGraphBackend` adapters (gtsam default, ceres). The VILENS heart. Emits health probes (degeneracy eigenvalues, marginal covariance). | core |
| **slamko_loop** | Global graph + loop-closure-as-factor + **relocalization** (LiftFeat-m1; libtorch isolated as an optional build target, not a package) + the **never-lost supervisor / health POLICY** (Good/Marginal/Lost state machine, watchdogs, recovery triggers; seal→branch→relocalize→merge, decoupled) + defensive numerics (catch-damp-rebuild). | core |
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
not pre-drawn): **slamko_mapping** (P4: submap persistence + map-server contract),
**slamko_sensors** (P5: LiDAR/GPS/wheel frontends; split at the 2nd sensor / the
LiDAR-pulls-PCL fault line), **slamko_semantic** (P6: object factors). Until then
`SubMap` lives in `core`, the global graph in `loop`.

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
