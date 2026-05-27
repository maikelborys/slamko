# slamko — orientation hub (root CLAUDE.md)

**slamko** is a modular, pluggable SLAM framework. Top priority: **never gets
lost, never fails, super-recoverable — while staying SIMPLE and STABLE.** Strong
like ORB-SLAM3 / OKVIS2-X / VILENS, but built from a small core where **every
sensor / capability is a plugin behind a `slamko_core` contract, not a rewrite.**

This file is the **hub**: every module's `CLAUDE.md` links back here so any
session — even one started cold inside a single module folder — knows the whole
system and its place in it. **Keep this file short**; details live in
[`MASTER_PLAN.md`](MASTER_PLAN.md), per-module `CLAUDE.md`, and `docs/`.

## Starting a session cold inside a module?

1. Read this file (the system map) + [`MASTER_PLAN.md`](MASTER_PLAN.md) (the why/what).
2. Read that module's `CLAUDE.md` (its role, contracts, status).
3. Read [`docs/DECOUPLING.md`](docs/DECOUPLING.md) (the contracts it must honor).
4. Then enter plan mode and produce the module's detailed plan.

## Module map

Each `slamko_*` is an **independent package**. The hard rule: **a module depends
only on `slamko_core` contracts, never on another module's internals.** That is
what makes them pluggable (swappable `.so` behind an interface) and what gives
the DigiForest/VILENS-style decoupling.

| Module | Role | Depends on | Status |
|---|---|---|---|
| **slamko_core** | Contracts + common types: `Factor`, `SensorFrontend`, `FactorGraphBackend`, `SubMap`, `EstimationFrame`, SE3/manifold, `NodeKey`, `RobustKernel`. Zero heavy deps. The spine. | — | planned |
| **slamko_vio** | Visual-inertial odometry (XFeat / LiftFeat-m1 / Shi-Tomasi + KLT + IMU + dead-reckoning). The fast S1. **Seeded by `~/coding/klt_vo`.** | core | seeded (klt_vo) |
| **slamko_fusion** | Heterogeneous fixed-lag smoother (**GTSAM iSAM2** + marginalization); pluggable solver adapters (gtsam default, ceres). The VILENS heart. | core | planned |
| **slamko_mapping** | Submaps, sparse/dense map representation, persistence, **map-server contract**. | core | planned |
| **slamko_loop** | Global: place recognition, **loop-closure-as-factor**, **never-lost multi-map supervisor** (Atlas seal→branch→relocalize→merge), defensive numerics. | core, mapping, reloc | planned |
| **slamko_reloc** | Relocalizer plugins (**LiftFeat-m1 default**, XFeat fallback, DBoW). | core | planned |
| **slamko_semantic** | Semantic layer: detection/segmentation, **object-level factors** (quadric/cuboid), semantic map layers, semantic relocalization. | core, mapping | planned |
| **slamko_sensors** | Sensor frontend plugins: LiDAR (plane/line), GPS, wheel/leg odom, ZUPT. Each emits factors via core. | core | planned |
| **slamko_msgs** | ROS 2 interface defs (map-server API, correspondences, status/lifecycle). | — | planned |
| **slamko_ros** | ROS 2 integration: nodes, the bridge (`map→odom→base`, Nav2 contract), launch. | all + msgs | planned |
| **slamko_bench** | Benchmark harness (EuRoC ATE, FPS A/B), the feature compare-all matrix. | — | planned |
| **slamko_viz** | Visualization (Plotly traj/ATE/map, RViz panels). | — | planned |

## Dependency graph (the decoupling)

```
slamko_core  ◄── vio, fusion, mapping, reloc, sensors, semantic
                 loop ──► mapping, reloc        semantic ──► mapping
slamko_msgs  ◄── ros
slamko_ros   ── integrates everything (the only place that knows all modules)
bench, viz   ── standalone tools
```

## Two-graph architecture (S1/S2)

Fast local **fusion** (Tier 2, real-time, owns `odom→base`, never blocks) +
slow async **global** (Tier 3 loop/mapping, owns `map→odom`, disposable &
rebuildable from odometry). Sensors (Tier 1) only emit factors. See MASTER_PLAN §1.

## Documentation discipline (ENFORCED — read [`docs/DOC_PROCESS.md`](docs/DOC_PROCESS.md))

**Docs must stay true to code + tests.** The rule every session/agent follows:

> implement → run the module's tests → **on green**, update that module's
> `docs/STATUS.md` (dated entry + the test/ATE numbers) and bump the
> `<!-- validated: <commit> <date> · tests: <result> -->` stamp on any design doc
> you changed → **commit code + docs together.** Never commit code without its
> STATUS entry.

`scripts/check_doc_freshness.sh` flags modules whose source moved ahead of their
docs (run in CI / pre-commit). This is how comprehensive docs get produced on
passing tests and corrected when commits change things.

## Hard rules
1. Apache-2.0 / BSD-3 only. GTSAM (BSD-3), Ceres (BSD-3), XFeat/LiftFeat/LightGlue
   (Apache), DBoW2 (BSD) all OK. **No ORB-SLAM3 GPL code shipped** — reuse the
   approach, not the code.
2. Modules depend only on `slamko_core` contracts. No cross-module internal deps.
3. Degradation = covariance inflation, never an `if(sensor_ok)` branch.
4. The global graph is disposable; the fast odometry never depends on it.
