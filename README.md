# slamko

A modular, pluggable SLAM framework. Top priority: **never gets lost, never
fails, super-recoverable — while staying SIMPLE and STABLE.** Strong like
ORB-SLAM3 / OKVIS2-X / VILENS, but built from a small core where **every sensor /
capability is a plugin behind a `slamko_core` contract, not a rewrite.**

> Status: design + scaffold. `slamko_vio` is seeded by a working CUDA stereo VIO
> (MH_01 EuRoC ATE 0.054 m @ ~240 fps). Built phase by phase — see the roadmap.

## Architecture — 3 tiers, 2 graphs

```
Tier 1 · FRONTENDS (plugins)   vio (XFeat/LiftFeat/KLT+IMU) · sensors · semantic
            │ each emits Factors (+ covariance)
            ▼
Tier 2 · LOCAL FUSION          fixed-lag smoother (GTSAM iSAM2) + marginalization
            │ fast · owns odom→base · never blocks
            ▼
Tier 3 · GLOBAL                loop-closure-as-factor · never-lost multi-map
            │ async · disposable · owns map→odom    supervisor · relocalization
```

Three robustness nets at three timescales: IMU/const-vel dead-reckoning (short
gaps) → archive-don't-discard multi-map (medium loss) → relocalization (kidnap).

## Packages (start lean: 6)

| Package | Role |
|---|---|
| `slamko_core` | contracts (`Factor`/`SensorFrontend`/`FactorGraphBackend`) + types + cross-cutting infra (time-sync, config, logging, serialization, health signals) |
| `slamko_vio` | visual-inertial odometry (the fast S1; XFeat/LiftFeat-m1/KLT + IMU + dead-reckoning) |
| `slamko_fusion` | GTSAM iSAM2 fixed-lag smoother + marginalization |
| `slamko_loop` | global graph + loop closure + relocalization + never-lost supervisor + health policy |
| `slamko_msgs` | ROS 2 interface definitions |
| `slamko_ros` | ROS 2 integration (composition root) + bridge + visualization |

Deferred until their phase: `slamko_mapping` (P4), `slamko_sensors` (P5),
`slamko_semantic` (P6).

## Docs

- [`CLAUDE.md`](CLAUDE.md) — the authoritative orientation + instructions (system map, roles, rules).
- [`MASTER_PLAN.md`](MASTER_PLAN.md) — vision, locked decisions, phased roadmap.
- [`docs/DECOUPLING.md`](docs/DECOUPLING.md) — the `slamko_core` contracts.
- [`docs/DOC_PROCESS.md`](docs/DOC_PROCESS.md) — how docs stay true to code + tests.

## License
Apache-2.0 / BSD-3 dependencies only (GTSAM, Ceres, XFeat/LiftFeat, DBoW2).
