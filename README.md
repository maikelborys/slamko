# slamko

**A modular, pluggable SLAM framework whose #1 job is to never get lost.**
Strong like ORB-SLAM3 / OKVIS2-X / VILENS, but built from a small core where **every
sensor and capability is a plugin behind a contract — not a rewrite.** Fast, robust,
super-recoverable, and deliberately **simple and stable**.

> **Status (2026-05-27):** `slamko_core` shipped (header-only spine, 25/25 tests).
> `slamko_vio` runs a CUDA stereo-inertial tracker with a **swappable feature
> front-end** — Shi-Tomasi (0.078 m ATE @ ~214 fps) or **XFeat-TensorRT**
> (**0.049 m @ ~93 fps**, learned 64-d descriptors) on EuRoC MH_01 (equal-coverage,
> IMU-fused, short-gap dead-reckoning). Full state: [`docs/SYSTEM.md`](docs/SYSTEM.md).

## The idea

SLAM systems usually fail in one of two ways: they **get lost** (lose tracking and
never recover), or they're a **monolith** (adding a sensor or swapping a feature
extractor means surgery on the whole estimator). slamko is built to avoid both.

**1. Never lost — three safety nets at three timescales.**

| When vision degrades | What catches it | Where |
|---|---|---|
| blur / occlusion (ms–0.5 s) | IMU + constant-velocity **dead-reckoning** | `slamko_vio` ✅ |
| medium loss (seconds) | **archive-don't-discard**: seal the submap, branch a fresh one, keep emitting odometry | `slamko_loop` (P2) |
| kidnap / long loss | **relocalize** by appearance + weld with one pose constraint | `slamko_loop` (P2/P3) |

Loss is detected by the **odometry stale-gap**, not a covariance spike — a blackout
*pauses* odometry, it doesn't inflate uncertainty (the OKVIS2-X lesson).

**2. Pluggable — everything is a `Factor` behind a contract.** A sensor frontend
turns a measurement into factors `{keys, residual, √information, robust_kernel}`; a
backend (GTSAM / Ceres) owns the solve. Adding a sensor = *register a frontend*.
Swapping the feature extractor (Shi-Tomasi ↔ XFeat ↔ LiftFeat-m1) = *register a
`FeatureSource`* — no other module changes. Degradation is **covariance inflation,
never an `if(sensor_ok)` branch**.

**3. Two graphs, never coupled.** A fast bounded **local smoother** (real-time pose,
owns `odom→base`) runs independently of a slow async **global graph** (loop closure,
recovery, owns `map→odom`). The global graph is *disposable* — it can be damped or
rebuilt from the still-trusted odometry without ever stalling the tracker.

Five principles, one per reference system: robustness-by-deletion (RKO-LIO),
covariance-not-branches (VILENS), disposable-global-graph (GLIM), decouple-at-a-map-
contract (DigiForest), recovery-as-state-machine (ORB-SLAM3). See
[`MASTER_PLAN.md`](MASTER_PLAN.md).

## Architecture

```
sensors ─► slamko_vio ──► EstimationFrame ─► slamko_fusion ──► SubMap ─► slamko_loop
 (D455/    FeatureSource   (T_WB, v, bias)   fixed-lag         (KF poses,  place-rec +
  EuRoC)   →Tracker→PnP                      smoother +         landmarks,  never-lost
           +IMU +DR                          marginalization    descriptors) supervisor
           owns odom→base                                                    owns map→odom
                                  └────────────► slamko_ros (composition root, bridge, viz)
```

Three tiers, two graphs, three TF owners — full data-flow + threading model +
glossary in [`docs/SYSTEM.md`](docs/SYSTEM.md).

## Packages (lean: 6, + 3 deferred)

| Package | Role |
|---|---|
| **`slamko_core`** | contracts (`Factor`/`SensorFrontend`/`FactorGraphBackend`/`FeatureSource`/`Relocalizer`) + types + SE3 + health signals. Header-only, Eigen-only. ✅ |
| **`slamko_vio`** | the fast visual-inertial tracker: swappable feature front-end (Shi-Tomasi / XFeat-TRT / LiftFeat-m1) + KLT + stereo + PnP + IMU + dead-reckoning. 🟢 |
| **`slamko_fusion`** | GTSAM iSAM2 fixed-lag smoother + marginalization (Schur + FEJ). _planned (P1)_ |
| **`slamko_loop`** | global graph + loop closure + relocalization + the never-lost supervisor. _planned (P2)_ |
| **`slamko_msgs`** | ROS 2 interface defs (map-server API). _planned (P4)_ |
| **`slamko_ros`** | ROS 2 integration: nodes, `map→odom→base` bridge, visualization. _planned_ |

Deferred until their phase: `slamko_mapping` (P4), `slamko_sensors` (P5),
`slamko_semantic` (P6).

## Build & run

```bash
# Colcon workspace (needs ROS 2 jazzy, CUDA, Ceres, OpenCV; TensorRT for XFeat).
cd ~/coding/slamko
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# EuRoC bench (needs ~/datasets/euroc + the euroc_publisher workspace):
bash scripts/bench_ate.sh MH_01_easy                         # Shi-Tomasi baseline
ros2 launch slamko_vio vio_euroc.launch.py seq:=<MH_01> feature_source:=xfeat  # XFeat-TRT
```

## Docs

- [`docs/SYSTEM.md`](docs/SYSTEM.md) — **start here**: the system map + status-at-a-glance + where every note lives.
- [`MASTER_PLAN.md`](MASTER_PLAN.md) — vision, locked decisions, phased roadmap.
- [`docs/DECOUPLING.md`](docs/DECOUPLING.md) — the `slamko_core` contracts.
- [`CLAUDE.md`](CLAUDE.md) — orientation + working rules.

## License

Apache-2.0 / BSD-3 dependencies only (GTSAM, Ceres, XFeat/LiftFeat, DBoW2). No GPL code shipped.
