# slamko_sensors — sensor frontend plugins

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** the "register a sensor, not a rewrite" modalities beyond VIO. Each is a
`SensorFrontend` emitting `Factor`s: **LiDAR** (plane/line landmarks, point-to-plane),
**GPS** (global-position prior + ENU datum node, in the slow graph, DCS-robust),
**wheel/leg odometry** (preintegrated + online velocity-bias for slip), **ZUPT**
(zero-velocity). Degradation = covariance inflation, never a branch.

**Implements:** `SensorFrontend` (one per modality). **Depends on:** slamko_core.
**Status:** planned. **Phase P5** (order: wheel+ZUPT → LiDAR → GPS).

**Starting cold here?** Read the 3 hub docs + this, then plan mode →
`docs/PLAN_P5_sensors.md`. Factor math (residual/cov/kernel per modality) is in
`../MASTER_PLAN.md` §4 and the klt_vo `docs/14` factor catalog.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
