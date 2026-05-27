# slamko_vio — visual-inertial odometry (the fast S1)

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** the fast local tracker. Feature front-end (**Shi-Tomasi / XFeat /
LiftFeat-m1**, swappable) + **KLT** flow + stereo + PnP + IMU preintegration +
**dead-reckoning on tracking loss** (const-vel+gyro) + **re-entrant VI
initialization** (gravity/bias — NOT a one-shot latch; re-triggerable by the
health supervisor, the gap that stops OKVIS self-recovering). Emits factors; owns `odom→base`.

**Seeded by `~/coding/klt_vo`** (functional CUDA stereo VIO: MH_01 ATE 0.054 m
VIO @ ~240 fps; gravity-locked init; short-gap IMU dead-reckoning). Migrate here
when P0 closes. Detailed prior docs: `~/coding/klt_vo/docs/13` + `docs/14`.

**Implements:** `SensorFrontend` (+ IMU & reprojection `Factor`s). **Uses:**
`CeresBackend` as the S1 inner loop. **Depends on:** slamko_core (+ CUDA, Ceres).
**Status:** seeded (klt_vo). **Phase P0** — feature compare-all ATE (ShiTomasi vs
XFeat vs LiftFeat-m1) + full-suite validation.

**Starting cold here?** Read the 3 hub docs + this + `klt_vo/docs/13,14`, then
plan mode → `docs/PLAN_P0_vio.md`.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
