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
**Status:** **P0 (Milestone B) shipped** — klt_vo ported, decomposed into a
ROS-agnostic `VioPipeline` + thin node, detector swappable behind
`slamko_core::FeatureSource` (`feature_source:=shitomasi|xfeat`), IMU-fused,
short-gap dead-reckoning, XFeat descriptors attached to landmarks (reloc map).
Equal-coverage MH_01: Shi-Tomasi 0.078 m @ ~214 fps · **XFeat-TRT 0.049 m @ ~93 fps**.
Next: **P1 `slamko_fusion`** (GTSAM + marginalization). See
[`docs/PLAN_P0_vio.md`](docs/PLAN_P0_vio.md) + [`docs/STATUS.md`](docs/STATUS.md).

**Build + bench:**
```bash
cd ~/coding/slamko && colcon build --packages-select slamko_vio --cmake-args -DCMAKE_BUILD_TYPE=Release
bash scripts/bench_ate.sh MH_01_easy      # EuRoC ATE (needs ~/datasets/euroc + euroc_publisher)
```

**Starting cold here?** Read the 3 hub docs + this + `klt_vo/docs/13,14`, then
plan mode → `docs/PLAN_P0_vio.md`.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
