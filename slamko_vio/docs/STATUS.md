# slamko_vio — Status log

Living, dated progress + numbers log. Append on every validated change
([`../../docs/DOC_PROCESS.md`](../../docs/DOC_PROCESS.md)). Plan:
[`PLAN_P0_vio.md`](PLAN_P0_vio.md).

## 2026-05-27 — B1: faithful port of klt_vo → slamko_vio (baseline guard) ✅

**What:** ported `klt_vo_core` (Shi-Tomasi, KLT, stereo matcher, triangulator,
PnP/CUDA, pose_estimator, Ceres LocalBA, IMU preintegration) + the node + launch
into `slamko_vio`, blanket-renamed `klt_vo`→`slamko_vio` (verbatim logic, flat
layout). Self-contained — no `slamko_core` dependency yet (that enters at B1b).
CMake mirrors klt_vo (CUDA sm_89, Ceres, OpenCV 4, ROS jazzy); builds standalone in
the slamko colcon workspace (~33 s). Bench harness `scripts/bench_ate.sh` ported +
adapted to slamko's install + `/slamko_vio/odometry`.

**Tests:** 19 ported core gtests pass (shitomasi, klt, stereo, imu_preint, p3p,
motion_ba) — `colcon test --packages-select slamko_vio`.

**GATE — same-machine A/B on EuRoC MH_01_easy (rate 1.0, IMU-on, launch defaults),
Sim3-aligned ATE via evo_ape:**

| build | RMSE (m) | mean | median | max | std | fps |
|---|---|---|---|---|---|---|
| klt_vo (reference, ros2_ws) | 0.0902 | 0.0846 | 0.0733 | 0.187 | 0.031 | ~210 |
| **slamko_vio (ported)** | **0.0688** | 0.0630 | 0.0563 | 0.144 | 0.028 | ~210 |

**Verdict:** PASS. Identical code → both within klt_vo's documented MH_01 band
(0.06–0.09 m, README); slamko_vio on the better side this run. The ~0.02 m gap is
run-to-run variance (RANSAC stochasticity + GPU-timing frame-selection), not a
regression. ~4.7 ms/frame (klt 0.11 / stereo 0.38 / pnp ~3.0). 3419 poses, clean
end-of-sequence shutdown.

**Note:** the `docs/13` headline 0.054 m is a best-case tuned figure; the
launch-default config the bench uses lands at 0.06–0.09 for both builds — so the
guard compares like-for-like (default config, both builds, same machine).

**Next — B1b:** decompose the monolithic node into a ROS-agnostic `VioPipeline` +
thin `vio_node`, and lift detect→`ShiTomasiSource : slamko_core::FeatureSource` /
KLT→`KltFlowTracker : slamko_core::FeatureTracker`. Gate: full-suite numbers
unchanged (±noise).
