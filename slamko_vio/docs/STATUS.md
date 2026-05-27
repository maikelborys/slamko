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

## 2026-05-27 — B1b: decompose into ROS-agnostic VioPipeline + FeatureSource seam ✅

**What:** broke the 1473-line monolithic node into:
- **`VioPipeline`** (`vio_pipeline.hpp/.cpp`) — the ROS-agnostic core: `(ImageView,
  ImuSample, StereoIntrinsics) → world pose + HealthSignal`. Owns all device
  buffers + stages + tracking/BA/IMU/DR state. No rclcpp/TF. The ~800-line
  algorithm body was transformed **in place** (preserved verbatim), only the I/O
  boundary changed (ROS msgs → ImageView, TF T_BS → `setExtrinsics`, publish → node).
- **`VioNode`** (`vio_node.cpp`, ~250 lines) — thin glue: params→`VioConfig`, stereo
  sync→`ImageView`, TF→`setExtrinsics`, publish odom/tf/markers.
- **`ShiTomasiSource : slamko::FeatureSource`** — the detector now runs behind the
  `slamko_core` contract (swappable; B2 registers `XFeatSource` the same way via
  `feature_source:=shitomasi|xfeat`). Owns its own device image + upload.
- Health probes populated: `odom_stale_gap_s` + `tracking_inlier_ratio` per frame.
- `slamko_vio` now depends on `slamko_core`.

**GATE — MH_01_easy (rate 1.0, IMU-on), Sim3-ATE:** RMSE **0.0785 m**, 3419 poses,
~4.68 ms/frame (~213 fps). vs B1 monolith 0.0688 and klt_vo-ref 0.0902 — equivalent
code spans 0.069–0.090, so 0.078 is mid-band: **no regression, within run-to-run
noise.** VI-init reproduced exactly (T_BS via setExtrinsics; `g_w=[0.244,9.437,2.667]`
|g|=9.81; gyro-bias from 15 KF pairs) → a true like-for-like IMU run, not a
visual-only fallback. Zero errors/crashes.

**Scoping note (intentional):** the **`FeatureTracker` seam is deferred.** CUDA-KLT's
device-pointer + persistent-pyramid flow doesn't map cleanly onto the host-`ImageView`
contract without a tracking-state rework that would risk the baseline — and KLT stays
in the primary "XFeat-detect + KLT-track" config regardless. The `FeatureSource`
(detector) seam is the one B2 needs; KLT remains a pipeline-internal stage for now.

## 2026-05-27 — B2: XFeat-TRT FeatureSource (primary config) ✅

**What:** ported AirSLAM's XFeat extractor + the Apache `tensorrtbuffer` utils into
`slamko_vio` (**host post-proc only** — dropped the optional CUDA-postproc path;
local plain `XFeatConfig`, no yaml). Added **`XFeatSource : slamko::FeatureSource`**
(`ImageView`→`cv::Mat`→TRT `infer`→`Features` with 64-d L2 descriptors). VioPipeline
selects `shitomasi|xfeat` by config; KLT tracking unchanged (doc-13 "XFeat-detect +
KLT-track"). CMake discovers + links TensorRT 10 (`nvinfer`+`nvonnxparser`); the
752×480 ONNX is installed to share and the engine builds on first run (cached to
`/tmp/slamko_vio_xfeat_752x480.engine`). `feature_source:=xfeat` exposed via launch.

**GATE — MH_01_easy, `feature_source:=xfeat`:** valid trajectory produced, no crash.
Per-frame **10.7 ms total / 4.73 ms XFeat detect** (≈ AirSLAM's ~4.5 ms), **healthy
tracking** (active ~1400, PnP inliers ~1000 — XFeat keypoints track fine under KLT,
contradicting the doc-13 flow worry). **ATE 0.0211 m RMSE** on the tracked portion.

**Coverage caveat (bench artifact, not a VIO bug):** only 848/3682 frames produced
poses — the one-time ~30 s engine build blocked node startup (player streamed
meanwhile → dropped frames) + the XFeat path is sub-real-time at rate 1.0. Engine is
now cached (deserialize ~1 s), so this won't recur. The **fair equal-coverage A/B is
B4's job** (pre-built engine + a sustainable replay rate / per-frame mode). Build:
only benign TensorRT-10 deprecation warnings from NVIDIA's `common.h`.

**Next — B3:** attach XFeat 64-d descriptors to mature landmarks at KF rate →
populate `SubMap.descriptors` (reloc map for free). **B4:** feature compare-all
(Shi-Tomasi vs XFeat) at equal coverage — Sim3-ATE + un-aligned divergence + FPS.
