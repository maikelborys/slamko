# slamko_vio — P0 plan (the swappable learned-feature VIO)

<!-- validated: 8498021 2026-05-27 · tests: P0 shipped — equal-cov MH_01 ShiTomasi 0.078 / XFeat-TRT 0.049 -->

Detailed plan for Milestone B. Read [`../../CLAUDE.md`](../../CLAUDE.md) +
[`../../docs/SYSTEM.md`](../../docs/SYSTEM.md) +
[`../../docs/DECOUPLING.md`](../../docs/DECOUPLING.md) first. Progress + numbers
land in [`STATUS.md`](STATUS.md).

## Goal

A fast stereo-inertial VIO whose feature front-end is **swappable behind the
`slamko_core` contracts** (`FeatureSource`/`FeatureTracker`), seeded from the
validated klt_vo tracker (MH_01 ATE 0.054 m @ ~240 fps). Primary target config:
**XFeat-detect (TensorRT) + KLT-track + descriptor@KF**. Shi-Tomasi stays as the
baseline backend; LiftFeat-m1 is a later backend.

## Target structure (after B1b — top-tier: ROS-agnostic core + thin node)

```
include/slamko_vio/
  feature/{shitomasi_source,klt_flow_tracker,xfeat_source}.hpp  # slamko_core impls
  stereo/{stereo_matcher,triangulator}.hpp
  pnp/{pose_estimator,pnp_cuda}.hpp
  backend/ceres_local_ba.hpp        # LocalBA = the CeresBackend S1 inner loop
  imu/{imu_preintegration,dead_reckoner}.hpp
  vio_pipeline.hpp                  # orchestrator, NO ROS: (ImageView,ImuSample)→EstimationFrame+HealthSignal
  vio_config.hpp
src/*.{cu,cpp} · nodes/vio_node.cpp (thin ROS glue) · launch/vio_euroc.launch.py
```

`VioPipeline` is ROS-free → bag/sim/real-portable + unit-testable. The node only
does subs/params/pub and wraps `cv::Mat`→`ImageView` at the boundary. This is the
structural upgrade over klt_vo's 1473-line monolithic node.

## Refactor strategy — phased, regression-guarded

**B1 · faithful port (monolith intact).** `cp` klt_vo_core + node into slamko_vio
FLAT (`include/slamko_vio/*.hpp` mirroring klt_vo's layout), rename `klt_vo::`→
`slamko_vio::` + include paths. Self-contained (ports klt_vo's own `types.hpp`; no
slamko_core dep yet — lowest-risk). Build standalone in the slamko colcon workspace,
CMake mirrors klt_vo (CUDA sm_89, Ceres, OpenCV 4, ROS jazzy). **GATE: MH_01 ATE ≈
0.054 m (imu) @ ~240 fps; sanity the per-seq table.**

**B1b · decompose under the guard.** Reorganize into the structure above; extract
`VioPipeline` (ROS-agnostic) + thin `vio_node`; lift detect→`ShiTomasiSource :
slamko_core::FeatureSource`, KLT→`KltFlowTracker : slamko_core::FeatureTracker`.
slamko_core dependency enters here. Stereo/PnP/BA/LMP/DR become pipeline stages,
logic unchanged. **GATE: full EuRoC suite numbers unchanged (±noise).**

**B2 · XFeat-TRT FeatureSource (primary).** Port AirSLAM `xfeat.cpp` +
`xfeat_postproc.cu` + minimal `tensorrtbuffer/buffers.h` (Apache headers; preserve
attribution — rewrite any GPL glue clean). Reuse `xfeat.onnx`/`xfeat.engine`
(static 752×480 = EuRoC-native), build engine on first run. `XFeatSource :
FeatureSource` → `Features` with 64-d L2 descriptors. Wire **XFeat-detect +
KLT-track**, swappable via `feature_source:=shitomasi|xfeat`. **GATE: valid
trajectory.**

**B3 · descriptor attachment @ KF rate.** Attach XFeat 64-d descriptors to mature
landmarks at keyframe rate → populate `SubMap.descriptors` (reloc map built for
free; hot path untouched). **GATE: SubMap carries a populated descriptor index.**

**B4 · feature compare-all benchmark (P0 deliverable).** Port harness to
`slamko/scripts/`; A/B Shi-Tomasi vs XFeat on EuRoC reporting **Sim3-ATE +
un-aligned divergence + FPS** (hard rule #5). Table → STATUS.md.

## Licensing (hard rule #1)

XFeat algorithm + weights/onnx = Apache-2.0 (reuse). AirSLAM `xfeat.cpp` carries an
Apache header (clean port, keep attribution). The relocalizer/Map-coupled code is
GPL — **not** copied; rewrite any needed glue fresh under Apache-2.0. klt_vo code is
already Apache (ours).

## Bench infra (verified present)

EuRoC at `~/datasets/euroc/{MH_01_easy,...}`; `euroc_publisher` built in
`~/coding/isaac_ros_ws/install`. Bench sources `/opt/ros/jazzy` +
`isaac_ros_ws/install` + the slamko `install/`. Topics: `/euroc/{left,right}/
image_rect_raw` + `/euroc/imu`. ATE via `evo_ape` vs `state_groundtruth_estimate0`.

## Validation gate discipline

Each gate: log numbers to STATUS.md, bump validated stamps, commit code+docs
together (DOC_PROCESS). Regression ≥5% on a gate → diagnose root cause → fix or
revert before proceeding. Report BOTH Sim3-ATE and un-aligned divergence (Sim3
hides catastrophic divergence — hard rule #5).
