<!-- validated: research-only (4-agent source study of cuVSLAM 16.0.0, no code) · 2026-07-09 -->
<!-- updates docs/PLAN_CUVSLAM_PROVIDER_01.md: several §7 unknowns are now RESOLVED by reading source -->

# RESEARCH — cuVSLAM is now OPEN SOURCE: what the code reveals & the improvement routes

**Date:** 2026-07-09 · **Source studied:** `~/coding/cuVSLAM_src` = github.com/nvidia-isaac/cuVSLAM
@ v16.0.0 (2026-06-02; first open release v15.0.0 on 2026-03-02). Full source: C++ 76% /
CUDA 8% / Python 12%, 489 files — tracker, PnP, SBA, IMU, SLAM, all readable.
**Method:** 4 parallel source-reading agents (odometry/health · slam/map · IMU · API/license)
cross-checked against `docs/PLAN_CUVSLAM_PROVIDER_01.md` (written 2026-06-26 against the
closed binary) and the empirical findings in `~/coding/cuvslam/CLAUDE.md`.

---

## 1. License (NVIDIA Community License, v. 2026-02-25)

- Commercial use YES; derivative works YES; **linking is explicitly NOT a derivative work**
  (`LICENSE:27`) → slamko (Apache-2.0) may link `libcuvslam.so` as an external dep.
- **Vendoring source into slamko: NO** — copied code stays NVIDIA-licensed, cannot be
  relicensed Apache-2.0. Patch in a fork of cuVSLAM_src, build our own `.so`, link it.
- Field-of-use: NVIDIA hardware only (we run RTX 4070 — fine). Ship their LICENSE with any
  redistributed binary. "Feedback" sent to NVIDIA is royalty-free licensed to them.
- **The v16 `.so` is NOT a drop-in for the APT isaac_ros 4.4 lib** — the C API was removed
  in v15 (`CHANGELOG.md:42`); only the C++ `cuvslam2.h` API exists. The slamko adapter must
  link our own build, not swap the system lib.

## 2. Every black-box mystery we measured is now EXPLAINED in source

| Empirical finding (ours, blind) | Root cause (source) |
|---|---|
| `vo_state=1` while pose teleports 50 m at a blank wall | PnP success = `avg_huber_cost < 0.6 \|\| cost < initial_cost` — **any** cost decrease passes (`libs/pnp/multicam_pnp.cpp:281`). No RANSAC / no inlier-count / no velocity gate in the stereo path. |
| "Flat" `vo_pose_covariance` (NEES useless) | Single-frame PnP Hessian inverse with **Identity fallback on still frames** (`multicam_pnp.cpp:219`), **Zero-info on empty map** (`track_online_multi.cpp:89`), previous-frame reuse on failure; count-normalized in the mono path → magnitude ~independent of feature count. SBA marginals never fed back (`cuvslam2.cpp:543-553`). |
| EuRoC: VIO WORSE than stereo-only (+30–135% ATE) | Biases init to ZERO, gravity from raw accel average assuming static (`libs/imu/inertial_optimization.cpp:265-290`), 20-KF/2-s init window, adaptive NEC fallback **disabled by hardcoded flag** (`track_online_inertial.cpp:292`). PLUS: v16 fixed "two IMU integration bugs causing stereo+IMU to underperform stereo-only" (`CHANGELOG.md:17`) — our binary predates the fix. **Re-benchmark VIO on v16.** |
| D455 casa teleports until IMU noise ×20 | Hardcoded **EuRoC** noise defaults, used even by the RealSense example (`libs/common/imu_calibration.h:47-51`, `examples/realsense/camera_utils.py:141`). Injection point: `ImuCalibration` fields (`cuvslam2.h:203-213`). Hidden second tuning surface not in the API: `imu_penalty=1e-3` etc. (`libs/pipelines/inertial_pnp.cpp:44-53`). |
| 3.56 m single-frame slam_path jumps on loop closure | `Tail::UpdatePoseBySLAM` applies the FULL correction delta in one shot; only gate is a 1 cm isApprox early-out — **no slew limit, no blending** (`libs/slam/async_slam/tail.cpp:50-78`). Same unsmoothed path after LocalizeInMap. → our `gate_live_pose` addresses a real hole. |
| Recovery weirdness after loss | One lost frame → `reset()` **wipes the entire odometry sliding map** + prediction (`multi_visual_odometry_base.cpp:45`). IMU-only coast max **1 s**, then full re-init needing 20 KF / 2 s (`tracker_state_machine.h:31-36`). |

Other load-bearing source facts:
- **Odometry is cleanly standalone** — `class Odometry` has zero dependency on `class Slam`;
  build-level one-way (`libs/odometry/CMakeLists.txt` links only math). Not instantiating
  `Slam` removes the tail-jump machinery **by construction**. Perfect fit for the
  odometry-only provider design.
- Their SLAM layer is weaker than slamko_loop at exactly our specialty: place recognition is
  **pose-guess-driven spatial retrieval** (no appearance/VPR at all; Shi-Tomasi patches +
  NCC≥0.9, `lcs_simple.cpp:189-248`), reloc = brute-force x/y/z/yaw probe sweep around a
  guess (`localizer.cpp:226-260`), **single map**, no kidnapped-robot reloc, `LocalizeInMap`
  *replaces* the current map. PGO = custom LM on SE(3) (`libs/math/pgo.cpp`), max 10 iters.
  → Nothing to adopt; slamko keeps owning loop/Atlas/reloc (double-loop-closure rule stands).
- Rich health signals **exist but are neither exposed nor gating**: per-obs residuals
  (`multicam_pnp.cpp:139`), PnP Hessian (`:277`, condition never checked), survivor ratio
  (KF selector, 41%), mono-only inlier counts. `VOFrameStat` carries only
  keyframe/heating/tracks2d/tracks3d (`libs/odometry/ivisual_odometry.h:47`).
- `disable_fusion_except_gravity=true` hardcoded in the public API ctor
  (`cuvslam2.cpp:404-406`) — but for ≤2-camera rigs full Inertial-PnP+SBA still runs; NVIDIA's
  own TROUBLESHOOTING recommends considering IMU OFF (`TROUBLESHOOTING.md:414-417`).
- PyCuVSLAM (nanobind) covers the FULL C++ surface incl. landmarks, IMU state, pose-graph
  read, map save/localize → **deterministic offline benchmarking without ROS**.
- Build: `cmake -DCMAKE_CUDA_ARCHITECTURES=89 ..` (RTX 4070) → `build/bin/libcuvslam.so`.

## 3. Verdict for the slamko architecture

**The source study VALIDATES the untrusted-provider ideology wholesale.** Every defensive
layer slamko built blind (imu_referee, gate_live_pose, seal-on-doubt, motion-proportional
edge information ignoring provider covariance, provider-agnostic eval) maps 1:1 to a real,
now-visible hole in the provider. No layer is redundant. Keep them all ON even if we patch
cuVSLAM (defense in depth: patches reduce garbage at the source; slamko gates remain the
guarantee).

## 4. Improvement routes (ordered)

**R1 — Offline de-risk with PyCuVSLAM v16 (no ROS, no code).** Resolves dossier §7-A/B/C by
measurement on the fixed IMU path: (a) re-run EuRoC VIO-vs-stereo A/B on v16 (the two IMU
bug fixes may flip the old conclusion); (b) NEES/trace study of the covariance knowing its
fallback regimes (filter Identity/Zero frames before fitting `nominal_var_t`); (c) export-
flags fps cost. Also fixes the OKVIS-GPU-nondeterminism A/B problem: deterministic offline
replay of the provider.

**R2 — Build v16 from source (sm_89) + slamko adapter per PLAN_CUVSLAM_PROVIDER_01 §6,
linking OUR .so.** Odometry-only (never instantiate `Slam`) → no tail jumps by construction;
feed real D455 IMU noise via `ImuCalibration` (kills the EuRoC-defaults pathology properly,
no ×20 hack). Provider selectable `okvis|cuvslam`, OKVIS stays default until §4.2 A/B green.

**R3 — Minimal "trusted-health" fork (the NEW opportunity, impossible before).** Localized
patches, kept as a branch of cuVSLAM_src (license-clean), candidates ordered by value:
  1. Expose per-frame `inlier_count / mean_residual / hessian_condition` via
     `VOFrameStat`+`State` (populate in `multicam_pnp.cpp:297-308`, thread through
     `multi_visual_odometry_base.cpp:88-99`) → slamko gets REAL provider health channels.
  2. Fix the covariance: drop the `setIdentity()` still-frame shortcut, flag near-singular H
     instead of silently inverting, optionally feed SBA marginals back.
  3. (Optional; slamko already gates this) velocity-sanity at
     `multi_visual_odometry_base.cpp:108-113`; or tighten `multicam_pnp.cpp:281` (drop the
     `|| cost < initial_cost` arm, add min-inlier).
  Keep patches minimal + upstreamable (issues/PRs accepted; note the Feedback clause).

**R4 — Do NOT adopt their SLAM/map layer.** Confirmed weaker than slamko_loop on every
immortality axis (no appearance reloc, single map, unsmoothed re-anchor, map-replace load).

**R5 — Design lessons that CONFIRM slamko choices (no action):** their 1-s coast + hard
reset + map wipe vs our HOLD/seal-on-doubt-island model; their guess-driven reloc vs our
VPR+proximity; their unsmoothed tail vs our gated live pose.

**Sequencing vs current focus:** Isaac-Sim/Nav2 closed-loop remains the active track. R1 is
cheap, offline, and parallelizable; R2/R3 are the "second provider" workstream to schedule
after (or alongside) the Nav2 milestone. R3 without R2 is useless; R2 without R1 violates
the measure-before-build method.
