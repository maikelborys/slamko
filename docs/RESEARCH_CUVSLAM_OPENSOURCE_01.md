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

**Sequencing vs current focus:** Nav2 closed-loop (GAZEBO, not Isaac — user decision
2026-07-09: Isaac starves the 8 GB GPU; surfaces = D455/EuRoC bags + Gazebo) remains the
active track; R1–R3 executed 2026-07-09 as the second-provider workstream (below).

---

## 5. EXECUTED 2026-07-09 — R1+R2+R3 shipped, all measured

**Setup:** venv `~/.venvs/cuvslam` (PyCuVSLAM 16 wheel cu12/py312 + rosbags/evo/scipy).
**GOTCHA (cost an hour):** a ROS-sourced shell has `/opt/ros/jazzy/lib` on LD_LIBRARY_PATH
→ the OLD isaac_ros 4.4 libcuvslam.so shadows the wheel's bundled v16 lib → import fails
with `undefined symbol: ...LocalizeInMap...`. Fix baked into
`scripts/bench_cuvslam_provider.sh` (prepends the wheel dir). Same trap hits the binding
build (stub generation). Source build: CUDA **12.6** explicitly
(`-DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.6/bin/nvcc` — default cmake picked 13.1, which
the 580 driver rejects) + `-DCMAKE_CUDA_ARCHITECTURES=89`.

**Benchmark harness (committed a2b7778):** `scripts/bench_cuvslam_provider.{py,sh}` —
4-channel provider scorecard: accuracy (ATE Sim3 vs GT) · never-jump (5 m/s velocity-gate
teleports, same gate as legacy detect_tracking_loss.py) · trust (covariance regime census +
translation NEES + pnp_health) · realtime (track-time percentiles). EuRoC + D455-rosbag
inputs. NVIDIA example loaders imported at runtime, never vendored.

### R1b — EuRoC v16 odometry-only (Slam class NOT instantiated = the provider mode)

| seq | stereo ATE | VIO ATE | stereo NEES(3) | VIO NEES(3) | fps (compute) |
|---|---|---|---|---|---|
| MH_01 | 0.052 | 0.081 | 82 | 707 | 1844 / 698 |
| MH_03 | 0.111–0.129* | **0.084** | 177 | 769 | 1465 / 729 |
| MH_05 | 0.115 | 0.141 (1 tele) | 103 | 1078 | 1739 / 746 |

*run-to-run variance ~15% from async SBA — use `--sync-sba` for tight A/Bs.
Findings: (a) v16 IMU fixes real — VIO no longer 2× worse, wins MH_03; stereo still the
safe default. (b) These are ODOMETRY numbers — the old binary's 0.023–0.057 were
post-loop-closure `slam_path`; the ~3× gap matches the known online-vs-slam_path ratio.
(c) **Covariance NOT calibrated: NEES 82–1078 vs expected 3 → 30–350× overconfident**
(worse in VIO). §7-A settled: DCS/residual gating or a measured scale, never raw NIS.
(d) OKVIS 31-fps ceiling: shattered (0.5 ms/frame).

### R1c — casa brutal/wall (D455 640×480, stereo-only, rig from bag camera_info)

| bag | teleports (>5 m/s) | max speed | lost flagged | cov trace tele/normal |
|---|---|---|---|---|
| brutal1_trim | 78 | 2273 m/s | **0** | 52× (38% >p95) |
| wall_trim | 242–307 | 6074 m/s | **0** | 20× (52% >p95) |

vo_state pathology fully alive in v16. Covariance trace spikes on teleports but only
separates ~half the frames → SUSPECT signal (bounded multiplier), never the sole gate.

### R3 — trusted-health fork SHIPPED + VALIDATED (cuVSLAM_src branch `slamko/trusted-health`, 9f861e0)

10 files, +143 lines, minimal/upstreamable: `common::PnpHealth` {observations, inliers,
mean_residual, initial/final_cost, info_condition} populated in `PNPSolver::solve()` (one
residual pass at the final pose + eigenvalue spread of H), threaded
`ISFMSolver::lastPnpHealth()` → `VOFrameStat` → `Odometry::State::pnp_health` → PyCuVSLAM.
Multicamera solver only (mono/inertial return empty). Needs `enable_observations_export`
(measured cost: none — 1522 fps with it ON).

**Validation on wall_trim (the blank-wall killer):**

| signal | during teleports (median) | normal (median) | clean-data bound (MH_03) |
|---|---|---|---|
| **inliers** | **1** | 16 | p5 = 14, p50 = 53 |
| **info_condition** | **1e12 (H singular)** | 1497 | p95 = 1.9e4, max = 4.7e4 |
| mean_residual | 2.3e-5 | 0.31 | — |

Perfect separation with orders-of-magnitude margin. The low residual DURING teleports
confirms the source-read mechanism: garbage matches are mutually consistent → tiny cost →
success flag passes. **Provisional adapter gates: `inliers < 10 || info_condition > 1e6 ⇒
SUSPECT/REJECT`.** This recovers the true loss signal cuVSLAM never had — at the source,
feeding slamko's existing seal-on-doubt machinery (which stays ON regardless).

**Next (adapter, PLAN_CUVSLAM_PROVIDER_01 §6):** Step 0 extract ProviderIngest; adapter
links OUR fork's .so (`~/coding/cuVSLAM_src/build/bin`, NOT the APT lib — C-API mismatch);
map pnp_health → ProviderSample quality; A/B vs OKVIS per §4.2 before any default flip.
