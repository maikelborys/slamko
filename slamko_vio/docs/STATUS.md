# slamko_vio — Status log

Living, dated progress + numbers log. Append on every validated change
([`../../docs/DOC_PROCESS.md`](../../docs/DOC_PROCESS.md)). Plan:
[`PLAN_P0_vio.md`](PLAN_P0_vio.md).

> **DEPRECATED (2026-06-12 pivot, noted 2026-06-26).** Legacy own-VIO (XFeat/KLT/IMU); the
> loose-fusion pipeline uses an EXTERNAL provider (OKVIS2-X / cuVSLAM). The own-VIO sources are
> **slated for deletion** (klt_vo HEAD is strictly better) — frozen pending removal.
> **EXCEPTION — under active development: the thin provider adapters (the package's charter).**

## 2026-07-09b — Inertial (VIO) mode SHIPPED: A/B GREEN, cuVSLAM-Inertial = default mode

`use_imu` in adapter+node+launch (launch default TRUE): `OdometryMode::Inertial`, D455-tuned
noise (OKVIS rsD455 values), `rig_from_imu` from d455.urdf, IMU **buffered and drained
≤ frame_t before each Track** — direct feeding threw "Timestamps are non-monotonic" (200 Hz
IMU races the sync queue) → 100% loss; the buffer fix is load-bearing. A/B vs OKVIS:
Suave 12.8 cm/0.8% (no flat regression), Escaleras **49 cm/1.7%** (stereo-only was
79.6 cm/6.6% — the IMU fixes the stairs under-scale; climb 7.06 m vs OKVIS 6.86 m).
0 teleports all runs. Policy: cuVSLAM-Inertial = recommended provider for new runs; OKVIS
= offline baseline + fallback; battery re-baseline = follow-up. Details:
[`../../docs/RESEARCH_CUVSLAM_OPENSOURCE_01.md`](../../docs/RESEARCH_CUVSLAM_OPENSOURCE_01.md).

## 2026-07-09 — cuVSLAM provider adapter SHIPPED + E2E validated (brutal bag, live)

New (all behind `SLAMKO_WITH_CUVSLAM`, auto-ON when `~/coding/cuVSLAM_src/build/bin/libcuvslam.so`
exists — the slamko/trusted-health fork build, CUDA 12.6 sm_89):

- `include/slamko_vio/cuvslam_conversions.hpp` — pure-Eigen basis conjugation (OpenCV→ROS)
  + 6×6 covariance block-reorder (rot-first→trans-first). **3 gtests PASS** (relative-motion
  invariance, block landing, symmetry) — the dossier §2.3 "unit-test FIRST" traps.
- `include/slamko_vio/cuvslam_provider.hpp` + `src/cuvslam_provider.cpp` — PIMPL adapter,
  ODOMETRY-ONLY (never instantiates `cuvslam::Slam` → the tail re-anchor jump machinery does
  not exist in-process). Quality = covariance (Hard Rule #3): healthy → cuVSLAM 6×6
  reordered+re-based ×`cov_scale` (80, NEES-measured); SUSPECT (fork pnp_health:
  `inliers<10 || cond>1e6`) → ×`suspect_cov_mult` (25, bounded); hard invalid → nullopt.
- `nodes/cuvslam_provider_node.cpp` — stereo IR + camera_info → `/cuvslam/odometry`
  (nav_msgs, the exact contract `provider_fusion_node odom_topic:=` consumes) +
  `/cuvslam/health` (Float64MultiArray `[t,obs,inliers,residual,cost,cond,suspect,valid]`;
  Float32 t was a bug — 256 s resolution at epoch scale). RPATH-pinned to the fork lib with
  `--disable-new-dtags` (DT_RPATH beats the ROS LD_LIBRARY_PATH that shadows with the
  ABI-incompatible APT isaac_ros 4.4 lib).

**E2E validation (CASA1_brutal1 live replay, 3037 frames tracked):** 52/56 teleport-arrival
frames flagged SUSPECT (93%); published covariance during teleports **1777×** normal
(9.5e0 vs 5.4e-3) → near-free edges in the chain, bounded by quality_mult_max; 26% of
non-teleport frames also SUSPECT (genuinely degraded brutal-bag stretches — down-weighted,
not rejected). The 4 unflagged margin teleports are the slamko-side referee's job (defense
in depth — all layer-2 gates stay ON). Chain contract: cuVSLAM traj through
`provider_chain_offline` reproduces to 4e-13 m (PASS).

**P-A GATE (full fusion graph): PASS same day.** `slamko_ros/launch/pa_cuvslam_bag.launch.py`
(mirror of pa_okvis_bag) on CASA1_Suave: 4419 poses, **fused-vs-provider divergence 0.0000 m**
(exact, as the no-global-constraints gate requires); provider clean (0 teleports, max 1.2 m/s,
9.9 m span); a bounded hard-loss stretch absorbed by atlas/soft-bridge. GOTCHA: the fusion
node needs `source ~/ros2_ws/install/setup.bash` for `libnvblox_lib.so` (exit 127 otherwise —
the battery.sh S3 fix; the nvblox_ros2 in-tree symlink is DANGLING).

Numbers + method: [`../../docs/RESEARCH_CUVSLAM_OPENSOURCE_01.md`](../../docs/RESEARCH_CUVSLAM_OPENSOURCE_01.md) §5.
Next: full A/B vs OKVIS (same bags, ATE + un-aligned divergence + map density per dossier
§4.2), VPR/P-B path on cuVSLAM (`vpr:=true`), then Gazebo closed-loop.

## 2026-06-04 — VIO health-trace instrumentation + VI-BA-dropout root cause (branch klt-fork-loopclosure)

**Built a per-frame health trace** (no regression: MH_01 still 5.84–6.36 cm). `timing.csv` now
logs `reproj_rms, inlier_ratio, pnp_ok, dr_active, n_imu_interval, interval_dt, max_imu_gap,
ba_init_cost, ba_final_cost, ba_iters, ba_converged, ba_fail` (+ short-IMU-window WARN). Plumbed
through a new non-breaking `slamko::LocalSolveStats lastSolveStats()` on the `LocalSmoother` core
contract (`CeresLocalSmoother` forwards `LocalBA`'s Ceres summary). New `scripts/plot_health.py`
(health vs per-frame ATE overlay + ba_fail breakdown) and `scripts/eval_rpe_attitude.py` (rpg-style
RPE by segment + roll/pitch-vs-yaw split + Sim3 scale; attitude panel has a known body-frame ~90°
artifact to fix). `ba_use_inv_depth` exposed as a launch arg.

**Root cause found — the VI local BA is effectively dead, and that masks a broken BA:**
- `LocalBA::solve()` succeeds on only **20 / 1211** post-init KFs (MH_01); ~1206 bail `ba_fail=2`
  (landmarks_ empty). Pose runs PnP frame-to-frame + IMU-DR, NOT the fixed-lag smoother. The good
  ATE comes *because* the BA stays out of the way.
- Mechanism: `prune_landmarks_()` permanently deletes every `<2-obs` landmark at the top of every
  solve; a 1-obs landmark is erased before it can earn a 2nd obs next KF → `maxobs==1` → empties.
  Front-end is HEALTHY (consecutive KFs share 400–800 landmarks; tracks chain) — the bug is purely
  in `LocalBA` map management.
- **Reviving the BA REGRESSES accuracy** (load-bearing): relaxing prune → `ba_solved` 20→448 but
  ATE → ~1.5 m; `ba_use_inv_depth=false` → ATE 2.74 m. Both confirm the VI-BA has no marginalization
  prior / FEJ, so when it runs it drifts to meters. Prune deletion REVERTED (documented inline) to
  keep production on the accurate PnP path. Real fix = Schur marginalization prior + FEJ, then relax
  prune + re-anchor inv-depth. Full writeup: memory `slamko-vio-ba-broken-worse-than-pnp`.

## 2026-06-04 — P0 gravity-as-state + lifelong submap sealing (branch klt-fork-loopclosure)

**P0 — gravity DIRECTION is now a continuously-estimated 2-DOF state** (was frozen after a
one-shot init → tilt locked → horizontal motion leaked into Z). `imu_factor.hpp`: gravity
promoted from a const member to a 9th Ceres parameter block (`AutoDiffCostFunction<…,3,3,3,6,
3,3,3,6,3>`). `local_ba`: one shared `grav_w_[3]` block (persists across windows = warm-start)
with `ceres::SphereManifold<3>` (|g| locked to 9.81); a **`GravityPrior`** (σ≈3° toward the
init seed) firewalls regression; an **excitation gate** (pin gravity when window rot-excite <
0.10 rad) holds it steady in low-excitation segments (stairs/hover) so it can't wander;
slew-limited, renormalized write-back. Flag `estimate_gravity` (VioConfig→vio_node→launch),
**default OFF = bit-identical** (sanity: MH_01 flag-off 9.6 cm = baseline; pinned-block inert).
- **Validated A/B (EuRoC MH_01):** ATE 8.88 cm (frozen) → **6.36 cm (estimated) = −28%**,
  scale 1.002. IMU/BA unit tests green.
- **magistrale1:** Z-span 69.2→71.5 m (≈unchanged) — gravity refinement does NOT fix
  magistrale's vertical drift, which is dominated by gyro-orientation drift over 13 min
  open-loop + real multi-floor, NOT gravity tilt. Confirms: long-run floor-tilt is a
  loop-closure problem, not an odometry one. Parametrization = S² (VINS RefineGravity math,
  idea-only); plan workflow `wl1pyrl12`, memory `slamko-vio-outdoor-drift-diagnosis`.

**Lifelong submap sealing.** `vio_pipeline` seals the current submap every `kf_per_submap`
KFs (or `submap_seal_metres`) into `submap_dump_dir` as `.smap` (saveSubMaps): `buildSubMap(id)`
sets a first-KF anchor + rebases payload to submap-local; disjoint epochs; XFeat descriptors +
per-KF VPR. **OFF when dir empty → odometry byte-identical** (seal reads state, never mutates).
Validated MH_01 (53 submaps) + magistrale1 (578). Feeds the offline loop-closure driver.

## 2026-05-28 — Deterministic replay: IMU↔frame gating (was ~40-80% ATE variance) ✅

**Problem:** same code+config gave wildly different ATE between runs — EuRoC V1_03_difficult
raw-odom SE3-ATE **52.9 / 74.7 / 95.3 cm** across runs (~40-80% variance). NOT frame drops (both
runs processed ~all 2147 frames). Root cause: the **IMU↔frame association race** in the ROS
replay. `rclcpp::spin` doesn't fix the imu-cb vs stereo-sync-cb callback order, so a stereo frame
could be processed before all its preceding IMU samples arrived; `VioPipeline::drain_imu_window`
then DISCARDS the late samples (`s.t < t_lo - 0.01`) → incomplete preintegration → divergence,
amplified on hard sequences near the tracking limit.

**Fix (`nodes/vio_node.cpp`):** frame-gating. `on_stereo` now buffers frames in `pending_frames_`
(holding the msg shared_ptrs); `on_imu` advances `latest_imu_ts_` and calls
`drain_pending_frames()`, which processes a buffered frame ONLY once `latest_imu_ts_ >= frame.ts`
— so `drain_imu_window` always sees the COMPLETE window regardless of executor callback order.
`use_imu_gate_` (= `enable_imu`) bypasses gating in no-IMU mode; the trailing ≤1 frame after the
last IMU sample is dropped (negligible for ATE).

**Result (two identical V1_03 runs):** trajectory delta **max 1.2 cm / mean 0.64 cm** over the
whole run (was tens of cm in ATE) — **~50-80× nondeterminism reduction**, ATE A/B now trustworthy.
Residual ~0.64 cm is GPU (XFeat-TRT FP16 / KLT-CUDA, not bit-identical) — negligible, not chased.
VIO confirmed healthy independently: EuRoC MH_05 raw-odom Sim3 scale **1.0127** (metric), ATE
~18-22 cm. See memory `slamko-vio-replay-nondeterministic`.

**Follow-up same day — frame drops (reliable QoS).** A second nondeterminism source surfaced in
the neverlost+reloc path: heavy GPU work (VPR every frame + LighterGlue) stalls the single-threaded
executor, and the image subs used `rmw_qos_profile_sensor_data` (BEST_EFFORT depth 5) so the rmw
silently DROPPED stereo frames (V1_03 2147→2134, ~80 cm OFF-vs-ON divergence). euroc_player
publishes RELIABLE, so the image/caminfo subs are now **RELIABLE + KeepLast(100)** (sync queue
20→100) → the player back-pressures instead of dropping → lossless replay. After this both OFF/ON
process all frames and the **loop-closure-CORRECTED ATE is reproducible** (OFF 39.21 vs ON 39.13 cm)
→ ATE A/B trustworthy; LighterGlue rescue == brute-force on easy revisits (no regression). Residual
~30 cm RAW-odom divergence on hard seqs is GPU (XFeat/KLT FP16), washes out post loop-closure.

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

## 2026-05-27 — B3: descriptor attachment @ KF rate (reloc map for free) ✅

**What:** each track captures its 64-d XFeat descriptor at birth (from the
`FeatureSource` detect output; descriptor-less Shi-Tomasi simply skips it). It
travels with the track through KLT culling, and at KF rate — when a track is
promoted to a landmark — the descriptor is stamped onto the landmark. New
`VioPipeline::buildSubMap()` assembles the global map as a `slamko_core::SubMap`
(landmarks + an N×64 descriptor index, `MapLandmark::descriptor_row`).

**GATE — short XFeat clip (MH_01, end_s=25):** `submap @ shutdown: 112341 landmarks,
112341 with descriptors (index 112341×64)` — the descriptor index is fully
populated; the reloc map is built with zero hot-path cost. slamko_loop (P2) will
consume this via `Relocalizer::addSubMap`.

**Caveat (inherited, to fix in P2):** `landmark_world_` + the descriptor store grow
unbounded (klt_vo's cumulative map never prunes) — fine for a session, but the reloc
index should be restricted to submap-keyframe landmarks once submap management lands
in slamko_loop.

## 2026-05-27 — B4: XFeat full-coverage validation (Milestone B closed) ✅

**What:** with the engine cached (~1 s startup, no build-drop), re-ran XFeat on the
full MH_01 sequence — **3682 frames processed, 3419 poses (equal coverage to the
Shi-Tomasi baseline)**, at rate 1.0 (real-time-capable, ~93 fps).

**Equal-coverage A/B (MH_01, IMU-on, Sim3-ATE, both 3419 poses):**

| front-end | ATE RMSE | mean | median | fps |
|---|---|---|---|---|
| Shi-Tomasi (baseline) | 0.078 m | 0.073 | 0.065 | ~214 |
| **XFeat-TRT** | **0.049 m** | 0.045 | 0.040 | ~93 |

**Verdict:** XFeat is **~37 % better** on equal coverage — a real, honest win (the
earlier 0.021 m was a partial-coverage artifact from the one-time engine build
dropping frames). XFeat keypoints track fine under KLT and stay real-time. Shi-Tomasi
stays as the fast fallback. **No exhaustive multi-sequence compare-all run** (per user
— XFeat is the chosen path; comparing-to-win adds nothing). Multi-sequence regression
sweeps remain available via `scripts/bench_all_euroc.sh` when needed.

**Milestone B (P0) closed:** swappable learned-feature VIO — ROS-agnostic
`VioPipeline`, Shi-Tomasi/XFeat behind `slamko_core::FeatureSource`, IMU-fused,
short-gap dead-reckoning, descriptors attached for reloc. Next phase: **P1
`slamko_fusion`** (GTSAM iSAM2 + marginalization) → refactor vio onto `T_WB` + the
`Factor` contract, guarded by this baseline.

## 2026-05-27 — P1b: VioPipeline routed through the LocalSmoother contract (ceres) ✅

**What:** `VioPipeline` now talks to the abstract `slamko_core::LocalSmoother`
(`smoother_`) instead of `LocalBA` directly. New **`CeresLocalSmoother`** adapter
(`include/src/ceres_local_smoother.*`) wraps klt_vo's `LocalBA` behind the contract,
bridging the two boundaries: pose frame **T_WB↔T_w_c** (`T_WB = (E·T_w_c)⁻¹`,
E = T_BS cam→body) and **raw-IMU→preintegration** (the adapter owns the klt_vo
`ImuPreintegration`, the exact call the pipeline used to make). The two mid-stream
`LocalBA` rebuilds (T_BS resolve, gravity calib) became `setExtrinsics`/`setImuParams`.
A `backend:=ceres|gtsam` param + an injection ctor (`VioPipeline(cfg, unique_ptr<LocalSmoother>)`)
let the **node** swap in `slamko_fusion::GtsamLocalSmoother` in P1c **without a vio→fusion
dependency** (Hard Rule #2). For now `backend:=gtsam` warns + falls back to ceres.

**GATE — exactness (deterministic, the real proof):** 4 gtests in
`test_ceres_local_smoother.cpp` prove the adapter is a **byte-exact pass-through to
LocalBA (≤1e-9)** for BOTH the visual-only AND the IMU `insert_keyframe_with_imu`
path (pose + refined velocity + bias), plus a non-identity-extrinsic round-trip.
`colcon test`: **23 tests, 0 failures**.

**GATE — end-to-end (MH_01, Sim3-ATE, 3419 poses each):** the plan's "≤1 mm
reproduction" gate proved **unmeasurable — the VIO is nondeterministic run-to-run**
(~3 cm ATE spread; source = CUDA front-end atomics → `solvePnPRansac`, entirely
**upstream of and unaffected by** this Tier-2 seam). Characterised the band instead:

| binary | Shi-Tomasi ATE samples (m) | XFeat |
|---|---|---|
| pre-refactor (5 runs) | 0.066, 0.067, 0.080, 0.087, 0.096 | 0.054 |
| **post-refactor ceres (4 runs)** | **0.065, 0.073, 0.074, 0.080** | **0.049** |

Distributions **overlap completely** (combined Shi-Tomasi median 0.074 ≈ documented
0.078) → behavior-preserving, consistent with the exactness proof. FPS unchanged:
~206–211 (Shi-Tomasi) / ~81 (XFeat) — the adapter is off the per-frame hot path.

**Gate restated (honest):** literal bit-reproduction is impossible (the contract
forces poses through `slamko::SE3`, which re-normalises the quaternion ~1e-15) AND
moot (front-end nondeterminism dominates). The **unit-exactness proof** is the gate;
the bench confirms no end-to-end regression. **Next — P1c:** wire `backend:=gtsam` at
the node (node gains the `slamko_fusion` dep), validate the GTSAM smoother
end-to-end on EuRoC (marginalization + IMU + accuracy + health probes).

## 2026-05-27 — P1c: GTSAM backend wired + tracks end-to-end (the IMU regime) ✅

**What:** `backend:=gtsam` now injects `slamko_fusion::GtsamLocalSmoother` at the
**node** (composition root) — `slamko_vio_core` still links only `slamko_core`
(Hard Rule #2 intact). GTSAM is fully encapsulated in the now-**shared**
`libslamko_fusion.so` (RPATH → GTSAM 4.3 at `/usr/local`), so the node links the
`.so` and never finds/links gtsam itself. Confirmed: `ldd` resolves
`libgtsam.so.4 → /usr/local` (the 4.3, not the 4.2 at `/usr`).

**The load-bearing fix — a latent P1a bug.** GTSAM's `CombinedImuFactor` ctor takes
`(pose_i, vel_i, pose_j, vel_j, bias_i, bias_j)` — the smoother passed
`(X_{i-1},V_{i-1},B_{i-1},X_i,V_i,B_i)`, swapping the bias/pose_j args, so GTSAM read
`V(i)` as a bias → `"retrieve vN as ConstantBias"`. It **never fired in P1a** (its only
test is visual-only, `use_imu=false` — the IMU factor was never exercised). This is
exactly what end-to-end P1c was for. Also added, for the smoother to survive real data:
- **Reset-on-setter** (`setExtrinsics`/`setImuParams` rebuild the window) so the
  `X(i)` index stays in step with the pipeline's VI-init restarts (CeresLocalSmoother parity).
- **V/B nodes only inside the IMU chain** (never for visual-only KFs — their
  unconstrained velocity/bias made fixed-lag marginalization throw); first IMU-ready
  KF anchors with priors.
- **Landmark management** — admit a landmark only on its ≥`min_landmark_obs`-th
  sighting + a weak prior + cap `max_landmarks=150`. Without it: an indeterminant
  system (single-view points) AND an **8-SECOND** per-KF batch solve (thousands of
  landmark vars). With it: `ms_ba` ~13 ms median (real-time), 0 indeterminants.
- **`setStereoCalib` wired in the pipeline** (was never called → both backends ran on
  zero intrinsics; the GTSAM stereo factors need the calib).

**GATE — end-to-end (MH_01, shitomasi):** GTSAM **tracks end-to-end with 0 smoother
failures**, real-time (`ms_ba` ~13 ms median, max ~196 ms at full-window KFs). On a
clean early segment, Sim3-ATE **0.025 m** (72 poses); 40-s clip 0.16 m (incl. the VI-init
transient). Robustness gate (per the project thesis — never-lost > accuracy): **met**.
**Honest deferral:** a clean *full-sequence* regression ATE (gtsam vs the ceres band
0.065–0.096) is **blocked by harness flakiness on this box** — (a) the rosbag2 recorder
records 0 msgs despite the node publishing at 20 Hz (DDS delivery to the recorder is
wedged; `rosbag2` works in isolation, `ros2 topic hz` sees the stream), and (b)
orphaned `slamko_vio_node`/`euroc_player` from killed runs corrupted runs (duplicate
publishers, SIGKILL-9 early deaths). Fixes landed: an **in-process pose→TUM dump**
(`pose_dump_path`) that bypasses rosbag2 for ATE, and a **zombie guard** in
`scripts/bench_ate.sh` + a CLAUDE.md rule. Full-sequence regression + flipping the
**default to gtsam** are deferred until a clean harness; **default stays `ceres`**
(validated-stable) — gtsam is validated-working.

**Unit gate:** `colcon test` slamko_fusion + slamko_vio = **28 tests, 0 failures**
(the gtsam smoother test still passes with reset + landmark mgmt + the factor fix).

## 2026-05-27 — multi-window forced-loss + never-lost pose-graph node params ✅

**What:** supporting hooks for the slamko_loop P2 live close-out (multi-submap merge) —
all node/pipeline glue, no algorithm change.
- **Multi-window forced-loss:** `VioConfig.dr_force_loss_windows` (a list of `[start,end)`
  s, rel to seq start) + the node string param `dr_force_loss_windows="s:e,s:e"`. The
  pipeline ORs these with the existing single `dr_force_loss_start/end_s` window, so one
  replay can induce SEVERAL tracking-loss episodes → several sealed submaps. Backward-
  compatible (empty list = old behavior).
- **Never-lost params:** `neverlost_use_pose_graph` (default off) + `neverlost_weld_once`
  (default on) plumbed into the lazily-built `SupervisorConfig` in `driveSupervisor`.
- **Launch:** `vio_euroc.launch.py` exposes the three args; bool args go through a
  `_bool()` = `ParameterValue(LaunchConfiguration(name), value_type=bool)` helper — a bare
  `LaunchConfiguration` resolves to a string and a bool node-param silently keeps its
  default (the bool analog of the `30` vs `30.0` double lesson; saw `pose_graph=0` despite
  `:=true`).

**Validated via slamko_loop** (V1_01, xfeat, 2 forced-loss windows): 2 SEALs + 2 WELDs,
pose-graph merge of 2 sealed submaps, weld-once held (1 weld/episode). See
`slamko_loop/docs/STATUS.md` (2026-05-27 P2 CLOSED). No new unit tests here (glue);
the loop suite (32 gtests) + the live run are the gate.

## 2026-05-27 — per-submap map sidecars for never-lost merge viz ✅

**What:** glue so the merged multi-submap map can be visualized in the corrected frame
(the raw `landmark_dump`/`pose_dump` are the uncorrected odom frame — DR drift baked in).
- `VioPipeline::maxLandmarkId()` — landmark IDs are monotonic, so this is the creation
  seam used to partition the map per never-lost submap.
- Node: records the id seam at each SEAL → writes `<landmark_dump>.submaps` at shutdown
  (per-submap `id_lo,id_hi` + the final welded anchor 3×4) + a per-frame
  `<pose_dump>.epoch` (active submap id, lockstep with the TUM dump).
- `scripts/plot_neverlost.py --submaps --pose-epoch` applies each submap's anchor
  (`map = anchor·odom`) before the Sim3 fit. Detail: `slamko_loop/docs/STATUS.md`
  (2026-05-27 merge visualization fix). On V1_01: Sim3-ATE 56.9 → 31.1 cm with the
  correction; submap 2's anchor was a ~49° rotation + 2.43 m (the measured blackout-2 drift).

## 2026-05-27 — submap epoch partition (disjoint never-lost submaps) ✅

`buildSubMap()` returned the whole cumulative `landmark_world_` (never pruned), so
never-lost sealed submaps were cumulative supersets (duplicated landmarks in the reloc
DB). Now each landmark is tagged with a `submap_epoch_` at creation; `beginSubmap()`
(called by the node on BRANCH) bumps the epoch; `buildSubMap()` returns only the active
epoch's landmarks → disjoint, self-contained submaps. **Epoch 0 with no branch ⇒
byte-identical to before** (normal/no-loss runs unaffected; the validated single-submap
reloc path unchanged). V1_01 multi-loss re-run: SEAL 0 = 40,615 lm, SEAL 1 = 90,707 lm
(own epochs, not cumulative); both welds fired; `scripts/check_neverlost.py` 7/7 PASS
(corrected ATE 16.9 cm). Detail: `slamko_loop/docs/STATUS.md` (2026-05-27 submap partition).

## 2026-05-27 — P4b-1: cross-session map I/O wiring (node) ✅

Node params `prior_map_dir` (loadSubMaps at startup → seed `XFeatRelocalizer` DB +
`supervisor.seedPriorMap`) and `map_save_dir` (saveSubMaps of the sealed Atlas + active at
shutdown). Cross-session welds are flagged in the log (`[CROSS-SESSION/prior map]` when
`welded_to_id < first_live_id`). Two-session V1_01 run: session 1 saved a 1-submap Atlas,
session 2 loaded it and welded into it (`WELD to submap 0 [CROSS-SESSION]`), `check_neverlost.py`
7/7 PASS (ATE 13.9 cm). Detail: `slamko_loop/docs/STATUS.md` (2026-05-27 P4b-1).

## 2026-05-27 — P4b-2: continuous-reloc node param ✅

Node param `neverlost_continuous_reloc` (default off) → `SupervisorConfig.continuous_reloc`:
the live session welds into a prior map / closes loops while OK (no forced loss). Validated
session 2 on V1_01 (prior map loaded, no loss): `WELD to submap 0 [CROSS-SESSION]` at ~2 s
in the OK state, `check_neverlost.py` 7/7 PASS (ATE 6.9 cm). Detail: `slamko_loop/docs/STATUS.md`.
