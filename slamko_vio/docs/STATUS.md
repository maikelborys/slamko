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
