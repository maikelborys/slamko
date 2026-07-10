# slamko — Pipeline status & cold-start (2026-06-13, major update 2026-07-09)

<!-- validated: 2026-07-09 · the consolidated "where we are NOW" snapshot of the
loose-fusion pipeline. Chronological detail: slamko_ros/docs/STATUS.md.
Plan: MASTER_PLAN.md §8. Research provenance: docs/REBUILD_PROPOSAL_01.md. -->

**Read this first if you're starting cold.** It is the one-page truth of what
runs today, the exact commands, the load-bearing gotchas, and the queue.
Everything below was validated on the real D455 casa bags.

---

## 0'. 2026-07-10 — MULTI-SESIÓN DIAGNOSTICADO A FONDO + OBJETIVO NORTE (dirección POR DECIDIR)

Un día entero de multi-sesión EuRoC con el motor nativo cuVSLAM: par MH01→03 = **8.6 cm
(banda ORB-SLAM3)**; cadena de 5 corre entera; el gap a ~1 m quedó diagnosticado capa a
capa hasta el veredicto (binding LC acotado por viewpoint; **la fusión-a-la-creación es
EL mecanismo** si se necesita coherencia aérea; el descriptor XFeat ABSUELTO: 0.102 m en
orbslam3_xfeat con la misma red). 6 commits en el fork (pnp_health, fix descriptores
re-save, pase LC cross-session completo). **Cómo seguimos: NO DECIDIDO** — entrada:
[`OBJETIVO_NORTE_01.md`](OBJETIVO_NORTE_01.md) (bifurcación terrestre/dron) +
[`PLAN_CUVSLAM_MULTISESSION_01.md`](PLAN_CUVSLAM_MULTISESSION_01.md) (cronología completa).

---

## 0. 2026-07-09 — cuVSLAM OPEN-SOURCE: 2nd PROVIDER SHIPPED + A/B + DEPLOY PLAN (read FIRST)

**cuVSLAM went full open source** (v16, github.com/nvidia-isaac/cuVSLAM, clone at
`~/coding/cuVSLAM_src`) — every blind-measured pathology now explained with file:line
(vo_state gate, flat covariance, IMU zero-bias init, tail jumps) and the untrusted-provider
ideology validated 1:1. One-day execution, 11 commits; the full story with all numbers:
[`RESEARCH_CUVSLAM_OPENSOURCE_01.md`](RESEARCH_CUVSLAM_OPENSOURCE_01.md) + memory
`slamko-cuvslam-opensource`.

- **Fork `slamko/trusted-health`** (branch in cuVSLAM_src, our own `.so`, CUDA 12.6 sm_89):
  exposes per-frame `pnp_health` (inliers/residual/H-condition). Wall-bag validation:
  teleports have inliers=1 + H singular (1e12) vs clean floor p5=14 / max 4.7e4 →
  **adapter gates `inliers<10 || cond>1e6`**. License: link OK from Apache-2.0, NEVER vendor.
- **Provider adapter SHIPPED** (`slamko_vio`: `CuvslamProvider` PIMPL + `cuvslam_provider_node`
  → `/cuvslam/odometry` + `/cuvslam/health`; launch `pa_cuvslam_bag.launch.py`). Odometry-only
  (never instantiates `Slam` → no tail-jump machinery in-process). P-A gate PASS (0.0000 m).
  E2E brutal: 52/56 teleports SUSPECT, covariance ×1777. **IMU interleave gotcha (load-bearing):**
  feed IMU BUFFERED and drained ≤ frame_t before each Track, else "Timestamps are non-monotonic"
  → 100% loss.
- **A/B vs OKVIS + Inertial mode:** stereo-only under-scaled Escaleras 6.6% → `Inertial`
  (D455-tuned noise, rig_from_imu from d455.urdf) fixes it: Suave 12.8 cm/0.8%, Escaleras
  49 cm/1.7%, 0 teleports. **Policy: cuVSLAM-Inertial = recommended provider for NEW runs;
  OKVIS = existing launches/battery default + offline baseline + fallback.**
- **Flash-bag verdict (honest, map shown):** cuVSLAM-Inertial degrades on the 45 fps flash
  stream (147 teleports) → 35 sealed islands along drift, NO false weld (honest-dangling
  working as designed). **OKVIS remains the flash-bag provider.** To diagnose: dotted-IR
  leakage through the splitter vs fast-motion@45fps. Chain script:
  `scripts/run_slamko_casa_flashbag_cuvslam.sh`; visuals in `results/cuvslam_v16/viz/`
  (`plot_cuvslam_ab.py`: escaleras 3D A/B, wall health timeline, flash TSDF floorplan+3D).
- **Provider bench harness:** `scripts/bench_cuvslam_provider.{py,sh}` — 4-channel scorecard
  (ATE/teleports/NEES+cov-census/fps) on EuRoC + D455 bags. Measured: cuVSLAM covariance
  30–350× overconfident (NEES) → `cov_scale=80`; 0.5 ms/frame (OKVIS 31-fps ceiling gone).
- **Robot deploy gap analysis:** [`PLAN_ROBOT_DEPLOY_01.md`](PLAN_ROBOT_DEPLOY_01.md) — wheel
  referee cinemático + health velocity governor ("intensidad") + depth cliff safety; order:
  Gazebo closed-loop (§1+2+3 together) → cliff → real robot. **Sim = GAZEBO (user decision;
  Isaac starves the 8 GB GPU — PLAN_ISAACSIM_01 shelved).**
- GOTCHAS today: ROS shell shadows the wheel's libcuvslam.so (use the .sh wrapper / RPATH);
  cmake picks CUDA 13.1 (force 12.6); timeout-killed launches leave the provider node alive →
  duplicate publisher = fake teleports (SIGNATURE: duplicated stamps in provider.tum);
  `pgrep/pkill -f` self-match (bracket the first char); Float32MultiArray timestamps lose
  256 s resolution at epoch scale.

---

## 0a. 2026-07-02 — MASTER PLAN v3 + THE FULL-BAG REGRESSION BATTERY (T1)

**The user restated his immortal-framework vision + asked for a total root refactor if needed.
Verdict (code audit): the vision IS the built architecture; consolidate, don't rewrite.** Plan:
[`PLAN_IMMORTAL_FRAMEWORK_01.md`](PLAN_IMMORTAL_FRAMEWORK_01.md) (T1 battery → T2 gates
default-ON → T3 decomposition [delete ~6.9k LOC own-VIO + carve the 3,087-line god node, 0 gtests]
→ T4 BNO055 compass → T5 soft-edge navigation). Memory `slamko-immortal-framework-v3`.

**T1 SHIPPED + first full run done:** `scripts/battery.sh` (one command, serial, all 10 real
bags with per-bag correct configs, GATES=on|off) + `scripts/battery_report.py` →
[`battery/BATTERY_t1_full_on.md`](battery/BATTERY_t1_full_on.md). **Result: 55 PASS / 7 FAIL,
0 crashes in all 10 bags; blackouts recover 100% (~0.8s re-anchor, IMU-referee recall 1.0);
never-lie ✅ everywhere.** The FAILs = 3 real findings:
1. **never-jump residual (T2 blocker):** with `gate_live_pose` @2.5 m/s the live pose still
   steps 0.139 m/tick (6.25 m/s) on flash + 0.065 m (3.87 m/s) on brutal1 — both at the FINAL
   loop-closure correction: the map→odom slew leg exceeds its 0.5 m/s bound per published tick.
   Fix = absolute per-published-step clamp at the publish path (one place, covers both legs).
2. **wall stays the hardest:** provider dead-reckons ~20 km "tracking ok"; referee seals only
   184/1214 lies; final loss never re-localizes (bag ends 4 s after return). Seal-on-doubt must
   be more aggressive on the blank-wall regime (feature-count channel).
3. **brutal1 welds back only 3 of 8 breaks** → 8 honest islands (the known VIEWPOINT recall
   ceiling; the pending multi-direction bags attack exactly this).

**GOTCHAS (hard-won today):** the harness KILLS background bash at ~300 s — `setsid nohup ... &
disown` from a foreground call for any long run, and CHECK FOR ORPHANED orchestrators after a
harness kill (two half-dead battery.sh were alive concurrently = the mutual-reap hazard).
`set -u` before `source /opt/ros/jazzy/setup.bash` dies silently — source first. Durable eval
venv: `~/.venvs/slamko-eval` (numpy+rosbags; /tmp venvs get wiped). suave bag = 49.5 s (157 s
row @rate 0.5 is a COMPLETE run, not truncated).

---

## 0b. 2026-06-30 — IMMORTAL GATES + DEPTH GEOMETRIC LOOP + DYNAMIC LOCAL COSTMAP + D455 CLEAN-MAP

**Commit `220c130` on `klt-fork-loopclosure`. All opt-in or validated; 4 packages build green.** This
session pushed never-lost enforcement + the Nav2 costmap foundation + D455 map quality. Full detail:
`slamko_ros/docs/STATUS.md` (3 dated entries) + the 4 docs below + memories `slamko-immortal-seal-on-doubt`,
`slamko-depth-odom-loop-closure`.

**What shipped (all in `provider_fusion_node.cpp` + `slamko_tsdf` + `slamko_loop` + `slamko_core`):**
- **A — live IMU referee** (`imu_referee`, default OFF): per-step accel-vs-gravity teleport-lie gate,
  gravity auto-estimated (EMA→8.9). The slamko_eval ch3 referee, brought LIVE → seal+break. Recall 0/5→0.67.
- **B — HOLD state** (`hold_on_loss`, default OFF): while tracking LOST, withhold ALL map growth (don't
  map the dead-reckoned stretch); fresh island on recovery. The user's "seal-on-doubt, don't connect".
- **Depth geometric loop weld** (`depth_loop_refine`, default OFF, Phase-1 of PLAN_DEPTH_ODOM_01): at the
  XFeat loop weld, snap the live depth cloud onto the nvblox ESDF (point-to-SDF ICP, `sdf_registration.hpp`
  ALREADY existed) from the XFeat coarse prior; gate inliers+RMS+Zhang-condition; refined edge else fall
  back. VALIDATED `LOOP CLOSED [DEPTH-REFINED]` 7.2cm rms. **Reject was honest** at the corridor (cond<0.02
  = along-axis degenerate). Accept on RMS+inliers NOT the strict `converged` flag; `max_rms=0.15` (~3 ESDF
  voxels = nvblox getVoxels nearest-voxel quantization floor).
- **Nav2 LOCAL costmap on a fixed-rate timer** (`local_costmap_rate_hz`, default 10) — decoupled from kf.
- **DYNAMIC LOCAL costmap** (`local_dynamic`, default **ON**): a 2nd DECAYING nvblox mapper integrated
  **per-frame @ live pose (~45 Hz)** in `onDepth`, decay-to-free + `clearOutsideRadius` window each tick
  (nvblox static+dynamic split). Reactive; uncorrected-pose drift never accumulates. Validated live
  (64×88 window, free/occ/unknown, no GPU starvation alongside OKVIS+TRT+global).
- **D455 clean-map fixes** (RESEARCH_D455_CLEAN_MAP_01): `volumetric_max_range_m` default 5→**3.5 m**
  (far stereo depth = #1 wall-thickener) + `costmap_noise_min_neighbors`=3 (2D speckle filter; −466
  isolated cells, walls intact).

**VALIDATED (casa bag, the ONLY D455-HW-depth bag):** coherence vs **OKVIS-full-SLAM as GT** (run
`rsD455_map848` do_loop_closures=true) — slamko depth-refined ATE **7.5 cm median / 18 cm RMSE, scale 1.02**.
Global costmap topic OK (`225×328 @5cm, latched, slamko_map`). Full immortal run: break-into-islands at the
corridor → weld onto submap 0 at the return (9 components → 4 fused).

**⇒ NEXT STEP (user-chosen): ISAAC SIM** *(superseded 2026-07-09 → GAZEBO, Isaac starves the 8 GB GPU; see §0 + PLAN_ROBOT_DEPLOY_01)*. Bags CANNOT close the Nav2 loop (passive replay) and we have
only 1 D455-HW-depth bag (stereo→depth bags too low quality). So Nav2 driving + brutal stress + map
iteration must go to **simulation or the real robot**. The user chose **Isaac Sim** (natural fit:
pairs with nvblox / Isaac ROS, GPU). See **`docs/PLAN_ISAACSIM_01.md`** (the next-session entry point).

**OPEN LEVERS (cheap, high-value, not done):** (1) sensor-error **`1/z²` integration weight** in nvblox
(thins walls; `nvblox_backend` — currently constant weight); (2) **depth pre-filter** (decimation/spatial/
temporal in disparity domain) before integrate; (3) fix `capture_costmaps.py` global QoS (it MISSES the
latched global — use a transient_local subscriber that waits, see `/tmp/glob_cap.py`); (4) per-axis
degeneracy-aware covariance on the depth loop edge (today isotropic); (5) **push** to remote.

**GOTCHAS for the next session:** the harness reports background bash "failed exit 1" even when a run
COMPLETES — check the output files, not the exit code. OKVIS is GPU-NONDETERMINISTIC (each run drifts
differently: 0.83 vs 7.97 m start-end on the same bag) → A/B map quality is confounded; use OKVIS-full-SLAM
as GT or deterministic offline A/B. The casa bag is `/mnt/data/d455_bags/casa_084815_flashbno_trim`; run
scripts in `scratchpad/run_full_immortal.sh` (all gates+depth-loop+dynamic-local) and
`scripts/run_slamko_casa_volumetric_live.sh`. Python with open3d/plotly = venv `/tmp/depthvenv`; rerun
viewer = `/tmp/rrviewer`.

---

## 0c. 2026-06-26 — UNIVERSAL EVALUATOR + NEVER-JUMP GATE + cuVSLAM provider

The session that built the **measurement system** and used it to find + fix a real defect. Full
detail: [`EVAL_SYSTEM_01.md`](EVAL_SYSTEM_01.md), `slamko_ros/docs/STATUS.md` (2026-06-26),
memory `slamko-universal-evaluator`.

**Why it matters (user's reframe, verbatim):** *"tiene haber una sistema universal de evaluación que
entienda bien y no deje despistarse."* The provider is untrusted/disposable BY DESIGN (HR#4), so the
EVALUATION must be provider-agnostic too — else every provider quirk (cuVSLAM `camera_info`, OKVIS
fps) derails the work into plumbing. **Don't pivot providers to dodge a blocker — the rumbo is
cuVSLAM (OKVIS can't hold 45 fps); MEASURE the slamko LAYER instead.**

**What got built + validated:**
- **`scripts/slamko_eval.py` — the universal 7-channel ideology scorecard** over ANY run dir
  (provider.tum, graph.tum, fusion.log, map/). 1 never-lose · 2 never-jump · 3 distrust(IMU) ·
  4 never-lie · 5 recover · 6 stable-frag · 7 geometric. Three INDEPENDENT physical witnesses, none
  trusting the provider: **IMU** (inertial), **depth→SDF** (geometric), **XFeat** (recognition).
  Channel 3 IMU referee (per-jump, attitude-free): a real >3 m/s motion has `|accel|−g` elevated; a
  TELEPORT moves position while the accel reads plain gravity → REAL vs LIE. **VALIDATED: across
  cuVSLAM AND OKVIS casa runs, 100% of inertially-impossible provider teleports fell inside a slamko
  LOST→RECOVERED sealed interval** (cuVSLAM `vol` diverged to 374 m / 102 teleports → all 102 sealed).
- **NEVER-JUMP defect FOUND then FIXED.** The new live slewed-TF dump (`traj_slewed.tum` = the pose
  Nav2 consumes) exposed channel 2 **FAIL**: the live robot pose jumped **17.5 m/s**, identical to
  the provider's teleports. Root cause: the slew only rate-limits the `map→odom` correction leg; the
  provider's teleport rides through the `odom→base` passthrough RAW. **Fix `gate_live_pose`** (opt-in,
  default OFF): absorb the teleport into a single odom-frame accumulator `T_gate_` so the live
  `odom→base` HOLDS (dead-reckons smooth) and re-anchors via the slew. A/B: **15 jumps/17.5 m/s →
  0 jumps/2.19 m/s PASS**; geometric coherence IMPROVED (excess 0.058→0.028 m); stability unchanged.
  Localized to the publish path — the optimized graph was ALREADY smooth (max 1 m node steps).
- **cuVSLAM provider:** runs the rumbo on casa direct-infra (full slamko run, 6/7 PASS). It IS GOOD
  (EuRoC 0 jumps); on uncalibrated casa it teleports but **slamko catches every teleport** — the whole
  point of the untrusted-provider design, now MEASURED.
- **`scripts/tsdf_slice.py`** — horizontal mid-height cut of a TSDF mesh (floor-plan view) + trajectory
  overlay; auto-cuts at the navigable (trajectory) height.
- **GTSAM / iSAM2 pose-graph backend = the MASTER_PLAN P-C′, SHIPPED + now the LIVE DEFAULT**
  (commits eb13a06→e9da7c4). `PoseGraphBackend::{Ceres, GtsamLM, GtsamISAM2}` behind the same
  `slamko_loop::PoseGraph` contract; `provider_fusion_node` defaults `pose_graph_backend=isam2`;
  GTSAM auto-detected in CMake (ON when installed, Ceres-only when absent). The compass yaw-prior is
  a **native GTSAM factor** (no Ceres fall-back). VALIDATED: EuRoC MH_03 ATE Ceres 19.10 ≈ GtsamLM
  19.14 ≈ iSAM2 19.19 mm (all = baseline OKVIS 22.48 improved); **iSAM2 6× faster LIVE** (O(touched)
  Bayes-tree vs Ceres O(graph) re-solve — the lifelong payoff); brutal-bag default validated (9 Atlas
  breaks → 0 batch-rebuild fallbacks, coherent map). Scope: GTSAM is ONLY the pose-graph solver —
  the nvblox volumetric TSDF, the provider, XFeat reloc are unchanged. Detail:
  `slamko_loop/docs/STATUS.md` + `slamko_ros/docs/STATUS.md` + memory `slamko-gtsam-smoother-inaccurate`.

**The "travesuras" (small things that bit — check here first):**
- **cuVSLAM rejects a dim-mismatched `camera_info`** (640 img vs 848 info) — inject the correct dims
  (`cam_info_inject_{640,848}.py`). Use a dimension-consistent bag.
- **RESOLVED (was misdiagnosed as a 'body_tf' bug):** the cuVSLAM inject-variant (wall/brutal)
  gave provider_fusion 0 odometry NOT because of body_tf (verified: cuVSLAM→body_tf both publish at
  29.93 Hz, TF resolves) but because **provider_fusion crashed at startup, exit 127
  `libnvblox_lib.so: cannot open`** — since the S3 fix it links nvblox and needs the lib on the
  loader path even with volumetric OFF. The inject run scripts lacked the `LD_LIBRARY_PATH` export
  the direct script had. One-line fix (commit 3f8bb65). **CASA1_wall now runs the cuVSLAM rumbo
  end-to-end** (776 poses, 27 submaps, never-lost fired: quality-break 11/11, IMU-shock 16; provider
  diverged 38 km on the jolts yet slamko stayed coherent — the immortal thesis on the brutal bag).
- **never-jump:** the `map→odom` slew is NOT sufficient on its own — the provider teleport rides
  `odom→base`; you need `gate_live_pose`. Threshold = **per-platform robot-max-speed + margin**
  (5.0 m/s too lax for casa, 2.5 worked; the IMU referee tells you what's real fast-motion vs teleport).
- **tsdf_slice:** ALWAYS cut at the NAVIGABLE (trajectory) height. A slice above/below the camera
  height makes the path project onto walls that exist only at the other height → looks like
  "trajectory inside a wall" but is a pure projection artefact (verified: 73% of the path >0.3 m clear,
  median 0.58 m → map coherent).
- **flash-bag splitter:** the pre-bag `n=0 / clean=0 / dotted=0` lines are WARMUP, not failure — it
  routes by IMAGE CONTENT (dot energy), not the buggy `frame_emitter_mode` metadata; final run routed
  3338 emitter-ON frames → `/nvblox/depth` (the D455 HW on-ASIC depth) → TSDF.
- **channel 7 honest dead-end:** neither the sparse XFeat cloud (~5 cm density floor MASKS doubling)
  NOR the dense TSDF mesh (the TSDF AVERAGES conflicting depth into one smoothed surface) can detect
  doubling OFFLINE — proven twice with synthetic-doubling injection. True doubling detection needs the
  LIVE point-to-SDF residual of a held-out revisit depth frame (the v2 hook).
- **tooling:** `rosbags` lives in the venv `/tmp/rerunvenv` (NOT system python — PEP668); the harness
  reports a background bash "failed exit 1" even when the run COMPLETED (check the output files, not
  the exit code); don't prefix a background bash with `pkill` (the harness reaps the group).

**Artifacts / "photos" for next sessions** (regenerate, don't re-derive):
- TSDF mid-height floor plan of the 45 fps flash bag: `python3 scripts/tsdf_slice.py
  /tmp/slamko_casa_vol/volumetric_live.ply out.png 0.25` (auto navigable height) — the deliverable
  that proved the map coherent.
- Full scorecard (with IMU referee): `/tmp/rerunvenv/bin/python3 scripts/slamko_eval.py <run_dir>
  --bag /mnt/data/bags/bno_ab/CASA1_40cmH_Stereo60_RGB30_BNO_848_trim`.
- Run the validated volumetric pipeline: `bash scripts/run_slamko_casa_volumetric_live.sh` (OKVIS +
  D455 HW depth → live nvblox TSDF → mesh + `~/volumetric_costmap` OccupancyGrid).
- Gated cuVSLAM A/B: `GATE=true GATESPD=2.5 bash scripts/run_slamko_cuvslam_casa.sh`.

**NEXT (proposed):** Nav2 integration — the nvblox local costmap is ALREADY published
(`~/volumetric_costmap`, nav_msgs/OccupancyGrid); wire it + the slamko global map into Nav2, gate
lifecycle on `localized`, drive goals. The never-jump gate is the PREREQUISITE that just landed (a
planner on a jumping TF would corrupt). See §"Next step" at the end of this doc.

---

## 0d. 2026-06-19 — THE IMMORTALITY PUSH (prior milestone)

A long session took the map from "grows without bound" to **ORB-SLAM3-style bounded +
never-lost + gated**, all measured on casa1 bags. The immortal core is now in place; what
remains is scope-expansion (GPS, out-of-core, real-robot stress). Detail in
`slamko_ros/docs/STATUS.md` (11 dated entries) + the memories below.

**What got built + validated (all in `provider_fusion_node.cpp`):**
- **Map bounded by AREA, not by visits** (the headline). ORB-SLAM data-association adapted to
  poses-fixed submaps: per-KF voxel dedup → **per-landmark cross-submap cull** (a known point is
  dropped, only NEW points kept) → **drift-tolerant 0.15 m occupancy voxel** → **viewpoint-aware**
  (cull same-view revisits, KEEP new-direction ones as omni-directional reloc anchors). Result:
  revisiting a house plateaus (~1.5 submaps/visit residual, was 3.5; landmark growth +3%/visit,
  was +100%). 10 vs 1000 visits converge to the same bounded map (finite house voxels).
- **R0 "never ingest garbage" gates:** (a) seal-quality — a submap sealed during a TRUE stale-gap
  loss is kept in the map but BARRED as a reloc target (no garbage match source → no I2 corruption);
  (b) DR-informed — bar only when the gyro DR disagrees with OKVIS's across-gap motion >15°
  (small d_rot = OKVIS bridged fine = trustworthy). Clean bag: loops intact (6).
- **Never-lost:** branch supervisor (stale-gap → seal+branch+soft edge) + DR-gate instrument
  (gyro-vs-OKVIS; OKVIS IMU-bridges ≤6 s, d_rot ≤4°, proven sound) + cross-session re-anchor.
- **I2 never-false-merge: VALIDATED** across 162 hard welds (max 1.01 m, 0 teleports) —
  `scripts/audit_i2.py`. PCM 3-vote + 30 m teleport gate + VPR cliff.
- **Compass instrument:** field-norm-gated raw-mag heading (`/bno055/mag`), 96% in-band on casa1.
- **RECALL ROOT-CAUSE REFRAME (the big finding):** the cosine "cliff" is a **VIEWPOINT** artifact,
  NOT descriptor quality. Same-heading revisits already match (EigenPlaces floor 0.426, 97% OK);
  the low tail is OPPOSITE-facing pairs (no overlap → unmatchable by ANY model/dense-matcher,
  correctly). Offline A/B (`scripts/vpr_ab_casa.py`, `vpr_dense_rescue.py`, venv `/tmp/vprvenv`):
  EigenPlaces beats SALAD/CosPlace on aggregate; LoFTR rescued 0/15 blind spots (no overlap).
  **CANCELLED two planned C++ builds** (EigenPlaces→SALAD swap; LoFTR cascade) — the fix is
  viewpoint COVERAGE (shipped: viewpoint-aware cull), not a better matcher.

**New offline tools (run under the venv):** `vpr_ab_casa.py` (model A/B), `vpr_dense_rescue.py`
(LoFTR rescue + heading split), `audit_i2.py` (false-merge audit), `lifelong_visits.sh` (N-visit
growth), `plot_{dr_gate,immortal,reloc_funnel,dense}.py`. Venv: `python3 -m venv --system-site-packages
/tmp/vprvenv && /tmp/vprvenv/bin/pip install torchvision kornia pytorch_lightning pytorch_metric_learning`
(system Python is PEP668-managed — NEVER pip into it; ROS depends on it).

**Immortality scorecard:** never-lost ✅ · map-bounded ✅ · never-garbage ✅ (R0 gates) ·
never-false-merge ✅ (162 welds) · recall ✅ (solved + understood: viewpoint coverage). Remaining =
scope-expansion only: GPS/compass-yaw factor 🟡 · out-of-core map (RAM-bound today) 🔴 ·
real-robot / extreme-stress bags 🟡.

---

## 1. What slamko IS now (the pivot, operational)

slamko = **lifelong map + multi-session relocalization + loose-fusion layer over
an EXTERNAL odometry provider** (OKVIS2-X default; klt_vo = validated 2nd
provider). slamko does NOT implement odometry. One node — `slamko_ros/
provider_fusion_node` — does the whole live pipeline:

```
provider /odometry ─► ProviderChain (decimate to KFs, motion-prop covariance)
                       │
   D455 IR stereo ─► XFeat detect (L+R, same hw frame) ─► stereo triangulate ─┐
                       │                                                        │
                       ├─► EigenPlaces per KF (VPR descriptor)                 │
                       ▼                                                        ▼
                  PoseGraph (Ceres SE3)  ◄──── loop/anchor edges ◄──── relocalize:
                       │                          (PCM consensus gate)   EigenPlaces top-10
                       ▼                                                  → XFeat/LighterGlue
                  map→odom slewed TF                                      → PnP verify
                  + sealed .smap submaps (anchors refreshed on optimize)
                  + graph.tum (optimized trajectory — the honest output)
```

Two relocalizers run in parallel: **session** (in-session loops, XFeat-NN
verify) and **prior** (cross-session re-anchor, LighterGlue verify — needed
because XFeat-NN can't match across different walks). Both feed ONE
`LoopConsensusGate` (slamko_core, 8 unit tests).

---

## 2. Phase status (MASTER_PLAN §8)

| Phase | What | State | Headline number (real casa bags, clean data) |
|---|---|---|---|
| **P-A** | OKVIS adapter → relative KF edges + cov → loose fuser → map→odom | ✅ | fused tracks provider to 0.000000 m (no global constraints) |
| **P-B** | Reloc recall: in-session loops + cross-session + cross-bag fusion | ✅ | Suave 4.2 cm · Escaleras 8.1 cm (slamko *improves* the 10.1 cm provider, 9 loops) · fusion LOCALIZED kf 3, 4 cm, 0 held jumps |
| **P-C** | Never-lost (blackout) — emerges from continuous reloc+consensus | 🟢 first pass | `CASA1_Suave_blackout` 3.12→0.16 m · `blackout4` 3.69→0.09 m, 0 crashes |
| **P-C′** | Anchor-edges in graph + iSAM2 poses-only + seal/branch state machine | ⬜ next | — |
| **P-D/E/F** | GNSS / extra providers / semantics | P-E started | klt_vo provider: Suave 0.080 m, Escaleras 0.070 m (xfeat config) |

---

## 3. Canonical commands (THE way to run, post-root-cause)

**Build:**
```bash
cd ~/coding/slamko && source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release        # core/loop/ros
# loop with the cross-bag LighterGlue verifier (REQUIRED for cross-session):
colcon build --packages-select slamko_loop --cmake-args -DCMAKE_BUILD_TYPE=Release -DSLAMKO_LOOP_WITH_TORCH=ON
```

**Map a bag (two-pass, ZERO GPU contention — the production recipe):**
```bash
# pass 1 records OKVIS odometry alone; pass 2 fuses offline (OKVIS off)
bash scripts/map_two_pass.sh /mnt/data/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO results/pc/<out> 1.0 1.0
# -> <out>/map/*.smap, <out>/graph.tum (optimized), <out>/provider.tum
```

**Fuse a new session onto a prior map (cross-session):**
```bash
PRIOR_MAP=$PWD/results/pc/twopass_esc4/map \
  bash scripts/map_pass2.sh /mnt/data/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO results/pc/fusion_final 1.0
# (needs <out>/odom_bag/ recorded by a pass-1 first; cp it in if reusing)
```

**Live (single-pass, accepts GPU contention) + cross-session gate:**
```bash
VPR=true PRIOR_MAP=<prior/map> MAX_WAIT=300 \
  scripts/bench_pa.sh <bag> results/<out> 0.5     # rate 0.5 to bound contention
PROVIDER=kltvo VPR=true scripts/bench_pa.sh <bag> results/<out> 0.5   # klt_vo provider
```

**Evaluate (HONEST — always graph.tum + Umeyama scale):**
```bash
# Sim3 ATE with the SCALE factor printed (scale != 1 -> calib/IMU bug, not drift)
# reference TUMs: ~/coding/klt_vo/results/d455/okvis_{Suave,Escaleras}.tum
# inspect map: ./install/slamko_loop/lib/slamko_loop/{smap_info,smap_cloud} <map_dir>
```

---

## 4. LOAD-BEARING gotchas (learned the hard way 2026-06-12/13)

1. **OKVIS calib config MUST match bag resolution.** The bno_ab CASA1 bags are
   **640×480**; `rsD455_odom848` (848, fx=426) inflated scale ~1.7× and looked
   like drift for a whole day. Correct = **`rsD455_map_odom`** (640, camera IMU,
   loops OFF). `rsD455_bno` is the BNO055 *external* IMU — diverges with the
   camera IMU. Verify `grep image_dimension <cfg>/okvis2.yaml` vs the bag first.
2. **The bno_ab bags' /camera/camera/imu accel is DOUBLED** (unite_imu_method:=2).
   Use the PROVEN launch `~/coding/BNO055/ab/okvis_ab_c1_d455imu.launch.py`
   (imu_relay.py --accel-scale 0.5 → /okvis/imu0, OKVIS_CFG env, 80 Hz odom).
   Suave survives 2× by luck; Escaleras diverges (z→km → `std::bad_alloc`).
   `map_two_pass.sh` pass 1 uses this launch.
3. **Evaluate on `graph.tum`, never `fused.tum`.** fused.tum is the causal online
   trail (raw provider + step at each loop); graph.tum is the optimized output.
   A cm-level start-end *closure* hides a 0.4–1.5 m shape error — Hard Rule #5.
4. **Our TRT inference contends with OKVIS for the GPU.** Online at rate 1.0
   degrades the provider; map from bags at rate ≤0.5 OR two-pass (zero
   contention). Online robot needs a GPU budget (reloc throttle
   `min_reloc_period_s=0.5` shipped; INT8 / klt_vo-190fps / Orin later).
5. **Cross-session needs LighterGlue** (`-DSLAMKO_LOOP_WITH_TORCH=ON`,
   libtorch at ~/libtorch). XFeat-NN silently falls back and never matches
   across walks — VPR retrieval is fine (cos 0.71), the *verify* is the gap.
6. **Check `nvidia-smi --query-compute-apps` + pgrep before blaming code** —
   klt_vo sprint benches rotate on this machine and steal the GPU.
7. **Two independent `ros2 bag play` skew by the pass-1 pre-roll.** Pass-2 odom
   is replayed IMAGE-DRIVEN (`scripts/odom_player.py`).

---

## 5. Key files

| Purpose | Path |
|---|---|
| The fuser node (whole live pipeline) | `slamko_ros/nodes/provider_fusion_node.cpp` |
| Provider contract + chain + covariance | `slamko_core/include/slamko_core/odometry_provider.hpp` |
| Consensus gate (PCM-lite) + 8 tests | `slamko_core/include/slamko_core/loop_consensus.hpp` · `test/test_loop_consensus.cpp` |
| Pose-graph (Ceres SE3) | `slamko_loop/{include,src}/.../pose_graph.{hpp,cpp}` |
| Relocalizer (VPR top-10 + PnP + LighterGlue) | `slamko_loop/.../xfeat_relocalizer.{hpp,cpp}` |
| Submap I/O (SMP5) | `slamko_core/include/slamko_core/submap_io.hpp` |
| Two-pass mapping | `scripts/map_two_pass.sh` · `scripts/map_pass2.sh` · `scripts/odom_player.py` |
| Live bench + provider switch | `scripts/bench_pa.sh` (PROVIDER=okvis\|kltvo, VPR, PRIOR_MAP env) |
| VPR recall diagnostic | `slamko_loop/tools/vpr_recall_diag.cpp` (--map2 = cross-map) |
| Map inspect / cloud export | `slamko_loop/tools/{smap_info,smap_cloud}.cpp` |
| OKVIS launch (640 + accel relay) | `~/coding/BNO055/ab/okvis_ab_c1_d455imu.launch.py` |
| Reference trajectories | `~/coding/klt_vo/results/d455/okvis_{Suave,Escaleras}.tum` |
| Best clean artifacts | `results/pc/twopass_esc4/` (Escaleras map) · `results/pc/fusion_final/` (fusion) |

---

## 6. The queue (next session, in order — refreshed 2026-06-26)

### Recommended next: NAV2 + nvblox local costmap (the "make it drive" phase)

slamko now has both maps coherent + the never-jump gate (the planner prerequisite). **The two
costmaps are SHIPPED** (2026-06-26, commits 7cdc37d/378116f): `~/volumetric_costmap` = GLOBAL (whole
TSDF occupancy slice, latched) + `~/local_costmap` = LOCAL (rolling 4 m window of the live nvblox
slice, re-centred on the robot each tick). Both validated live (`scripts/capture_costmaps.py`).
Default backend is iSAM2; `gate_live_pose` default ON. The remaining work is **Nav2 wiring**:
1. **TF/map contract**: slamko emits `slamko_map→slamko_odom→slamko_base`; remap to Nav2's
   `map→odom→base_link`, and expose the GLOBAL costmap as `/map` (or point Nav2's global_costmap
   static_layer at `~/volumetric_costmap`). Capture the GLOBAL at END-of-run (it grows; mid-run = one
   room only — the "solo salón" gotcha).
2. **gate_live_pose stays ON** (default) — Nav2 must consume the SMOOTH TF; set `live_gate_speed` to
   platform max + margin.
3. **Wire `~/local_costmap`** into Nav2's `local_costmap` (already an OccupancyGrid). Refinements: use
   the ESDF distance (reactive gradient) or the `nvblox_nav2/nvblox_costmap_layer` plugin (needs the
   ESDF exposed — today behind the PIMPL); publish the local at 30 Hz (re-crop a cached slice).
4. **Gate Nav2 lifecycle on `localized`** (the bridge pattern from `okvis_nav2_bridge`) so goals
   aren't accepted against the startup frame before the first reloc; `/initialpose` cold-start
   override.
5. **Send `/goal_pose`** in sim/replay; confirm the global planner uses slamko's `/map` and the
   local controller respects the nvblox obstacles; spawn a dynamic obstacle → confirm reroute.

Open questions to resolve while doing it: is the global `/map` the sparse-occupancy or the TSDF
slice? does nvblox need its own TF (`odom→base` from slamko + `base→depth_optical` from a static
URDF)? local-only nvblox first (no static prior). Trade-off: this is the first time slamko output
drives a controller — start in **bag-replay → sim (`cerebro_robot_sim`) → real D455**, never
real-robot first. **Write this up as `PLAN_NAV2_NVBLOX_01.md` when starting.**

### Also queued (smaller, can interleave)
- **Unify the gate threshold with the quality-break** so channel 2 (live output) and channel 3
  (map-seal recall) move together (today the gate at 2.5 m/s caught more than the quality-break at
  4.0). Feed a gate-detected teleport into the seal path.
- ~~Fix the cuVSLAM inject-variant body_tf 0-odometry bug~~ **DONE (commit 3f8bb65)** — it was a
  missing `LD_LIBRARY_PATH` (provider_fusion exit-127 on libnvblox), not body_tf; wall/brutal now run.
- **Channel 7 v2 — live point-to-SDF revisit residual** (the only metric that resolves
  sub-decimetre doubling; sparse + dense-mesh both proven blind offline).

### Prior queued task: RECORD NEW BRUTAL STRESS BAGS (still valid)

**NEXT TASK (user-chosen): RECORD NEW BRUTAL STRESS BAGS.** The immortal core is built
+ validated on casa1, but the EXTREME failure modes can't be stressed without data we
don't have (the c2/c3 dirs are output folders, NOT rosbags; only CASA1_* are valid bags).
Record bags that hit the modes we've only theorized:
- **IMU-blackout** (cover lens AND a hard bump = visual+IMU loss together = the true
  unobservable interval — does OKVIS finally go fully stale? does the DR-gate bar it?).
- **Wall-pointing / degraded** (long stretch facing a featureless wall = covariance-marginal
  → exercises the degraded policy + the seal-quality gate).
- **A genuinely DIFFERENT place** (another house/floor) → the strict I2 false-merge test
  (prior=casa1, query=other → expect 0 LOCALIZED).
- **Multi-DIRECTION revisit** (traverse a place, leave, return facing the OPPOSITE way) →
  the only way to demonstrate the viewpoint-aware cull's positive recall gain (shipped but
  unproven for lack of this bag).
- **Rough-terrain / fast / flip** (the user's drone-flip / rocky-car scenario) → stress the
  branch supervisor + soft edges under real tracking loss.
Then re-run `bench_pa.sh` + `lifelong_visits.sh` + `audit_i2.py` on them.

**Then, the remaining immortal scope-expansion:**
1. **GPS/compass yaw factor in the pose-graph** (P-D) — compass is instrumented (field-norm
   gate); add the unary yaw factor + Kok-Schön ellipsoid calib + (outdoor) GNSS anchor.
2. **Out-of-core map** (lifelong at city/years scale) — page submaps to disk, bounded
   Working Memory + LTM (RTAB-Map pattern). Today the whole map + reloc DB is in RAM.
3. **Multi-prior Atlas** — load N prior maps, localize in any, bridge A↔B (the full A/B/A).
4. **Anchor-edges in the pose-graph** (P-C′) — re-anchors as graph edges so corrections
   distribute through the trajectory (kills mid-run drift between re-anchors).
5. **`/slamko/health` topic + rviz panel** — the permanent Good/Marginal/Lost monitor.
7. **iSAM2 poses-only** swap for the global graph (P-C′, real-time incremental).
