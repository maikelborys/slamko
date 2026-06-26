# slamko — Pipeline status & cold-start (2026-06-13, major update 2026-06-19)

<!-- validated: 2026-06-19 · the consolidated "where we are NOW" snapshot of the
loose-fusion pipeline. Chronological detail: slamko_ros/docs/STATUS.md.
Plan: MASTER_PLAN.md §8. Research provenance: docs/REBUILD_PROPOSAL_01.md. -->

**Read this first if you're starting cold.** It is the one-page truth of what
runs today, the exact commands, the load-bearing gotchas, and the queue.
Everything below was validated on the real D455 casa bags.

---

## 0. 2026-06-26 — UNIVERSAL EVALUATOR + NEVER-JUMP GATE + cuVSLAM provider (read this FIRST)

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

## 0b. 2026-06-19 — THE IMMORTALITY PUSH (prior milestone)

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
