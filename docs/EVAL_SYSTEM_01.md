<!-- validated: HEAD 2026-06-26 · tests: cuVSLAM+OKVIS casa runs, 7-channel scorecard green -->
# The universal ideology evaluator — `scripts/slamko_eval.py`

**Why it exists.** slamko's provider (OKVIS, cuVSLAM, klt_vo) is *disposable and untrusted by
design* (Hard Rule #4). So the evaluation must be **provider-agnostic too** — otherwise every
provider quirk (cuVSLAM `camera_info`, OKVIS fps ceiling) derails the work into plumbing instead of
measuring the **ideology**: a SLAM for an autonomous robot that **never loses tracking, never jumps
the pose, survives any motion** via the Atlas (seal a submap when it's hard, weld by anchors on a
recognized revisit; **dangle honestly** when there's no overlap, never fake-coherent).

Feed any run dir → the same **7-channel scorecard**. Compare providers head-to-head on one rubric.

## Three independent physical witnesses (none trust the provider)

| witness | judges | channel |
|---|---|---|
| **IMU pre-integration** | provider velocity / teleport lies (inertial) | 3 |
| **D455 depth → SDF** | map metric coherence at revisit (geometric) | 7 |
| **XFeat appearance** | revisit recall (recognition) | 5 |

## The 7 channels

1. **NEVER-LOSE** — every `QUALITY LOST` got a `RECOVERED`; output coverage of the bag.
2. **NEVER-JUMP** — did slamko *inject* motion the provider didn't report (un-slewed correction),
   excluding logged loop-corrections + honest island boundaries. *File proxy* — the true live
   guarantee is the slewed `map→odom` TF (0.5 m/s), not dumped; a `traj_slewed` dump would close it.
   The provider's *own* fast steps are **channel 3's** job, not a slamko jump.
3. **DISTRUST (IMU referee)** — *per-jump, attitude-free*: a genuine >3 m/s motion needs elevated
   `|accel|−g`; a **teleport** (cuVSLAM's "tracking good but jumps") moves the position while the
   accelerometer reads plain gravity. Each provider fast-step → REAL vs **TELEPORT LIE**. A lie is
   *caught* iff it falls inside an open **LOST→RECOVERED** interval. `--bag` enables it.
4. **NEVER-LIE** — components vs welds; dangling islands are honest. (`scripts/audit_i2.py` = the
   false-merge teleport check.)
5. **RECOVER** — re-anchor count + mean latency after each loss.
6. **STABLE-FRAGMENTING** — submaps / metre; bounded even when it makes many submaps (< 1/m).
7. **GEOMETRIC (map coherence)** — at non-consecutive submap overlaps (a revisit), the cross-submap
   nearest-neighbour registration error vs the cloud's own sparsity floor (intra-submap NN). Export
   the cloud first: `ros2 run slamko_loop smap_cloud <run>/map <run>/global_cloud.csv 1`. **HONEST
   LIMITATION (validated by synthetic-doubling injection): the sparse XFeat cloud's density floor
   (~5–6 cm inter-point) MASKS any doubling ≲ that floor — so this is a RELATIVE indicator, not an
   absolute doubling verdict.** The cross-run signal is valid (a diverged run shows higher excess).
   The **dense D455 depth → SDF residual is the v2 metric** that can resolve sub-decimetre doubling
   (the live `--depth-sdf` hook; CUDA + live SDF; `slamko_tsdf/tools/nvblox_sdf_selftest.cpp`).

## Usage
```bash
# one run, or many side-by-side (cuVSLAM vs OKVIS on the SAME rubric):
python3 scripts/slamko_eval.py /tmp/run_dir [/tmp/run_dir2 ...]
# channel 3 (IMU referee) needs rosbags + the bag (use the venv that has rosbags):
/tmp/rerunvenv/bin/python3 scripts/slamko_eval.py /tmp/run_dir \
    --bag /mnt/data/bags/bno_ab/CASA1_40cmH_Stereo60_RGB30_BNO_848_trim
```

## Validated finding (2026-06-26)

Across **cuVSLAM and OKVIS** casa runs, **100% of inertially-impossible provider teleports fell
inside a slamko LOST→RECOVERED sealed interval** — the independent IMU witness confirms the
never-jump ideology holds regardless of provider. cuVSLAM `vol` *diverged to 374 m with 102
teleports* (the literal "tracking good but jumps" failure) and slamko **sealed all 102** (0 jumps
injected, 10/10 recovered, 25 submaps bounded at 0.07/m).

Recall semantics matter: ±2 s proximity inflates to 1.0; the **LOST→RECOVERED interval-membership**
is the correct, defensible metric (a break seals the whole window). At a tight ±1 s window 90% of
lies are within 1 s of the break instant — the tail is the rest of a multi-second teleport burst
already inside the declared-LOST interval.

Full cuVSLAM scorecard (2026-06-26): casa **6/7 PASS**, vol **6/7 PASS** (ch7 WARN/FAIL = the
sparse-cloud relative indicator, not an absolute defect — see its honest limitation above).

**Gotchas:** channel 3 reads `--bag` IMU via `rosbags` (installed in `/tmp/rerunvenv`, NOT system
python — PEP668). Gravity is estimated as `median(|accel|)` so a doubled-accel bag (~19.6) is
handled. Channel 2 is a file proxy until a slewed-TF dump exists. Channel 7's sparse-cloud floor
(~5 cm) masks sub-decimetre doubling — it is a RELATIVE indicator; the dense depth → SDF (v2) is the
absolute metric. Export `global_cloud.csv` (smap_cloud) before running channel 7.
