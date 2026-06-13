# slamko — Pipeline status & cold-start (2026-06-13)

<!-- validated: 2026-06-13 · the consolidated "where we are NOW" snapshot of the
loose-fusion pipeline. Chronological detail: slamko_ros/docs/STATUS.md.
Plan: MASTER_PLAN.md §8. Research provenance: docs/REBUILD_PROPOSAL_01.md. -->

**Read this first if you're starting cold.** It is the one-page truth of what
runs today, the exact commands, the load-bearing gotchas, and the queue.
Everything below was validated on the real D455 casa bags.

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

## 6. The queue (next session, in order)

1. **Anchor-edges in the pose-graph** — re-anchors as graph edges + refresh
   sealed anchors post-optimize, so corrections distribute through the whole
   trajectory (kills the mid-run drift between re-anchors; the P-C′ headline).
2. **Landmark-cloud overlap merge-verification** — the geometric 4th defense
   layer (after VPR→PnP→consensus): transform a submap's cloud by the accepted
   anchor, reject if it doesn't overlap the target (Bosch reversible-merge;
   the principled answer to "would geometry align better" — no new deps).
3. **Explicit seal/branch state machine** + **`/slamko/health` topic + rviz
   panel** (the permanent monitor: Good/Marginal/Lost, streaks, re-anchors).
4. **provider_chaos.py + stress S2–S4** (provider death/restart, covariance
   storm, rate abuse — `docs/PLAN_STRESS_SUITE.md`).
5. **Multi-prior Atlas** — load N prior maps at once, localize in any, bridge
   A↔B when one session sees both (the user's full A/B/A scenario).
6. **klt_vo hardening** — its stairs z-compression (own repo: bias
   carry-forward + gravity gate); reproducibility median-of-3 on the gates.
7. **iSAM2 poses-only** swap for the global graph (P-C′, real-time incremental).
