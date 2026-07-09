# PLAN_ISAACSIM_01 — Nav2 closed-loop in Isaac Sim (SUPERSEDED → GAZEBO)

> **SUPERSEDED (2026-07-09, user decision):** Isaac Sim starves the 8 GB GPU (memory
> `slamko-gazebo-sim`: 344 OKVIS losses vs 0 in Gazebo). The sim surface is **GAZEBO**
> (`docs/PLAN_GAZEBO_SIM_01.md` if present / cerebro_robot_sim) + D455/EuRoC bags. The
> closed-loop deploy plan is now [`PLAN_ROBOT_DEPLOY_01.md`](PLAN_ROBOT_DEPLOY_01.md).
> Kept for provenance — the Nav2 wiring notes below remain useful for the Gazebo run.

<!-- authored 2026-06-30 · next-session entry point. Cold-start: read docs/PIPELINE_STATUS_01.md §0
(2026-06-30) first, then this. Commit 220c130 on klt-fork-loopclosure. -->

## Why Isaac Sim now (the constraint that forces it)

Bags **cannot** close the Nav2 loop — a bag is passive replay, no control, no reaction to goals. And we
have **only ONE D455-HW-depth bag** (`casa_084815_flashbno_trim`); the stereo→depth bags are too low
quality. So everything that needs a live closed loop or fresh data — **Nav2 driving, brutal stress tests,
map-quality iteration** — must move to **simulation or the real robot**. User chose **Isaac Sim** (NVIDIA):
natural fit because the stack is already GPU/nvblox/Isaac-ROS-adjacent, and Isaac Sim gives a controllable
D455-equivalent sensor + ground truth + repeatable scenes (unlike GPU-nondeterministic bag replay).

## What Isaac Sim unlocks that the bag can't
1. **Nav2 closed loop** — send `/goal_pose`, the global planner uses slamko's `/map` (the
   `~/volumetric_costmap`), the controller uses the reactive `~/local_costmap`, the robot DRIVES, reacts,
   reroutes. The actual "make it drive" milestone.
2. **Ground truth** — Isaac Sim gives exact poses → finally a CLEAN ATE / map-quality A/B (no OKVIS
   nondeterminism confound). Validate the D455 clean-map fixes (range cap, 1/z², filters) rigorously.
3. **Brutal stress on demand** — spawn dynamic obstacles, cover the camera, featureless walls,
   multi-direction revisits, kidnaps — to exercise the immortal gates (A referee, B HOLD, atlas break,
   depth weld) repeatably. No need to physically record bags.
4. **Map-clean iteration** — repeatable scene → change a param → measure the map vs GT. The loop the
   single bag can't give.

## The integration architecture (what feeds what)
```
Isaac Sim (D455-equiv: stereo IR + RGB + IMU + depth, + GT pose)
   │  (ROS 2 bridge / Isaac ROS)
   ├─► /camera/.../infra1,2 + /imu  ─► OKVIS VIO (rsD455_odom848)  ─► /okvis/okvis_odometry
   ├─► /camera/.../depth            ─► slamko provider_fusion (volumetric:=true)
   │                                     ├─ STATIC global TSDF (deformable) ─► ~/volumetric_costmap
   │                                     └─ DYNAMIC local TSDF (decay@45Hz) ─► ~/local_costmap
   └─ slamko TF: slamko_map→slamko_odom→slamko_base
        │  (remap to map/odom/base_link)
        ▼
   Nav2 (global_costmap StaticLayer ← ~/volumetric_costmap ; local_costmap ← ~/local_costmap)
   lifecycle gated on `localized`  ─►  planner + controller  ─►  /cmd_vel  ─►  Isaac Sim robot
```
Same slamko binary as the bag runs — only the FRONT (sensor source) changes from `ros2 bag play` to
the Isaac Sim ROS 2 bridge.

## Steps (next session)
1. **Bring up Isaac Sim** with a D455-equivalent camera on a mobile base in an indoor scene; publish the
   ROS 2 topics OKVIS+slamko expect (infra stereo + IMU + depth + camera_info). Confirm topic
   names/QoS match what `run_full_immortal.sh` consumes (or remap).
2. **Run the slamko stack** against the sim topics (OKVIS + provider_fusion `volumetric:=true
   local_dynamic:=true imu_referee:=true hold_on_loss:=true depth_loop_refine:=true gate_live_pose:=true`).
   Confirm `~/volumetric_costmap` + `~/local_costmap` publish (use the robust `/tmp/glob_cap.py`, NOT
   capture_costmaps which misses the latched global).
3. **Wire Nav2** (PLAN_NAV2_01): nav2_params.yaml, TF remap slamko_*→map/odom/base_link, global_costmap
   StaticLayer ← `~/volumetric_costmap`, local_costmap ← `~/local_costmap`, lifecycle gate on `localized`.
4. **Drive a goal** → confirm planner + controller + reroute on a spawned obstacle.
5. **Rigorous A/B with GT**: validate the D455 clean-map fixes + the immortal gates against Isaac Sim
   ground truth (the confound-free measurement the bag never gave).

## Open questions for the next session
- Isaac Sim D455 sensor fidelity: does it model the stereo depth NOISE realistically (else the clean-map
  fixes won't show)? May need to inject noise or accept it's optimistic vs the real D455.
- Isaac Sim version + Isaac ROS bridge setup on this machine (TBD — check what's installed).
- Frame conventions: Isaac Sim camera optical frame vs OKVIS T_SC vs slamko depth_extrinsic — verify the
  TF tree (the "HIGHEST RISK = frame mismatch" lesson from the cuVSLAM provider work applies).
- nvblox cubes in RViz: not published today (only 2D OccupancyGrid + offline .ply); for live 3D in
  RViz add a mesh/voxel publisher, OR use the Rerun viz (`viz:=true`, viewer `/tmp/rrviewer`).

## State carried in (commit 220c130, all opt-in flags + defaults)
`imu_referee` OFF · `hold_on_loss` OFF · `atlas_break_on_quality` OFF · `gate_live_pose` OFF ·
`depth_loop_refine` OFF · `local_dynamic` **ON** · `volumetric_max_range_m` 3.5 ·
`costmap_noise_min_neighbors` 3 · `local_costmap_rate_hz` 10 · `depth_loop_max_rms` 0.15 ·
`depth_loop_min_cond` 0.02. For the full immortal config see `scratchpad/run_full_immortal.sh`.
Docs: PLAN_NAV2_01 (Nav2 wiring), PLAN_DEPTH_ODOM_01 (depth loop + Phase-2 small_gicp 2nd provider),
RESEARCH_IMMORTAL_IDEAL_01 (SOTA immortal SLAM + humanoids), RESEARCH_D455_CLEAN_MAP_01 (clean-map recipe).
