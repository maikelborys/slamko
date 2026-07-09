# slamko_ros — ROS 2 integration (the composition root)

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** **the only module that knows all the others** — it wires concrete
implementations together (composition root) and exposes them to ROS 2. Nodes,
launch files, and the **bridge** that owns the Nav2 contract: `/map` (latched),
`/tf` (`map→odom→base_link`), `/odom`, lifecycle gating on localization status.
Mirrors the proven `okvis_nav2_bridge` pattern. **Also hosts visualization**
(rviz panels: submaps, segment graph, health/localization status); offline Plotly
viz lives in `../scripts/`.

**Depends on:** all `slamko_*` + slamko_msgs. **Status:** P-A active —
`provider_fusion_node` (loose fuser over an external odometry provider:
relative KF edges + covariance → `slamko_loop::PoseGraph`, `map→odom` slewed
TF) + `provider_chain_offline` (the no-ROS reproducible gate) +
`launch/pa_okvis_bag.launch.py` (OKVIS2-X pure-VIO on a D455 bag) · `launch/pa_cuvslam_bag.launch.py` (cuVSLAM-Inertial provider, P-A PASS 2026-07-09). Bench:
`../scripts/bench_pa.sh`. Numbers: `docs/STATUS.md`.

**Starting cold here?** Read the 3 hub docs + this, then plan mode →
`docs/PLAN_ros.md`. Keep ONE namespacing convention; read message stamps directly.

## Live visualizer (Rerun) — `VizSink`

A live "Pangolin-but-modern" debug viewer (rerun.io): window A = the camera image
with XFeat keypoints overlaid + a HUD; window B = the 3D landmark map building
live, camera frustums, and pose-graph edges drawn **by type** (chain/soft/loop/
cross-session-prior/proximity-candidate). One scrubbable timeline; dangling submaps
hang honestly. OFF by default — the node builds + runs identically without it
(`viz_sink.cpp` is a no-op unless `-DSLAMKO_WITH_RERUN`).

```bash
# build with the viz (fetches the Rerun C++ SDK 0.33 once)
cd ~/coding/slamko && colcon build --packages-select slamko_ros \
  --cmake-args -DSLAMKO_WITH_RERUN=ON

# terminal 1: launch a viewer (pip install rerun-sdk, or the `rerun` binary)
rerun                       # opens the live viewer, listens on the default gRPC port

# terminal 2: run slamko with viz on (live stream) ...
ros2 run slamko_ros provider_fusion_node --ros-args -p viz:=true   # + your usual params
# ... OR record an offline, rewindable capture (open later with `rerun run.rrd`):
ros2 run slamko_ros provider_fusion_node --ros-args -p viz:=true \
  -p viz_endpoint:=/mnt/data/bno_ab/run.rrd

# no-ROS smoke test of the viz path (writes a synthetic .rrd):
ros2 run slamko_ros viz_selftest /tmp/x.rrd && rerun /tmp/x.rrd
```

First place to look if window B is empty: confirm `reloc:=true` (keypoints/submaps
come from the XFeat path) and that the `rerun` viewer is reachable (the node logs
`live viz: STREAMING` vs `NOT connected` at startup). Detail + the edge-type colour
key: `docs/STATUS.md` (2026-06-20) + `../docs/RESEARCH_LIFELONG_FUSION_01.md` §viz.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
