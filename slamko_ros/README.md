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
`launch/pa_okvis_bag.launch.py` (OKVIS2-X pure-VIO on a D455 bag). Bench:
`../scripts/bench_pa.sh`. Numbers: `docs/STATUS.md`.

**Starting cold here?** Read the 3 hub docs + this, then plan mode →
`docs/PLAN_ros.md`. Keep ONE namespacing convention; read message stamps directly.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
