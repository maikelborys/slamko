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

**Depends on:** all `slamko_*` + slamko_msgs. **Status:** planned (after the
modules it composes exist).

**Starting cold here?** Read the 3 hub docs + this, then plan mode →
`docs/PLAN_ros.md`. Keep ONE namespacing convention; read message stamps directly.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
