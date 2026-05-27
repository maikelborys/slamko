# slamko_msgs — ROS 2 interface definitions

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** the wire contracts. Message/service/action defs for the **map-server API**
(submap exchange, `map→odom` correction), cross-session/loop **correspondences**
(`T_AB` + score + segment id), **localization_status** lifecycle, and factor/odom
telemetry. Pure interface package (no logic) so producers/consumers and multi-robot
share one schema.

**Depends on:** nothing (interface-only). **Consumed by:** slamko_ros, slamko_mapping.
**Status:** planned. **Phase P4** (with slamko_mapping).

**Starting cold here?** Read the 3 hub docs + this, then plan mode →
`docs/PLAN_P4_msgs.md`.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
