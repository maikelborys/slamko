# slamko_loop — global consistency & the never-lost spine

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** Tier-3 global. (1) **loop-closure-as-factor** — a self-contained **SE3
pose-graph backend** (`pose_graph.{hpp,cpp}`, Gauss-Newton + LM, drop-bad-edge) over
submap anchors; (2) the **never-lost multi-map supervisor** — state machine
OK→RECENTLY_LOST→LOST + **archive-don't-discard** + validated
**seal→branch→relocalize→WELD→merge** (decoupled, gates OUTSIDE the graph); (3)
**defensive numerics** (GLIM): disposable global graph, catch singular-system →
damp → rebuild. Loss detection = **odometry stale-gap** (not covariance).
(4) **relocalization** (folded in): **XFeat** place-recognition + PnP + weld
(`xfeat_relocalizer`), reusing the descriptors the VIO already attaches — no new model
(LiftFeat-m1 a future swap). (5) the **health POLICY** (Good/Marginal/Lost) IS this
supervisor. (6) **cross-session**: `seedPriorMap` loads a prior Atlas → the same weld
machinery localizes a fresh session into it (reactive + continuous-in-OK).

**Depends on:** slamko_core only. **Owns:** `map→odom`.
**Status:** ✅ **shipped** — P2 + P2.5 (pose-graph) + cross-session (P4b). Full
seal→branch→WELD→recover, multi-submap merge, and prior-map relocalization validated
end-to-end on EuRoC V1_01 (XFeat); gated by `../scripts/check_neverlost.py`.

**Starting cold here?** Read the 3 hub docs + this + `docs/STATUS.md` (the numbers) +
`docs/PLAN_P2_loop.md` (the phase map).

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
