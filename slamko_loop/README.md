# slamko_loop — global consistency & the never-lost spine

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** Tier-3 global. (1) **loop-closure-as-factor** (relative-pose / pose-graph
edges); (2) the **never-lost multi-map supervisor** — state machine
OK→RECENTLY_LOST→LOST + **archive-don't-discard** + the user's validated
**seal→branch→relocalize→merge** (decoupled, gates OUTSIDE the graph); (3)
**defensive numerics** (GLIM): disposable global graph, catch singular-system →
damp → rebuild from odometry. Loss detection = **odometry stale-gap** (not covariance).
(4) **relocalization** (folded in): LiftFeat-m1 default place-recognition + weld;
the libtorch/TRT extractor is an **optional build target**, not a package.
(5) the **health POLICY**: the Good/Marginal/Lost state machine + watchdogs +
recovery triggers ARE this supervisor (health *signals* come from core/vio/fusion).

**Depends on:** slamko_core only. **Owns:** `map→odom`.
**Status:** planned. **Phase P2 — the priority feature.**

**Starting cold here?** Read the 3 hub docs + this + the user's OKVIS2-X
`ATLAS_DESIGN_01.md` (the proven template), then plan mode → `docs/PLAN_P2_loop.md`.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
