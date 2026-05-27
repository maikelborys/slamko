# slamko_viz — visualization tools

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** see what's happening. Offline Plotly (trajectory vs GT + ATE + landmark
map; dead-reckoning compare) and online RViz panels (submaps, multi-map/segment
graph, localization status). Seed from klt_vo `scripts/viz_traj_ate.py`,
`viz_dr_compare.py` (+ the `.venv-viz` plotly/kaleido setup).

**Depends on:** nothing (consumes outputs / topics). **Status:** planned.

**Starting cold here?** Read the 3 hub docs + this, then plan mode →
`docs/PLAN_viz.md`.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
