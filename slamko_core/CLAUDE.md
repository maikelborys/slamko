# slamko_core — contracts & common types (the spine)

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** defines every contract the other modules meet at — `Factor`,
`SensorFrontend`, `FactorGraphBackend`, `Relocalizer`, `SubMap`, `EstimationFrame`,
`NodeKey`/`NodeType`, `RobustKernel`, SE3/manifold helpers. **Header-light, zero
heavy deps** (Eigen only). Everything depends on this; this depends on nothing.

**Implements:** the interfaces themselves (abstract). **Consumes:** —.
**Depends on:** nothing.
**Status:** planned (Phase P1 — defined alongside slamko_fusion).

**Starting cold here?** Read the 3 hub docs above + this file, then enter plan
mode → `docs/PLAN_P1_core.md`. The interface sketches in `../docs/DECOUPLING.md`
are the starting point.

**Doc rule:** on green tests → update `docs/STATUS.md` + bump validated stamps →
commit code+docs together (`../docs/DOC_PROCESS.md`).
