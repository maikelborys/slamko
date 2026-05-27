# slamko_core — contracts & common types (the spine)

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** defines every contract the other modules meet at — `Factor`,
`SensorFrontend`, `FactorGraphBackend`, `Relocalizer`, `SubMap`, `EstimationFrame`,
`NodeKey`/`NodeType`, `RobustKernel`, SE3/manifold helpers. **Header-light, zero
heavy deps** (Eigen only). Everything depends on this; this depends on nothing.

**Also owns cross-cutting infra** (GLIM `common/`+`util/` model): time-sync /
buffering (TimeKeeper, interpolatable trajectory buffer, thread-safe queues),
config + per-platform presets, structured logging, map **serialization schema**,
frame conventions, and the **health-signal interfaces** (probes `vio`/`fusion`
emit). The "easy-to-forget" pieces every reference built into core (GLIM) or
painfully retrofitted (OKVIS2-X). Still thin on *algorithms*. `SubMap` lives here
until slamko_mapping is split out (P4).

**Implements:** the interfaces themselves (abstract). **Consumes:** —.
**Depends on:** nothing.
**Status:** ✅ **shipped** (P1) — contracts + SE3 + `LocalSmoother` + health signals,
plus **SubMap serialization** (`submap_io.hpp`, P4a: the map-persistence schema for
cross-session). 26 gtests, 0 fail. `SubMap` lives here until `slamko_mapping` splits out.

**Starting cold here?** Read the 3 hub docs above + this file + `docs/STATUS.md`. The
interface contracts are in `../docs/DECOUPLING.md`.

**Doc rule:** on green tests → update `docs/STATUS.md` + bump validated stamps →
commit code+docs together (`../docs/DOC_PROCESS.md`).
