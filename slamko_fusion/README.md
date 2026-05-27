# slamko_fusion — heterogeneous fixed-lag smoother (the VILENS heart)

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** the Tier-2 local fusion graph. A fixed-lag smoother over `T_WB` +
velocity + bias + landmark nodes, ingesting heterogeneous `Factor`s by covariance
(no sensor branches). **Marginalization** (Schur complement + First-Estimates
Jacobian) replaces gauge-by-constant — the biggest quality upgrade vs klt_vo.

**Implements:** `FactorGraphBackend` adapters — **`GtsamBackend` (iSAM2, default)**
+ `CeresBackend` (S1 inner loop / fallback). **Depends on:** slamko_core (+ GTSAM, Ceres).
**Status:** planned. **Phase P1** (with slamko_core).

**Starting cold here?** Read the 3 hub docs + this, then plan mode →
`docs/PLAN_P1_fusion.md`. Marginalization reference: VINS-Mono `MarginalizationFactor`
(Ceres) or GTSAM `IncrementalFixedLagSmoother`.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
