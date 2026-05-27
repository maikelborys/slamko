# slamko_semantic — semantic layer

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** semantics as first-class SLAM. Object detection / segmentation →
**object-level factors** (quadric / cuboid landmarks, QuadricSLAM/CubeSLAM style,
DCS-robustified) + semantic map layers + semantic relocalization (recognize *what*,
not just *where*). It's a `SensorFrontend` that emits object factors, plus a map
layer. Later: the seam to the user's higher-level stack (Cerebro/Médula).

**Implements:** `SensorFrontend` (object factors). **Depends on:** slamko_core,
slamko_mapping (+ a detector net, Apache/BSD). **Status:** planned. **Phase P6.**

**Starting cold here?** Read the 3 hub docs + this, then plan mode →
`docs/PLAN_P6_semantic.md`.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
