# slamko_bench — benchmark harness

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** shared, reproducible benchmarking. EuRoC **ATE** (evo), **FPS** controlled
A/B (interleaved + warmup to cancel GPU DVFS — see klt_vo `bench_fps_ab.sh`), the
**feature compare-all matrix** (Shi-Tomasi+KLT vs XFeat-detect+KLT vs XFeat-match vs
LiftFeat-m1), forced tracking-loss harness for the never-lost tests. One place so
every module is measured the same way; results feed each module's `docs/STATUS.md`.

**Depends on:** nothing (drives the built nodes). **Status:** planned (seed from
klt_vo `scripts/bench_ate.sh`, `bench_fps_ab.sh`, `viz_*`). 

**Starting cold here?** Read the 3 hub docs + this, then plan mode →
`docs/PLAN_bench.md`.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
