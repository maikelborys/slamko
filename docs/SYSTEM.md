# slamko — System map (how the modules interact, now & next)

<!-- validated: 2026-05-27 (P2 CLOSED · P4a) · tests: core 26 + fusion 4 + vio 24 + loop 32 gtest 0 fail · gtsam tracks MH_01 · never-lost seal→branch→WELD→recover + MULTI-SUBMAP pose-graph merge (2 disjoint sealed) validated live on V1_01 (XFeat, auto-check 7/7) · SE3 pose-graph + stress + weld-once + disjoint submaps + SubMap serialization (P4a) -->

The one-page projection of the whole system. A **map, not a textbook** — it states
what's true now + where it's headed, and is corrected as code lands. Deep detail
lives next to the code (header "why" blocks) and in each package's `docs/STATUS.md`.
Read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md) +
[`DECOUPLING.md`](DECOUPLING.md) alongside this.

**This file is the cold-start orientation map** — start here to locate any module,
its state, and where its notes live. Skim the table below, then dive into the
package's `README.md` → `docs/STATUS.md` → `docs/PLAN_<phase>.md`.

## Status at a glance — "where are we"

One row per package; updated when a milestone lands (detail in each `docs/STATUS.md`).
✅ shipped · 🟢 active · ⬜ planned/deferred.

| Package | Tier | Phase | State | Headline | Validated |
|---|---|---|---|---|---|
| `slamko_core` | spine | P1 | ✅ shipped | contracts + SE3 + feature seam + **SubMap serialization (`submap_io.hpp`, P4a)** · 26 gtests | `e7a1953`+ |
| `slamko_vio` | T1 | P0/P1b | ✅ shipped | Shi-Tomasi 0.078 m @ ~214 fps / **XFeat-TRT 0.049 m @ ~93 fps** (equal-coverage MH_01) · descriptors attached · **routed through `LocalSmoother` (ceres), `backend:=ceres\|gtsam`** | `8498021`+ |
| `slamko_fusion` | T2 | P1 | ✅ shipped | GtsamLocalSmoother; **P1b** ceres routing (unit-exact); **P1c** `backend:=gtsam` injected at node, **tracks MH_01 end-to-end 0 smoother-fails, real-time** (fixed latent CombinedImuFactor arg-order bug + landmark mgmt). Full-seq ATE + default-flip deferred (box harness) | `8498021`+ |
| `slamko_loop` | T3 | **P2 ✅ CLOSED** | ✅ shipped | never-lost supervisor + XFeat relocalizer + **SE3 pose-graph backend** (loop-closure-as-factor, GN+LM, drop-bad-edge) + weld-once. **Validated end-to-end on live V1_01 (XFeat): single-submap weld (P2c) AND multi-submap pose-graph merge** (2 sealed submaps, 2 welds, nodes{0,1,2}/edges{0→1,0→2}). 32 gtests 0 fail (incl. 10-test stress suite: 30-submap chain, 5×5 grid loops, outlier-drop, deterministic, gauge-free stability) | `0139e29`+ |
| `slamko_msgs` | — | P4 | ⬜ planned | map-server API / status / correspondences | — |
| `slamko_ros` | root | — | ⬜ planned | composition root + map→odom→base bridge + viz | — |
| `slamko_mapping` | T3 | P4 | 🟢 active | **P4a ✅** SubMap (de)serialization (`submap_io.hpp`); **P4b ✅** cross-session localization — reactive (`prior_map_dir` + recovery weld, ATE 13.9 cm) AND proactive (`continuous_reloc` welds into the prior map while OK, no loss — ATE 6.9 cm), both auto-check 7/7 on V1_01. Next: cross-session merge viz, then split the package | — |
| `slamko_sensors` | T1 | P5 | ⬜ deferred | wheel/ZUPT → LiDAR → GPS frontends | — |
| `slamko_semantic` | T1/T3 | P6 | ⬜ deferred | object-level factors + semantic reloc | — |

**External integration** (robot stack, *not* slamko packages): D455 driver / sim
(`cerebro_robot_sim`) / EuRoC bag → `slamko_ros` inputs; `slamko_ros` → Nav2 + TF
downstream. The wider workspace is mapped in [`~/coding/CLAUDE.md`](../../CLAUDE.md).

**Where the notes live** (find any decision fast): architecture + *why* → this map +
the inline header "why" blocks · what's validated, the numbers, what was
tried-and-reverted → each package's `docs/STATUS.md` · locked decisions + roadmap →
`MASTER_PLAN.md` · contracts → `slamko_core` headers + `DECOUPLING.md` · rules →
`CLAUDE.md`.

## 3 tiers · 2 graphs · 3 owners

```
 SENSORS                 TIER 1 · FRONTENDS            TIER 2 · LOCAL FUSION         TIER 3 · GLOBAL
 (D455 / EuRoC bag)      (plugins, emit Factors)       (fast graph, owns odom→base)  (async, owns map→odom)

 stereo + IMU ──► slamko_vio                           slamko_fusion                  slamko_loop
                  FeatureSource ─detect→ Features       fixed-lag smoother            place-rec (Relocalizer)
                  FeatureTracker ─KLT──→ Track          (GtsamBackend iSAM2)          never-lost supervisor
                  stereo+PnP, IMU preint, dead-reckon   + marginalization (Schur+FEJ) seal→branch→reloc→weld
                  CeresBackend (S1 inner loop)          emits HealthSignal            defensive numerics
                       │                                      │                            │
                       └──── EstimationFrame ────────────────►│──── SubMap ───────────────►│
                            (T_WB, v, bias, custom_data)      │     (KF poses, landmarks,   │
                                                              │      descriptor index)      │
                                                              ▼                            ▼
                                                       odom→base (TF)              map→odom (TF correction)
                                                              └──────────┬─────────────────┘
                                                                         ▼
                                              slamko_msgs (map-server API, status, correspondences)
                                                                         ▼
                                              slamko_ros (composition root: nodes, map→odom→base bridge, rviz)
```

**Two graphs (locked).** A *fast* bounded local smoother (Tier 2) for real-time
pose, and a *separate async* global graph (Tier 3) for consistency/loop-closure/
recovery. The fast odometry **never depends on** the slow global graph — so the
global graph is disposable (can be damped, torn, rebuilt from odometry) without ever
stalling the tracker.

**Three TF owners, never overlapping.** vio/fusion own `odom→base`; loop owns
`map→odom`; `slamko_ros` is the only place that publishes the composed tree. Nothing
else writes TF (the OKVIS2-X "one owner per transform" rule).

**The two types that cross tiers:** `EstimationFrame` (Tier 2→3) and `SubMap`
(Tier 3 unit), each with a `custom_data` escape hatch so the pipeline extends without
changing the interface. Everything else is package-internal.

## Threading doctrine (locked — OKVIS2-X + GLIM)

- **Single-writer per graph.** Each graph (local, global) has exactly one writer
  thread. No shared-mutable graph state across threads.
- **Cross-thread requests = atomic flags, consumed at ONE race-free point** — after
  the optimize() join, never mid-solve. A deferral backlog is replayed on merge.
- **Bounded queues with back-pressure, never stall.** A slow global thread drops/
  coalesces, it does not block the hot path.
- Thread map: `slamko_vio` = fast hot thread (per-frame, µs budget) · `slamko_fusion`
  = local-graph thread (keyframe rate) · `slamko_loop` = async/disposable thread
  (place-rec + recovery, best-effort).

## The never-lost spine (3 timescales × where it lives)

| Timescale | Mechanism | Lives in | Status |
|---|---|---|---|
| ms–0.5 s (blur/occlusion) | IMU / const-vel+gyro **dead-reckoning** | `slamko_vio` | ✅ ported (klt_vo) |
| seconds–minutes (medium loss) | **archive-don't-discard**: seal submap → branch fresh → keep emitting odom | `slamko_loop` supervisor | P2 |
| long loss / kidnap | **relocalize** (LiftFeat-m1 + DBoW) → **weld** with one relative-pose constraint | `slamko_loop` `Relocalizer` | P2/P3 |

**Loss trigger = odometry stale-gap** (`HealthSignal.odom_stale_gap_s`), wall-clock/
monotonic — NOT a covariance spike. A blackout *pauses* odom; it does not inflate
covariance (the OKVIS2-X finding). The covariance/eigenvalue fields in `HealthSignal`
are degeneracy monitors, not the primary loss signal.

## Two principles that are code-paths, not slogans

- **Degradation = covariance, not a branch.** A degraded modality is a `Factor` with
  inflated `sqrt_information()`; its weight shrinks toward zero and the optimizer
  down-weights it automatically. There is no `if(sensor_ok)` anywhere.
- **The global graph is disposable.** The backend catches the optimizer's
  indeterminant-system exception, damps the offending variable, and rebuilds missing
  nodes/edges from the still-trusted odometry. A bad loop closure damps, never crashes.

## Health flow

`slamko_core` defines the *signals* (`HealthSignal`, `HealthReporter`). `slamko_vio`
and `slamko_fusion` *emit* them (stale-gap, min-eigenvalue, marginal-cov, inlier
ratio). `slamko_loop` owns the *policy* — the Good/Marginal/Lost state machine,
watchdogs, recovery triggers. Signals and policy are deliberately split so the
supervisor can gate recovery **outside** the estimator graph (tight in-graph coupling
"pendulates").

## Contract vs implementation (the evolution map)

**Stable contracts** (change rarely, everything depends on them): the `slamko_core`
interfaces — `Factor`, `SensorFrontend`, `FactorGraphBackend`, `FeatureSource`,
`FeatureTracker`, `Matcher`, `Relocalizer` — plus the `slamko_msgs` map-server API.

**Swappable implementations** (add/replace freely, nothing else changes): every
`*Source` (ShiTomasi / XFeat / LiftFeat-m1), every `*Backend` (Ceres / GTSAM), every
`Relocalizer`, every sensor frontend. Swap one by registering a different impl in the
`slamko_ros` composition root — no other package edits.

**How the deferred packages slot in without touching existing code:**
- `slamko_mapping` (P4): `SubMap` persistence behind the map-server contract.
- `slamko_sensors` (P5): a wheel/LiDAR/GPS frontend = `registerFrontend(new WheelFrontend(T_BS))`. The backend never names a sensor.
- `slamko_semantic` (P6): object-level `Factor`s (quadric/cuboid) on the same graph.

That "register a frontend / backend / relocalizer, never a rewrite" property is the
whole point of the `slamko_core` seam (DigiForest/VILENS decoupling).

## Glossary (load-bearing terms)

- **T_WB** — body-in-world pose, the standard pose node; each sensor carries its own `T_BS` (sensor-in-body).
- **Factor** — `{keys, residual, √information, robust_kernel}`; the unit of fusion and of pluggability.
- **SubMap** — self-contained, serializable chunk of the global map (KF poses + landmarks + descriptor index + anchor); the unit of archive-don't-discard.
- **Marginalization** — dropping old nodes via Schur complement → a linear prior on the separator, with **FEJ** (First-Estimates Jacobian) for consistency. Replaces lossy gauge-by-constant.
- **Stale-gap** — seconds since the odometry stream last advanced; the loss trigger.
- **Weld** — re-anchoring an archived submap to the live one with a single relative-pose constraint after a relocalization hit.
