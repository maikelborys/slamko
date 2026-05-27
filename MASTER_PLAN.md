# slamko — Master Plan

<!-- validated: (scaffold) 2026-05-27 · tests: n/a -->

The top-level plan. Per-module detailed plans derive from this and live in each
module's `docs/PLAN_*.md` (produced in plan mode when work on that module starts).
Technical deep-dive + research provenance: `~/coding/klt_vo/docs/14_slam_master_plan.md`
(the originating design doc; will migrate into slamko_vio/slamko docs).

## 0. Vision & principles

A modular SLAM framework that **never gets lost, never fails, is super-recoverable,
yet SIMPLE and STABLE.** Five principles (one per reference system):

1. **Robustness by deletion** (RKO-LIO) — fewer assumptions = fewer silent failures.
2. **Degradation = covariance, not a branch** (VILENS).
3. **The global graph is disposable** (GLIM) — catch singular-system → damp → rebuild from odometry; never crash.
4. **Decouple at a published map contract** (DigiForest) — odom ⊥ reloc ⊥ mapping ⊥ map-server.
5. **Recovery = state machine + archive-restart, not merge** (ORB-SLAM3) — heavy merge optional/S2.

## Package structure (start lean: 6)

Start at **6 packages** — `core, vio, fusion, loop, msgs, ros` — not 12. The
pluggability lives in `slamko_core`'s interfaces, not in package count; split more
out only when a phase's code crosses a real boundary (so contracts aren't frozen
before data flows). **Deferred:** `slamko_mapping` (P4), `slamko_sensors` (P5),
`slamko_semantic` (P6). Folded: reloc→loop (libtorch as optional target), viz→ros,
bench→`scripts/`. Health is designed-in (signal ifaces in core, policy in loop).
Grounded in how GLIM/Kimera (≈1 lib) and OKVIS (~13, matured) actually package.

## 1. Architecture — 3 tiers, 2 graphs

- **Tier 1 · Frontends** (`slamko_vio`, `slamko_sensors`, `slamko_semantic`): each
  turns a measurement into **factors** (+ its own covariance). Sensor-agnostic core.
- **Tier 2 · Local fusion** (`slamko_fusion`): GTSAM iSAM2 fixed-lag smoother over
  `T_WB` + velocity + bias + landmark nodes. **Marginalization** (Schur+FEJ).
  Fast, owns `odom→base`, never blocks, never depends on Tier 3.
- **Tier 3 · Global** (`slamko_loop`, `slamko_mapping`, `slamko_reloc`): submaps,
  place recognition, loop-closure-as-factor, never-lost multi-map supervisor,
  cross-session, map-server. Async, disposable, owns `map→odom`.

The only types crossing tier boundaries: `EstimationFrame`, `SubMap` (each with a
`custom_data` escape hatch). Contracts in `slamko_core`; see `docs/DECOUPLING.md`.

## 2. The Factor abstraction (register-not-rewrite)

`Factor = { keys (typed node handles), residual, √information (the only uncertainty
knob), robust_kernel }`. `SensorFrontend::process()` turns measurements into factors;
`FactorGraphBackend` (gtsam/ceres adapter) owns nodes+solve+marginalization. Adding a
sensor = `registerFrontend(...)`. Full interface sketch in `docs/DECOUPLING.md`.

## 3. The "never lost" spine (the priority)

Three safety nets, three timescales:
1. **Short gap (~ms–0.5s):** IMU / const-vel+gyro dead-reckoning. ✅ done in klt_vo (R).
2. **Medium loss → multi-map:** state machine OK→RECENTLY_LOST→LOST; **archive-don't-
   discard** (seal submap, branch a fresh one, keep producing odometry). Port the
   user's validated OKVIS2-X **seal→branch→relocalize→merge** (decoupled supervisor).
3. **Long loss / kidnap:** relocalize (LiftFeat-m1) + weld with one relative-pose
   constraint. Full welding-BA merge optional / delegated to S2.
Defensive numerics (GLIM): disposable global graph, catch-damp-rebuild.

## 4. Locked decisions
Two-graph S1/S2 · GTSAM iSAM2 backend (Ceres = S1 inner loop) · pose node `T_WB` ·
marginalization replaces gauge-by-constant · decoupled recovery supervisor (tight
coupling "pendulates") · loss detection = odometry stale-gap (not covariance) ·
relocalizer default **LiftFeat-m1** (un-boosted; beats XFeat, BoW-stable) · Apache/BSD only.

## 5. Roadmap (each phase → a module's detailed plan, made in plan mode)

| Phase | Module(s) | What | State |
|---|---|---|---|
| **P0** | slamko_vio | Solidify S1: gravity init, IMU dead-reckoning, feature compare-all ATE (ShiTomasi vs XFeat), full-suite validation | ✅ |
| **P1** | slamko_core, slamko_fusion | `Factor`/`Frontend`/`Backend` + `LocalSmoother` interfaces; **marginalization**; GTSAM adapter (tracks MH_01) | ✅ |
| **P2** | slamko_loop | Never-lost core: state machine + archive + decoupled supervisor + catch-damp-rebuild; **+P2.5** SE3 pose-graph backend (loop-closure-as-factor). Full seal→branch→WELD→recover + multi-submap merge live on V1_01 | ✅ |
| **P3** | slamko_loop / reloc | **XFeat reloc + cheap weld ✅** (P2b) + **BoW vocabulary + inverted-index** for scalable candidate pre-selection ✅ (P3a/b, `bow.{hpp,cpp}`, fallback-safe, no recall regression). LiftFeat-m1 + a persisted/pre-trained vocabulary deferred | ✅ |
| **P4** | slamko_core → slamko_mapping, slamko_msgs | **Submap persistence ✅** (`submap_io.hpp`) + **multi-session ✅** (load prior map → relocalize, reactive + continuous). map-server contract + package split = remaining | 🟢 active |
| **P5** | slamko_sensors | Wheel+ZUPT → LiDAR plane/line → GPS → (objects in semantic) | ⬜ |
| **P6** | slamko_semantic | Object-level factors, semantic map layers, semantic reloc | ⬜ |

Sequencing: lean base solid (P0 ✅) → extensible (P1 ✅) → never-lost spine (P2 ✅, the
priority) → recovery/reloc (P3 ✅, scalable index in) → persistence/multi-session (P4 🟢) →
sensors (P5) → semantics (P6). **Note:** P3/P4 interleaved early because cross-session persistence reuses
the same relocalizer + pose-graph weld as in-session recovery (one Atlas, one weld machine).

## 6. Open items
Migrate klt_vo → slamko_vio (when P0 closes). Wire a Claude Code hook to run
`check_doc_freshness` on Stop (optional). Decide multi-map: core-always-on vs optional
(recommend: state-machine+archive always on, merge optional).
