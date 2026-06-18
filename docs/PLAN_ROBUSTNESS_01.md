# PLAN — Robustness-first (the authoritative plan for "never-lost, never-corrupt")

<!-- status: AUTHORED 2026-06-19 · supersedes the P-C ordering in MASTER_PLAN §8
     and folds in docs/PLAN_STRESS_SUITE.md (S1–S8). Source of truth for the
     robustness work; MASTER_PLAN keeps the P-A..P-F frame + research provenance.
     Validated stamp bumps as each R-phase ships. -->

This plan answers the project's actual question — *can the robot map and get lost
without fear, because it always recovers and never corrupts its map?* — and fixes
the **order** of the remaining work. It is grounded in: the loose-fusion research
(anchor-don't-weld, PCM/GNC consensus, recall-at-100%-precision), the OKVIS
long-loss measurement (2026-06-19: OKVIS **never auto-resets**, holds warm state,
bridges visual loss with IMU, the "12 s reset" was an external restart), and a
code review that found the graph currently has **no garbage gates**.

---

## 1. Thesis — a never-lost system is defined by what must NEVER happen

Accuracy is secondary (user: *"me importa el resultado, que sea estable sin
romperse"*). The system is defined by three **invariants**; everything else is
built on top of them.

- **I1 — Never ingest garbage / unobservable motion into the map.** (input contract)
- **I2 — Never make an irreversible false connection.** (merge contract)
- **I3 — Never be permanently lost** — always tracking, recovering, or *honestly
  disconnected-but-recoverable*. (recovery contract)

If these hold, the robot has no fear. If one breaks, nothing downstream matters.

## 2. Model — the map is a federation of independent islands

The map is **not a single map**; it is a federation of internally-coherent
**submaps (islands)**, each in its own frame, connected by two edge types:

- **HARD edge** — appearance-verified (EigenPlaces retrieval → XFeat+LighterGlue →
  RANSAC inliers → consensus). Precise. Stored as a **reversible anchor** — never
  a destructive weld.
- **SOFT edge** — dead-reckoning-approximate (yaw from integrated gyro, roll/pitch
  from gravity), **high covariance**. Purpose: approximate visual placement +
  search-prior + navigation hint. Does not bend the real geometry.

A submap is sealed **only if it is good** (enough mature landmarks, healthy
tracking). No island is ever destructively fused with another. This is the
ORB-SLAM3 Atlas idea **made reversible** — its two fatal flaws (no consensus
layer, irreversible weld) are exactly what HARD-edge-reversible-anchor + consensus
fix.

## 3. Degradation is GRADED, not binary — one state machine covers wall, blackout, flip

| Regime | Signal (OKVIS) | Policy |
|---|---|---|
| **GOOD** | cov ×1, ≥30 % image coverage | ingest; **grow** the map (seal submaps) |
| **DEGRADED** | cov ×10–100, Marginal (<30 %); wall / low-texture | ingest pose with **inflated covariance**, but **PAUSE growth** (no new landmarks/keyframes baked into a submap); never seal or merge from here |
| **LOST** | odom stale-gap / IMU saturation / unobservable | **seal before, REFUSE the interval entirely, branch fresh after, reconnect by APPEARANCE only** |

- The **wall** ("apuntas a la pared, la pose se va") = DEGRADED → pause growth, don't
  bake drift, never seal a garbage submap.
- The **blackout** = LOST by stale-gap → OKVIS bridges <1 s on IMU, >1 s seal+branch.
- The **drone flip / fast trot / humanoid impact** = LOST by IMU saturation → the
  danger is not IMU *absence* (OKVIS holds) but IMU *lying* (saturated accel that
  OKVIS silently integrates and is poisoned by). Detect saturation → declare the
  interval unobservable → clean break.

Transitions use **hysteresis** (e.g. enter DEGRADED at cov ×10 for ≥3 frames, exit
at cov ×3 for ≥5 frames) to avoid Good↔Marginal flapping.

## 4. Honest epistemics — when motion is unobservable, do NOT invent a connection

When vision **and** IMU are both garbage for an interval, the relative motion is
**unobservable** — no trick recovers unmeasured metric information. The correct
behaviour is **two honest islands waiting for appearance overlap**, not a
fabricated bridge. A SOFT edge exists only if *some* weak cue survived (wheel/leg
odometry, a second IMU, the gyro if only the accel saturated); if nothing
survived, **there is no edge — and that is correct**. A monolithic SLAM cannot
survive a flip; a multi-map system that reconnects by appearance can, *because it
accepts the gap and closes it with recognition, not fantasy.*

## 5. The ordering insight — GATES before ANCHOR-EDGES

The current MASTER_PLAN jumps to **P-C′ (anchor edges in the graph)** next. But
anchor edges **propagate corrections across the whole graph**, and the review
proved the graph has **no input gates**. Propagating corrections through a graph
full of garbage **spreads the corruption faster**. Therefore:

> **Gates first (clean input) → anchor edges second (propagate clean corrections).**
> This is the single most important reprioritisation in this plan.

---

## 6. The phased plan (R0 → R3)

### R0 — Protect the input (I1) · highest priority, most exposed today
Maps onto a **P-C hardening pass** that must precede P-C′.

- **R0.1 — Empirical campaign (see §7).** RUN the stress bags + observe the garbage
  modes (0-landmark submaps, IMU poisoning, wall drift) **before** writing gates.
- **R0.2 — Seal-quality gate.** `provider_fusion_node` seals every 50 KF blindly
  (`:568`). Add: don't seal unless ≥N mature landmarks **and** tracking was healthy
  across the segment (image-availability check). Refuse/branch otherwise.
- **R0.3 — Ingestion health gate (the 3 regimes).** Classify each pose
  GOOD/DEGRADED/LOST from OKVIS covariance + TrackingQuality; inflate or hard-break
  accordingly; **never bake DEGRADED into a submap**. (No `if(sensor_ok)` — Hard
  Rule #3; degradation = covariance inflation. Hard break only on LOST.)
- **R0.4 — IMU saturation / jerk gate.** OKVIS's plausibility gates exist
  (`ImuError.cpp:66-106`) but are **disabled** in `rsD455_map_odom/okvis2.yaml`
  (`a_jerk_max=0`, `a_plausible_max=0`, `g_plausible_max=0`). Enable them
  (start: `a_plausible_max≈30`, `g_plausible_max≈5`, `a_jerk_max≈10`,
  `a_coherence_r≈0.01`) and/or pre-filter; on saturation, mark the interval
  unobservable (→ LOST).
- **R0.5 — Degraded-tracking policy.** The wall case, with hysteresis (R0.3 made
  concrete + tuned).

### R1 — Safe, propagating connections (I2)
- **R1.1 — Anchor edges in the graph (= P-C′).** Welds become pose-graph
  constraints so corrections propagate; mid-run z-dips (witnessed −2.3 m) defended.
- **R1.2 — Reversibility (anchor-don't-weld).** Anchor edges are removable; a merge
  later shown inconsistent is retracted (RRR-style cluster-and-undo).
- **R1.3 — Soft edges (4-DoF).** Yaw from integrated gyro (NOT compass indoors —
  iron-proximity test); roll/pitch from gravity; high covariance. For orphan-window
  placement + navigation hint.

### R2 — Harden and PROVE recovery (I3)
- **R2.1 — Long-loss / IMU-loss / clean-break handling.** Seal-before,
  refuse-interval, branch-after, reconnect-by-appearance. Validated on the campaign
  bags.
- **R2.2 — Adversarial false-merge test.** When a look-alike bag exists (or is
  synthesised), prove the consensus gate rejects. Metric = **recall-at-100 %-
  precision**, never the closure count.
- **R2.3 — Health monitor / supervisor state machine surfaced** (rviz panel + log)
  so the GOOD/DEGRADED/LOST state and seal/branch/weld events are observable live.

### R3 — Use the map (future, P-D+)
Navigable graph + VLM soft-edge traversal, dense mapping, GNSS anchoring (P-D),
magnetometer yaw, semantic layers (P-F), cross-session landmark maturation.

---

## 7. What to be testing — the differential method + the stress battery

### 7a. The core protocol (user-specified): golden map → stress same bag → compare

The reference is the **clean run of the same bag** — no external ground truth
needed. Every stress test is differential against it:

- **T0 — Golden map.** Run the bag with good tracking, no stress → the reference
  map (submaps + landmarks). Inspect: internally coherent, landmarks sensible. This
  is "what the environment actually looks like."
- **T1 — Stress on the SAME bag.** Inject blackouts / falls / IMU-loss / degradation
  into the same trajectory (chaos tool, blackout bags, or synthetic injection).
  Same scene, same path → same expected geometry.
- **T2 — Compare submaps against the golden map.** For each stressed submap, find
  its golden counterpart by location and measure agreement (landmark/pose
  discrepancy RMS). **PASS** = stressed submaps coincide with the golden map where
  they should; **nothing distorted, displaced, or garbage** entered. A 0-landmark
  or drifted submap shows up as no-match / high discrepancy — i.e. the test *sees*
  the garbage that the gates (R0) must stop.

This protocol directly probes the invariants: I1 (garbage → submap won't match
golden), I2 (false merge → spurious distortion vs golden), I3 (recovered →
segments reconnect onto golden).

### 7b. The stress battery (folded from PLAN_STRESS_SUITE.md — load-bearing)

Chaos tool: `slamko_ros/nodes/provider_chaos.py` sits between provider and
`provider_fusion_node` (remap `odom_topic` through it), injecting scheduled,
reproducible faults: message DROP window, DELAY/jitter, covariance INFLATION (×k),
ORIGIN RESET (provider crash/restart). `--script t0:drop:10,t1:reset,...`.

| # | Scenario | How | Owner | PASS gate |
|---|---|---|---|---|
| **S1** | Visual blackout / kidnap | `CASA1_Suave_blackout{,4}` (lens covered; OKVIS → Lost) | R2.1 | Zero crashes; seal+branch; on re-sight reloc lands a gated anchor; un-aligned divergence bounded; final map welds BOTH segments; **submaps match golden (T2)** |
| **S2** | Provider death + restart | chaos `reset` + `drop` ≥10 s | R2.1 | Fuser detects new odom epoch (jump gate), starts fresh segment, reloc re-anchors; no TF teleport (slew bounded) |
| **S3** | Covariance storm | chaos `inflate ×100` windows | R0.3 | Edge info follows covariance (Hard Rule #3); fused pose never jumps; state transitions GOOD→DEGRADED→LOST→GOOD logged |
| **S4** | Rate / latency abuse | rate 0.5×/2×; chaos `delay` 200 ms | R0 (regr.) | bench_pa gate still PASS (chain rate-agnostic) |
| **S5** | False-loop injection | inject adversarial wrong loop edge (aliasing) | R1.2 / R2.2 | Consensus + inertial sanity rejects it, OR reversible undo recovers; **map agreement vs golden green (T2)** |
| **S6** | Multi-floor | `CASA1_Escaleras` + per-floor reloc | R1.1 | No z-collapse; reloc never welds across floors |
| **S7** | Long-duration soak | same bag looped N× (concatenated epochs) | R3 | Memory + graph size bounded; optimize() latency flat |
| **S8** | Cross-session reloc | map A (casa1) → localise B (casa2 / re-run) | R2.1 | Reloc match rate + anchor correctness vs the RTABmap-validated reference |
| **S9** | IMU blackout / saturation | drop or saturate IMU in a window (extend `imu_relay.py`); + degraded `CASA1_Escaleras_occ80` | R0.4 | Saturated interval declared unobservable → clean break, NO poisoned poses; OKVIS warm-state intact; **submaps match golden (T2)** |

### 7c. Cross-cutting test discipline (always)
- Verdicts **machine-checkable** (exit codes), never "looks fine in rviz".
- Report **both** Sim3-aligned ATE **and** un-aligned divergence (Hard Rule #5) — a
  blackout test that only reports aligned ATE hides the teleport.
- Reproducible: chaos schedules deterministic, bags pinned.
- A scenario that **can't fail is not a test** — document how each gate WOULD fail.
- Visualise with `scripts/plot_slamko.py` (interactive 3D, submaps coloured +
  anchors) and `scripts/plot_neverlost.py` (2D top-down); the T2 overlay (stressed
  vs golden) is the primary artifact.
- Benchmark-driven: regression ≥5 % in any gate → revert.

---

## 8. Reference — OKVIS exact limits (measured 2026-06-19, never re-derive)

- **No auto re-initialisation.** `isInitialized_` set once (`Frontend.cpp:770`),
  cleared only pre-init. No watchdog. The "12 s reset" was an external restart in
  the old RTABmap setup — **we never restart OKVIS.**
- **TrackingQuality** (`ViInterface.hpp` + `ThreadedSlam.cpp:1068`): Lost if <0.01
  coverage, Marginal 0.01–0.30, Good ≥0.30. Returns 0 if <8 matched points.
- **Covariance IS populated** (`Publisher.cpp:287`) — heuristic, quality-scaled
  ×10 (Marginal) / ×100 (Lost). **Not** the true Hessian, but monotonic → usable as
  the health signal.
- **`maxImuPropagationGap_ = 1.0 s`** — holds the last pose past a 1 s IMU gap
  (= the user's <1 s / >1 s rule, for free).
- **IMU hard saturation** `a_max=160`, `g_max=10`; **plausibility/jerk gates
  DISABLED** in `rsD455_map_odom` (the R0.4 hole).
- **`keyframe_overlap=0.55`**; `max_num_keypoints=1200`; `num_keyframes=5`.

## 9. Status & next

- **Next action:** R0.1 — the empirical campaign (§7), serial, on the user's GPU.
  No code changes until the garbage modes are observed.
- Live status + queue: [`PIPELINE_STATUS_01.md`](PIPELINE_STATUS_01.md).
- Task list (R0–R3 mapped): tasks #2–#8 in the session tracker.
