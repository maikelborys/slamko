# PLAN — Stress-test suite (the "never breaks" battery)

> **STATUS (2026-06-19): FOLDED INTO [`PLAN_ROBUSTNESS_01.md`](PLAN_ROBUSTNESS_01.md) §7 (scenarios S1–S9 + the differential golden-map method). Retained as the detailed harness/chaos-tool spec; the robustness plan is the source of truth for ordering and gates.**

<!-- status: DESIGNED 2026-06-12 (user directive: "sistema fuerte estable →
varios diferentes stress tests"). Implemented incrementally: S1-S3 land with
P-C, S5 with the reversible anchor, S7-S8 with the lifelong map-server. -->

The user's criterion is stability ("que sea bien estable sin romperse") and
auto-recovery on tracking loss. ATE benchmarks don't measure that — this
battery does. Every scenario has a hard PASS gate; a regression in any gate
blocks the phase that owns it. All scenarios run via `scripts/stress/` harness
entries (bench_pa.sh pattern: zombie-ABORT pre-flight, own-PID teardown,
machine-checkable verdict).

## The chaos tool

`slamko_ros/nodes/provider_chaos.py` — sits between the provider and
`provider_fusion_node` (remap `odom_topic` through it) and injects faults on
command/schedule: message DROP window (silence N s), DELAY/jitter, covariance
INFLATION (×k), and ORIGIN RESET (re-zero the odom frame mid-run = provider
crash/restart). Faults are scheduled (`--script t0:drop:10,t1:reset,...`) so
runs are reproducible.

## Scenarios

| # | Scenario | How | Owner phase | PASS gate |
|---|---|---|---|---|
| **S1** | Visual blackout / kidnap | `CASA1_Suave_blackout{,4}` bags (lens covered mid-run; OKVIS quality → Lost) | P-C | Zero crashes; supervisor seals+branches; on re-sight reloc lands a gated anchor; un-aligned divergence bounded; final map contains BOTH segments welded |
| **S2** | Provider death + restart | chaos `reset` (origin re-zero) + `drop` ≥10 s on a casa bag | P-C | Fuser detects the new odometry epoch (jump gate), starts a fresh chain segment, reloc re-anchors it; no TF teleport (slew bounded) |
| **S3** | Covariance storm | chaos `inflate ×100` windows (simulates Marginal/Lost reporting) | P-C | Edge information follows covariance (Hard Rule #3); fused pose never jumps; health policy state transitions logged Good→Marginal→Lost→Good |
| **S4** | Rate/latency abuse | bag at rate 0.5× / 2×; chaos `delay` 200 ms jitter | P-A (regression) | bench_pa gate still PASS (chain is rate-agnostic per Merfels) |
| **S5** | False-loop injection | inject an adversarial wrong loop edge (aliasing simulation) into the pose-graph | P-C (anchor gating) | Covariance gate + inertial sanity check rejects it, OR the reversible-merge undo recovers; map agreement post-check green |
| **S6** | Multi-floor | `CASA1_Escaleras` (already a P-A gate) + reloc per floor when P-B lands | P-A/P-B | No z-collapse; reloc never welds across floors |
| **S7** | Long-duration soak | same bag looped N× back-to-back (concatenated epochs) | P-C′/lifelong | Memory + graph size bounded (summarization works); optimize() latency flat |
| **S8** | Cross-session reloc | map session A (casa1) → localize session B (casa2 / casa1 re-run) | P-B gate | Reloc match rate + anchor correctness vs the RTABmap-validated reference |

## Order of implementation

1. **S1** first — the bags exist, it IS the P-C gate, no new tooling beyond the
   supervisor itself.
2. **provider_chaos.py** next (one small node unlocks S2/S3/S4).
3. **S5** lands with the reversible-anchor implementation (it tests the gate
   that P-C introduces).
4. **S7/S8** when the map-server/persistence layer exists in the new chain.

## Principles (from MASTER_PLAN, enforced here)

- Verdicts are machine-checkable (exit codes), never "looks fine in rviz".
- Report BOTH Sim3-aligned ATE AND un-aligned divergence (Hard Rule #5) — a
  blackout test that only reports aligned ATE hides the teleport.
- Reproducible: chaos schedules are deterministic scripts, bags are pinned.
- A scenario that can't fail is not a test: each gate has a documented way it
  WOULD fail (e.g. S2 fails today by design — the epoch detector doesn't exist
  yet; that's the point of building it).
