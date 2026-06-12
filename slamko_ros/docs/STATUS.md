# slamko_ros — STATUS (validated facts + numbers)

<!-- validated: 2026-06-12 · tests: offline gate PASS (Suave+Escaleras TUMs) · live gate PASS (CASA1_Suave, CASA1_Escaleras bags) -->

## 2026-06-12 — P-A shipped: provider_fusion_node + loose chain over OKVIS2-X

First real content of the composition root (MASTER_PLAN v2 §8 P-A — the
smallest end-to-end loop of the loose-fusion-over-external-odometry pivot).

**What landed**
- `slamko_core/odometry_provider.hpp` — `ProviderChain`: provider pose stream →
  keyframe decimation (0.10 m / 0.10 rad / 1 s) → relative edges, information
  from the provider's reported covariance (diag sum of the two endpoint poses,
  floored at 5 mm/2 mrad; PoseGraph default sigmas when the provider reports
  zero). OKVIS's quality-scaled covariance (10× Marginal / 100× Lost) flows
  straight into edge weight — Hard Rule #3, no `if(sensor_ok)`.
- `nodes/provider_fusion_node.cpp` — subscribes `/okvis/okvis_odometry`
  (param; reliable default, `provider_best_effort` for BE providers), feeds
  `slamko_loop::PoseGraph` (chain-composes in the fused frame so a future
  optimize() propagates), publishes `~/fused_odometry` (map frame, exact
  correction) + slewed `slamko_map→slamko_odom` TF (0.5 m/s, 0.5 rad/s caps) +
  `slamko_odom→slamko_base` raw provider TF (param-gated). TUM dumps for bench.
- `tools/provider_chain_offline.cpp` — the no-ROS reproducible gate: recorded
  provider TUM → same chain+graph path → optimize() → compare vs input.
- `launch/pa_okvis_bag.launch.py` — OKVIS2-X **pure VIO** (config
  rsD455_odom848: `do_loop_closures=false`, the double-LC rule) on a D455 bag +
  the fusion node. `scripts/bench_pa.sh` = pre-flight zombie ABORT (never
  blind-pkill — the okvis process may be the user's), run, teardown only our
  PIDs, gate compare.

**Numbers (gate: fused must track the provider — no global constraints yet)**

| Gate | Input | KFs/samples | Result |
|---|---|---|---|
| offline | okvis_Suave.tum (2475 poses) | 376 KF / 375 edges | max 2.9e-14 m, PASS |
| offline | okvis_Escaleras.tum (5184 poses) | 747 KF / 746 edges | max 4.8e-14 m, PASS |
| live | CASA1_Suave bag (80 s, OKVIS live) | 2315/2315 samples | max 0.000000 m, PASS |
| live | CASA1_Escaleras bag (165 s, multi-floor) | 6079/6079 samples | max 0.000000 m, PASS |

The live comparison is fused-vs-provider **within the same run**, so it is
invariant to OKVIS's own run-to-run nondeterminism — what it proves is the
chain math + plumbing, exactly P-A's scope. (Run 2 of Suave repeated the PASS —
see results/pa/.)

**Where to look when it misbehaves:** `results/pa/<bag>/launch.log` (OKVIS +
node output), `provider.tum` empty → topic/QoS mismatch (is OKVIS publishing?
`ros2 topic hz /okvis/okvis_odometry`), `fused.tum` diverging → edge math or a
global constraint landed when none should.

**Next (P-B):** EigenPlaces reloc as the first global-constraint source into
this same pose-graph; then optimize() moves off the callback thread.

---

## 2026-06-12 — P-B step 2a: KF images + EigenPlaces + sealed VPR submaps in the provider chain

`provider_fusion_node` grew the P-B capture path (param-gated, `image_topic`
empty = pure P-A): infra1 image ring buffer (0.6 s) → nearest image per chain
keyframe (±60 ms) → **EigenPlaces TRT** (wrapper reused from slamko_vio_core —
composition-root privilege; `slamko_vio` now exports the vendored
tensorrtbuffer headers) → per-KF `global_descriptor` → **sealed SMP submaps**
(`kf_per_submap`=50, anchor = first KF of segment, manifest maintained,
trailing partial sealed in the destructor on clean SIGINT).

**Live validation (CASA1_Suave bag, `vpr:=true`):** 5 submaps / 250 KF /
**VPR coverage 100%** (`smap_info` hard gate OK), sane anchors. Caveats found
and handled: (a) first run pays ~30 s building the EigenPlaces TRT engine
(GPU contention delays OKVIS init) — one-time, cache at
`/tmp/slamko_vio_eigenplaces_512.engine`; (b) bench teardown now SIGINTs
before killing so the trailing submap seals (bench_pa.sh).

**The architectural point:** keyframe capture starts seconds after bag start
(OKVIS init), so the 133 s start-room data hole that killed the old
magistrale bridge cannot recur. **Next (P-B step 2b):** XFeat features per KF
→ retrieval top-10 (per the step-1 verdict) + LighterGlue/PnP verify →
reversible gated anchor edge → optimize off-thread.
