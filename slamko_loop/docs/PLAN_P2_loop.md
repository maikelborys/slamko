# slamko_loop — P2 plan (the never-lost supervisor)

<!-- validated: (P2a) 2026-05-27 · tests: 10 gtest 0 fail (synthetic, no ROS) -->

Read [`../../docs/SYSTEM.md`](../../docs/SYSTEM.md) (never-lost spine) +
[`../../docs/DECOUPLING.md`](../../docs/DECOUPLING.md) + [`../README.md`](../README.md)
first. Progress + numbers: [`STATUS.md`](STATUS.md).

## Goal
The Tier-3 never-lost spine (slamko's flagship): keep producing odometry through
tracking loss and re-anchor on revisit, **never** letting a corrupted estimate or a
false relocalization corrupt the map. Architecture = submaps (OKVIS2-X/GLIM) +
ORB-SLAM3 multi-map archive-restart + DigiForest decoupling (the supervisor is a policy
OUTSIDE the estimator graph). Loss = odometry **stale-gap**, not covariance.

## Phases
- **P2a ✅** — the supervisor policy, **no solver** (closed-form SE3 weld):
  `NeverLostSupervisor` (state machine + map→odom owner) + `SubMapArchive` (seal/branch
  Atlas) + `AnchorGate` (multi-cluster lazy-anchor weld gate) + the `Relocalizer` seam
  (tested with an injectable stub). 10 synthetic gtests, 0 fail. See STATUS.
- **P2b** — **XFeat relocalizer**: a `slamko_core::Relocalizer` impl that matches a query's
  XFeat descriptors against archived submaps' N×64 XFeat index (already produced by
  `slamko_vio::buildSubMap()`) + PnP/RANSAC geometric verification → emits `RelocResult`
  feeding the weld gate. No new model. LiftFeat-m1 = future swappable option.
- **P2.5** — loop-closure-as-factor + a tiny self-contained SE3 pose-graph solver over
  submap anchors (Gauss-Newton, uses only `se3.hpp`; try/catch → damp → drop-edge,
  Hard Rule #4). The v1 stores the edge data model so this adds only the solve loop.
- **Deferred (opt-in plugin):** dense submap-to-submap alignment `Factor` (the OKVIS2-X
  `useMap2MapFactors` mechanism — IoU overlap → dense ICP residuals) for
  forest/repetitive/appearance-change robustness where visual place-rec is weak. Needs a
  dense payload (`SubMap.custom_data`); drops into the global pose-graph via the
  pluggable `Factor` contract with **zero core changes**. slamko core stays sparse.

## Locked decisions
No solver in v1 · loss = stale-gap (≥ VIO `dr_max_s_`) · weld gated OUTSIDE the graph,
multi-cluster lazy-anchor · `T_map_odom = active.anchor = S.anchor · T_query_match` ·
sparse core, dense as an opt-in factor · depends on `slamko_core` only (Hard Rule #2).

## Key files
`include/slamko_loop/{supervisor_state,submap_archive,anchor_gate,never_lost_supervisor}.hpp`
· `src/{submap_archive,anchor_gate,never_lost_supervisor}.cpp` · `test/test_supervisor.cpp`.

## Verify
```bash
cd ~/coding/slamko
colcon build --packages-select slamko_loop --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select slamko_loop && colcon test-result --test-result-base build/slamko_loop
```
Deterministic (synthetic inputs) — no ROS/rosbag2, sidesteps the box's flaky run-harness.
