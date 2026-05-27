# slamko_loop — P2 plan (the never-lost supervisor)

<!-- validated: (P2.5+hardening) 2026-05-27 · 32 gtest 0 fail (5 pose-graph + 10 stress + 2 supervisor pose-graph + weld-once) + never-lost cycle (seal→branch→WELD→recover) validated end-to-end on V1_01 XFeat forced-loss replay -->

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
- **P2b ✅** — **XFeat relocalizer** (`XFeatRelocalizer : slamko_core::Relocalizer`): NN
  descriptor match (Lowe ratio) on the submap's N×64 XFeat index + **PnP-RANSAC (P3P
  copied to `slamko_core`)** → query cam-in-submaplocal → body via the extrinsic →
  `RelocResult.T_query_match`. Supervisor weld now composes with live odom
  (`T_active_sealed = T_query_match · T_WB⁻¹`). 16 gtests 0 fail. No new model, no
  OpenCV. LiftFeat-m1 = future swappable option. See STATUS.
- **P2c ✅ (seal→branch→recover end-to-end; weld = follow-on)** — supervisor +
  relocalizer wired into the **live `slamko_vio_node`** (`enable_neverlost:=true`; node
  gains a node-only `slamko_loop` dep, core stays decoupled). MH_01 forced-loss replay
  (`dr_force_loss=[30,33]s`): the full cycle fired on real VIO health —
  OK→RecentlyLost→**SEAL+BRANCH** (stale-gap 1.15s)→Relocalizing→recover→OK. In-process
  logging, no rosbag2. **Weld ✅** (V1_01 XFeat, forced loss [25,28]s): the full
  seal→branch→**WELD**→recover loop fired — the branch re-localized against the sealed
  map via XFeat→PnP, re-anchoring map→odom (~0.2 m). Needed: supervisor stays
  Relocalizing until welded + a relocalizer DB cap (`max_db_landmarks`). Plot via
  `scripts/plot_neverlost.py`. See STATUS.
- **P2.5 ✅** — loop-closure-as-factor + a tiny self-contained SE3 pose-graph solver
  over submap anchors (`PoseGraph`, `pose_graph.{hpp,cpp}`): nodes = anchors, edges =
  relative-pose factors `Z = anchor_from⁻¹·anchor_to`, Gauss-Newton on `se3.hpp` only
  (right-perturbation, `J_r⁻¹≈I` — exact at a consistent optimum). LM damping +
  worst-χ²-edge drop = the disposable-graph robustness (Hard Rule #4); `optimize()`
  never throws. One fixed gauge + one edge reduces ALGEBRAICALLY to the closed-form
  weld, so the supervisor's `use_pose_graph` flag (default OFF) is byte-identical to
  P2c; ON, each weld becomes an edge and ALL anchors re-solve (multi-submap merge).
  22 gtests 0 fail (5 pose-graph + 2 supervisor pose-graph). See STATUS.
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
`include/slamko_loop/{supervisor_state,submap_archive,anchor_gate,never_lost_supervisor,
pose_graph,xfeat_relocalizer}.hpp` · `src/{submap_archive,anchor_gate,
never_lost_supervisor,pose_graph,xfeat_relocalizer}.cpp` ·
`test/{test_supervisor,test_relocalizer,test_pose_graph}.cpp`.

## Verify
```bash
cd ~/coding/slamko
colcon build --packages-select slamko_loop --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select slamko_loop && colcon test-result --test-result-base build/slamko_loop
```
Deterministic (synthetic inputs) — no ROS/rosbag2, sidesteps the box's flaky run-harness.
