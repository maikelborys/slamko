# slamko methodology + roadmap — build the TRUNK, hang the leaves (2026-06-21)

Why this doc: the user asked "what's the best way to build a SLAM framework, and is our iterative
order right?" (instinct: "no está bien parece"). Research (GLIM, OKVIS, Cartographer, maplab,
RTAB-Map, Kimera + GTSAM/iSAM2) → the answer below. Captures the REFINED vision + the validation
protocol + the multi-session isolation rule.

## The vision (user, verbatim intent, 2026-06-21)
> Use ANY odometry. Know when tracking is BAD or LOST → seal and start a NEW map when tracking
> recovers. While mapping, if we find COINCIDENCES with other maps → FUSE them together
> geometrically. Validate on EuRoC: replay bags from different times, STACK them, check the fusion
> is geometrically correct. First traversal = reference; subsequent ones = compared against it.

This IS the Atlas multi-map + never-lost + geometric-merge spine. It is the "trunk", not a leaf.

## The principle the research converges on
Every reference framework is LAYERED and was built bottom-up, each layer validated before the next.
The **global factor graph is the unifying abstraction** — everything reduces to "add a factor".
GLIM = GTSAM backbone + callback-slots for extension; GTSAM/iSAM2 = incremental factors; OKVIS/
maplab = front-end → windowed BA → global graph → multi-session. The load-bearing rule:

> **Don't build a feature until the layer it depends on is solid.** Persistent-MapPoint refine,
> cross-session merge, GPS — all of them only pay off once the global graph DISTRIBUTES corrections.

## The honest diagnosis (what "no está bien" actually is)
Architecture (6 decoupled packages + Factor/Provider/Relocalizer contracts) is CORRECT — matches
GLIM/maplab. We don't rebuild it. BUT we've been iterating on LEAVES (viz, compass, MapPoints,
EuRoC harness) while the TRUNK is half-built. Our own docs flag it:
- The global backend is a hand-rolled Ceres PoseGraph, NOT the iSAM2-poses-only of the MASTER_PLAN
  (P-C′ deferred).
- **Corrections are applied as an OUTPUT TRANSFORM (`T_global_map_`), not as anchor-edges INSIDE the
  graph.** A loop/reloc re-anchors at points but does NOT distribute the correction across the whole
  trajectory.
- Soft-edges don't carry the DR-gate covariance (#12 pending).

Proof it bites: brutal-on-suave (2026-06-21) localized but the map stayed WARPED — the loop anchored
it but could not un-bend the drift, because the soft/anchor edges don't constrain the graph with
real covariance. That is a TRUNK gap, surfaced while polishing leaves.

## The ladder (do in order; validate each before the next)
0. **Eval harness** — EuRoC ATE/RPE + cross-session consistency + un-aligned divergence/health. ✅
1. **🎯 THE TRUNK — global graph that distributes corrections:**
   1a. Corrections as **real anchor-edges in the pose-graph** (not an output transform) → a loop
       spreads across the whole trajectory (= un-warps the brutal run).
   1b. **Per-edge covariance**: chain stiff · **soft inflated FROM the DR-gate measurement (#12)** ·
       loop robust-kernel. So a loud loop pulls degraded segments straight.
   1c. **Atlas merge as a graph operation**: tracking-lost → seal + start new map (new anchor
       component) → cross-map match → add an inter-map **anchor-edge** (covariance from match
       inliers) → the two maps fuse geometrically when the graph optimizes.
   1d. (stretch) migrate to **iSAM2 poses-only** incremental (MASTER_PLAN P-C′); minimum = the Ceres
       pose-graph properly covariance-weighted with the edges above.
2. **Lifelong / map-quality on the correct trunk:** persistent MapPoints (Phase A done, B/C next),
   cross-session landmark merge — they fuse coherently only once 1 is solid.
3. **Scope expansion:** GPS/compass-yaw factor, out-of-core map, semantics.

## EuRoC stacking validation protocol (the user's test, GT-backed)
EuRoC MH_01/03/05 = the SAME Machine Hall at different "times" → the canonical multi-session stack,
and unlike casa it has GROUND TRUTH → real geometric ATE, not just consistency.
- **Run 1 (reference):** MH_01 → build atlas map A. ATE_1 vs GT = the per-session baseline.
- **Run 2 (stack):** MH_03 with prior=A → detect tracking-lost/new-map as needed, match into A,
  add inter-map anchor-edge, optimize. Measure: (a) ATE of the fused result vs MH_03 GT; (b)
  GEOMETRIC FUSION error = align A and the MH_03 map into the common GT frame, report the
  inter-map drift at the merge (should collapse toward 0 after the anchor-edge optimizes).
- **Run 3 (stack):** MH_05 onto {A, MH_03}. Same metrics. The map must stay BOUNDED + coherent.
- Pass = each stacked session's fused ATE ≈ its solo ATE, AND the inter-map seams are geometrically
  consistent (no doubling, no warp) under GT.
- **Rate:** EuRoC is OKVIS-stable → rate 1.0 fine (the rate≤0.5 rule is the casa Stereo60 + harsh-
  motion issue, not EuRoC). Run VPR-on still ≤0.5 only if GPU-contended by a concurrent session.

## Multi-session isolation rule (other Claude Code runs KLT_VO on EuRoC)
UPDATE 2026-06-21 (user): **KLT_VO is TOTALLY independent — it does NOT run OKVIS**, and its bags
live in `~/datasets/euroc` (mine in `/mnt/data/euroc_bags`). So collision is minimal:
- **Reaping is SAFE** — my benches reap `okvis2x_..._subscriber|provider_fusion_node`; KLT has none
  of those. Bag-player reap matches my `/mnt/data/euroc_bags/...` path, not KLT's `~/datasets`.
- **Topics**: still set `ROS_DOMAIN_ID` (42) so /euroc/* and /okvis/* never alias the other session.
- **GPU**: the ONLY real shared resource, and only IF KLT uses XFeat/TRT. Check `nvidia-smi` is free
  before a GPU-heavy run; if KLT is mid-TRT, back off. Otherwise proceed.
→ slamko EuRoC runs can proceed isolated (ROS_DOMAIN_ID + PID/path reaping + GPU-free check).
