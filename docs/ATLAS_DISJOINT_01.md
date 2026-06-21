# Atlas disjoint-islands model — break / vote / fuse / dangle (2026-06-21)

The user's model, verbatim intent:
> Use ANY odometry. When tracking is BAD/LOST → seal and start a NEW map when it recovers.
> While mapping, if FEATURES match another map → it welds AUTOMATICALLY (not spatial overlap —
> feature similarity). If not → it stays DANGLING until a later traversal brings matching
> features ("la pequeña mapa tira"). So a brutal run should render as FRAGMENTS, not one warp.

This is the honest-dangling Atlas: break on loss, fuse on a real feature match, hang otherwise.
SHIPPED + validated this session. Cold-start here + [[slamko-atlas-disjoint-islands]].

## What was built (etapas, all opt-in, all committed)
- **1a — PoseGraph per-component gauge** (`3e57803`, `pose_graph.cpp`): `optimize()` union-finds the
  edges and pins the lowest-id node of EACH connected component. A loss-broken island is then
  well-posed (floats at its own gauge, no singular solve); a weld that joins two islands merges
  the component → ONE gauge → one bends onto the other (geometric fusion for free). Unit test
  `PoseGraph.DisjointIslandsGaugedThenWeldFuses` (green).
- **1b — break on loss** (`6414179`): `atlas_break_on_loss` — a stale-gap tracking loss does NOT
  bridge the chain; the post-gap keyframe starts a NEW disjoint component (no edge added).
  Supersedes the #12 soft-bridge when ON.
- **coalesce — no slivers** (`5686286`): `atlas_min_component_kfs` (20) — only break once the active
  map matured, so clustered/end-of-run losses don't spawn useless 10-landmark sliver maps. (User:
  "otras mapas súper pequeñas no vale".) `render_atlas.py --min-lm` also drops slivers from the plot.
- **1c / 2 — render by FINAL fused component** (`48bfd2d`): `PoseGraph::connectedComponents()` +
  re-tag `components.csv` at teardown by post-weld connectivity → fused fragments share a colour,
  a never-matched fragment shows dangling. The merge already happens automatically (a feature-match
  weld + the per-component gauge); this makes it visible/measurable.
- **accumulative 2nd-vote** (`f3b3f0f`→`cceb850`→`1391d0a`): promote a WEAK proximity match (inl <
  strong-bar 40) to a weld when N consistent votes land on the SAME prior submap. Votes accumulate
  PER submap (not a single overwriteable slot); they agree on the implied session→global CORRECTION
  `T_global_q · pose(q)⁻¹` (invariant to the robot's motion — NOT the per-kf pose), with a looser
  tolerance `proximity_vote_agree_m` (2.0 m) for noisy weak matches. Params:
  `proximity_votes_needed` (2), `proximity_vote_agree_m` (2.0).
- **harness**: `pa_okvis_euroc_x.launch.py` (+ atlas_break + force_loss) · `scripts/stack_euroc.sh`
  (MH stacking) · `scripts/render_atlas.py`.

## Validation (measured)
- **suave_break — the full cycle, PROVEN** (same-bag injected blackout): 4 break-components →
  **2 FUSED maps**. Strong same-place matches (110–122 inliers, consistent) → auto-weld → the
  fragments fused back into one coherent map; ONE fragment found no match → correctly DANGLED.
  This is the user's whole model working end-to-end.
- **wall**: 3 REAL maps (3.4k/5.4k/3.4k lm), zero slivers (coalesce fix), 2 spread tracking-losses.
- **EuRoC MH stacking, GT-backed** (MH_01 ref + MH_03+blackout): S1 ATE **2.92 cm**; S2 ATE
  **10.09 cm**. The blackout fragment localized to the prior at start (kf 0, 69 inliers) but the
  post-blackout half found only WEAK matches to the prior (inl 15–36) whose implied corrections
  **scatter > 2 m** (cross-recording PnP error — different traversal/viewpoint). The accumulative
  vote correctly REFUSED to weld inconsistent evidence → the fragment DANGLED → ATE 10 cm.

## The honest conclusion (why we stopped here)
**The Atlas plumbing is COMPLETE and CORRECT.** It fuses on strong consistent matches (suave_break)
and DANGLES on weak inconsistent ones (EuRoC cross-recording) — welding > 2 m-inconsistent matches
would be a false-merge/teleport, the exact thing the never-false-merge (I2) defense must prevent.
Dangling there is the RIGHT behaviour, not a bug.

**The remaining limiter is cross-recording MATCH QUALITY (recall), NOT the Atlas.** Different
traversals of the same hall give weak, geometrically-noisy matches (meter-scale PnP error even at
15–35 inliers) — the known viewpoint-coverage ceiling. Fixing it is a separate, harder problem
(dense LighterGlue verify / more inliers / more viewpoint coverage), not a vote-threshold tweak —
4 careful iterations of the vote mechanism confirmed the matches are simply too noisy to trust.

## Process notes (learned the hard way this session)
- The manual `pa_okvis_euroc_x` teardown is fragile: children escape, a kill chain hit `exit 144`
  and left a 17-min okvis zombie + a run that never launched. A robust `bench_euroc_x.sh` (PID
  capture + reap, like `bench_pa.sh`) is worth building before more EuRoC x-runs.
- The `rtk` shell wrapper breaks compound `grep`/`pgrep` — use `python3 subprocess` for process
  checks (done throughout).
- EuRoC isolated runs: `ROS_DOMAIN_ID=42`; reap by name is safe (KLT_VO runs no OKVIS).
