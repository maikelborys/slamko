# Plan — persistent MapPoints + re-association (the "no-doubling, immortal map" refactor)

## STATUS 2026-06-20 EOD (read this first)
- **P1 DONE** (commit 2a22c59): `RelocResult.matches` surfaces the inlier (query-feature,
  submap-landmark-id) correspondences. Validated: brutal rate0.5 LOOP CLOSED inliers=100
  assoc=100; 194 landmarks re-recognised on revisit (proof: `scripts/plot_assoc.py`, yellow).
- **P2 occ-refresh = NO MEASURABLE EFFECT** (commit 6f390a0, kept reversible). Clean A/B
  (occ_refresh on vs off): single-session 16 submaps both; cross-session (suave on its own
  prior) **137 localized, 2 duplicate submaps, 6 culled — IDENTICAL both ways.**
- **THE BIG REFRAME (honest):** the dramatic "doubling" the user hated was **OKVIS DIVERGING
  at rate 1.0** (provider y-span 23m vs 11.6m at 0.5, same bag), NOT slamko's mapping. At
  rate 0.5: single-session map is COHERENT (`render_map_corrected.py`: frozen vs loop-
  corrected anchors near-identical), and cross-session ALREADY de-doubles (existing cull
  backstop + prior occupancy → only 2 leaks / 137 localizations, co-located not offset).
  → The doubling problem is MOSTLY SOLVED. The full MapPoints refactor (B, P3 below) is a
  big build for a small remaining problem — **MEASURE first (EuRoC ATE) before investing.**
- **NEXT (user-set sequence): (1) EuRoC Machine Hall with BLACKOUTS (VIO / VIO+IMU dropout),
  ATE vs GT, then (2) the refactor IF the ATE reveals a real problem.**

## NEXT TASK — EuRoC MH cross-session + blackout harness (build this)
The validation the user wants ("todo completo"): EuRoC Machine Hall, overlapping sessions
(MH mid→end as session 1, then full as session 2 → watch them FUSE), with **ATE vs ground-
truth** + **rotatable Plotly landmarks** + **blackout tests** (cover VIO → VIO+IMU bridges).
Pieces that EXIST: bags `/mnt/data/euroc_bags/mh_0{1,3,5}_okvis`, GT under
`/mnt/data/datasets/euroc/MH_0*/mav0/state_groundtruth_estimate0`, OKVIS-EuRoC launches
(`~/coding/euroc_publisher/launch/okvis_euroc_bag.launch.py` → /euroc/cam{0,1}, /euroc/imu0
→ /okvis/okvis_odometry), `~/coding/RTABmap/euroc_ate.py`. Pieces to BUILD:
- a `pa_okvis_euroc.launch.py` (OKVIS-EuRoC + provider_fusion with EuRoC cam topics +
  EuRoC intrinsics as PARAMS — EuRoC publishes no CameraInfo; cam0 is 752×480 = XFeat-native).
  EuRoC raw is distorted (radtan) → needs rectified images for slamko's PnP/stereo
  (`euroc_rectify_pairs.py` exists) OR feed rectified intrinsics.
- a 2-session runner (session1 segment → map; session2 full with prior_map_dir → fuse) + ATE
  (euroc_ate.py) + the Plotly landmark render (`plot_map3d.py` / `plot_assoc.py`).
- blackout injection: `force_loss_start/end` already drops odom; for a VISUAL blackout drop
  the image topic window; test VIO-only vs VIO+IMU bridge behaviour.



**Decision (2026-06-20, user):** adopt **option B — persistent MapPoints with cross-visit
re-association (ORB-SLAM3 core), inside slamko.** This is a DELIBERATE break from the
MASTER_PLAN "iSAM2 over poses/anchors ONLY — no landmarks ever" rule: the user's hard
requirement (a map that is *coherent, geometrically straight, and refines instead of
doubling on revisit*) cannot be met by a pose-graph over frozen submaps. It needs landmark-
level data association. Cold-start this doc + [[slamko-persistent-mappoints-decision]].

## The root cause the user identified (verbatim: "cuando revisita zona no se detecta mismos landmarks")
A coherent revisit-refined map needs the camera to **re-observe the SAME 3D points** on
return and **add observations** to them (so they refine, not duplicate). slamko today seals
each submap with its OWN per-pass landmarks (fresh triangulation every visit) — there is no
persistent point identity across visits. On revisit, accumulated VIO drift offsets the new
landmarks from the originals (more than the 0.10 m dedup voxel) → a **second offset submap is
sealed** = the doubling. The loop closure corrects the *trajectory* but the already-sealed
submaps are frozen with their old anchors → the two copies stay offset.

**A merge that only voxel-dedups after alignment is a band-aid** — without re-associating the
SAME points by descriptor, you always fight the doubling. The missing abstraction = persistent
MapPoints (ORB-SLAM3's `MapPoint`: a 3D point + descriptor + the list of keyframes that
observed it).

## The load-bearing nuance (makes this cheaper than a from-scratch ORB-SLAM3)
The data association is **already computed and thrown away.** On every loop the relocalizer
runs PnP and finds inliers = correspondences between the query's features and an existing
submap's landmarks (65–131 points matched on the brutal bag). slamko uses that ONLY for the
pose-graph loop edge (`processRelocResult` → `addLoopEdge`) and **discards the per-landmark
correspondences for the map.** Surfacing those correspondences from the relocalizer is the
hook for the merge/fusion.

## Phased plan (build + validate each phase with a top-down map PNG)
- **P1 — surface the reloc correspondences.** Make `XFeatRelocalizer::relocalize/verifyAgainst`
  return the matched (query-feature ↔ submap-landmark-id) pairs, not just the inlier count +
  pose. (Today they're computed inside PnP-RANSAC and dropped.)
- **P2 — MapPoint identity + observations.** Promote submap landmarks to persistent MapPoints
  with an observation list (which keyframes saw them). On a loop, the matched query features
  attach as NEW observations of the EXISTING MapPoint (don't create a duplicate). Unmatched
  query features → new MapPoints.
- **P3 — fuse + refine on revisit (kills doubling).** Merge matched MapPoints (one copy);
  multi-view refine their position from all observations (light local BA or robust averaging,
  structure-only, poses from the graph). The revisit makes the map MORE accurate, not doubled.
- **P4 — Atlas / multi-session (the user's reuseMap=1 flow).** Maps as an atlas; load prior
  maps; on revisit, the MapPoint re-association drives cross-session merge. Sessions 1→2→3→4
  accumulate into one coherent immortal map.

P1+P2+P3 kill the within-session doubling (the immediate pain). P4 is the lifelong multi-session
layer.

## Today's findings (context for the next session — don't re-derive)
- **The "bent corridor" was OKVIS DIVERGING at rate 1.0**, NOT slamko. Same brutal bag:
  provider y-span 23 m @rate1 vs 11.6 m @rate0.5 (2× = divergence), half the poses. **Always
  run VPR-on at rate ≤0.5 + check provider.tum y-span (~[-8,2]); the live viewer adds GPU load
  → keep it light.** At rate 0.5 the map IS coherent (a corridor + rooms, loop closes).
- **Compass yaw-prior = marginal indoors-with-loops** (clean A/B at rate 0.5: compass-on vs off
  maps nearly identical). VIO yaw was already good (innov 2–8°) + loops correct drift → nothing
  to fix. Shipped gated + OFF by default (commit e07f0f4); payoff is loop-free/outdoor + long
  vision-loss. NOT the doubling fix.
- **Live Rerun viz works** (web viewer at `http://127.0.0.1:9090?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy`,
  native `rerun_cli/rerun --serve-web`, NOT the pip wrapper). Trajectory re-log throttled to
  every 8 poses (was O(N²), choked it). `scripts/render_map_png.py` = the lightweight offline
  judge (no viewer).

## How to run (reference)
```bash
# coherent baseline (OKVIS stable, no viewer):
VPR=true scripts/bench_pa.sh /mnt/data/bno_ab/CASA1_brutal1_Stereo60_RGB30_BNO_trim results/viz/x 0.5
python3 scripts/render_map_png.py --run-dir results/viz/x --out /tmp/x.png   # judge it
```
