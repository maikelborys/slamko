# Plan — persistent MapPoints + re-association (the "no-doubling, immortal map" refactor)

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
