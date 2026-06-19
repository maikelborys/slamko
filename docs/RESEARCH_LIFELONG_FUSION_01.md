# Research synthesis — coherent lifelong cross-session fusion (2026-06-19)

5 parallel research agents (ORB-SLAM3 Atlas, RTAB-Map, lifelong map-maintenance,
cross-session graph fusion, robust back-ends). Strong cross-corroboration. This doc
is the decision record + the implementation program. Companion: `PLAN_BRUTAL_RUNS_VIZ_01.md`.

## The validated finding (the "doubling" root cause)
A revisit must add a **weighted RELATIVE pose-graph edge** into the prior map's anchors and
**re-optimize** — NOT re-base the whole session with one rigid SE3. A rigid SE3 has 6 DOF; intra-
session drift is an error field that GROWS with path length (N×6 DOF). 6 DOF can null the residual at
exactly one place (the match) while it grows away from it → the drifted tail lands offset → **doubled
copy**. Confirmed independently by ORB-SLAM3 (welding-window BA + essential-graph PGO, welding KFs
fixed), maplab (`aam` rigid anchor is ONLY init; `relax` adds relative edges = required 2nd stage),
Cartographer (frozen prior trajectory + many inter constraints), Kimera-Multi (between-edges,
co-optimized), RTAB-Map (inter-session loop edge + graph optimization distributes drift).

slamko already has the backbone (iSAM2 poses-only, anchor-don't-weld, PCM consensus). The gap: the
cross-session match only updates a rigid `T_global_map_`; it does NOT enter the optimized graph.

## Edge-covariance design (the actionable core — error flows to lowest-information edges, `Σ eᵀΩe`)
| Edge | Information / covariance | Robust kernel |
|---|---|---|
| **clean chain** (provider rel-pose, normal track) | Σ from OKVIS rel-pose cov — SMALL = the reference stiffness (today it's a flat sigma → wrong) | none / light Huber |
| **loss-bridged soft** (created on tracking-loss) | Σ **inflated by integrated gyro/odom uncertainty over the gap** — rotation σ∝√Δt (ARW), translation grows with distance; accumulate per-segment. **Today = SAME as clean = the bug.** Must be reliably softer than clean, far softer than reloc. | none (true measurement, just uncertain) |
| **internal loop** (same-session, verified) | Σ from registration/PnP covariance (tight when inlier-rich) | GNC/TLS or DCS |
| **cross-session weld** (reloc to prior) | Σ from cross-session registration cov (tight → must WIN and snap the tail). Prior anchors **held fixed = gauge**. | **PCM pre-filter** (max-clique χ²0.99) → survivors full weight; GNC/TLS in-loop backstop |
Only the *ratios* matter (scaling all Ω doesn't change argmin) — pick one consistent scale.

## Lifelong = recognition-gated mapping (the "localize where mapped, SLAM where new" backbone)
Experience-Based Navigation (Churchill & Newman) + ORB-SLAM3 Atlas: **try relocalization first; seal a
new submap only on sustained reloc failure OR genuinely new viewpoint/area** (covisibility-coverage
gate, e.g. ORB-SLAM "<90% of reference points tracked"). Emergent: map converges with appearance
VARIETY, not traversals. SLAM stays ON by default; new-submap creation is gated. This fixes the
duplication (brutal sealed 13 submaps of suave's house).

## The implementation program (priority order)
- **A — cross-session reloc = weighted edge in iSAM2, not rigid re-base.** [fixes doubling] Add the
  prior submap anchor as a FIXED node in the session graph (gauge) + a loop/between edge from it to the
  current keyframe with registration covariance; optimize() bends the session onto the fixed prior.
  (Or the GPS-style per-node PRIOR factor on the current KF at the prior-derived global pose.)
- **B — loss-bridged edge actually soft.** [makes it yield] Inflate its sigma by the DR-gate's measured
  d_rot/d_trans (or integrated gap uncertainty, σ∝√Δt). = task #12. Clean chain stays stiff.
- **C — new-submap creation gated by recognition + coverage.** [fixes duplication] Don't seal in an
  area a prior submap already covers (PR match verified); seal only in novel viewpoint/area.
- **D — redundant-submap culling/merge backstop** (ORB-SLAM 90%/≥3 lifted to submaps + Reduced Pose
  Graph "attach-don't-duplicate"). [guarantees area-bound regardless of recall]
- **E — proximity detection** (geometric match by OPTIMIZED pose, viewpoint-independent) — RTAB-Map's
  answer to slamko's exact "viewpoint cliff"; recovers opposite-heading revisits WITHOUT swapping VPR.
- **F — robust: PCM (have) → GNC/TLS in-loop; prior FROZEN as authority.** Smoothness: feed correction
  through map→odom (REP-105; bridge already does). Dynamic worlds: per-landmark Persistence Filter (later).

**Doing now: A + B**, then re-run brutal1-revisit and measure (p90 NN dist should drop from 1.04m; the
ghost should collapse). Then C.

## RESULT (2026-06-19) — A+B BUILT + correct, but doubling needs E (proximity detection)
A (`PoseGraph::addPriorFactor` unary robust + cross-session adds prior+optimize, gate decoupled so
cross-session bypasses the same-submap PCM streak) and B (chain edge soft ONLY on `seg_was_hard_loss`,
not missing-VPR-images) are implemented, reversible (`xsession_prior_factor`), and work WHERE matches
exist. **They did NOT collapse the doubling** — root cause found:
- The 15 prior factors landed at kf 5–271 (top) and 776–786 (return) — a **GAP kf 271→776 with ZERO
  matches**, which is EXACTLY the drift region (the jolts + far end). The jolts → motion blur → VPR
  retrieves nothing there, so NO constraint (rigid OR prior-factor) can correct the drift there.
- Evaluation was also confounded by **OKVIS GPU-contention nondeterminism** (a VPR-on run diverged to
  y=23 m; provider.tum, untouched by our code, was the diverging signal). VPR-on runs need rate≤0.5
  AND a provider-stability check before any ATE/alignment number is trusted.
- **Conclusion:** A+B are necessary FOUNDATION (correct, harmless, reversible) but NOT sufficient. The
  doubling unlock is **E — proximity detection**: once roughly localized, find prior submaps near the
  current OPTIMIZED pose and verify geometrically (XFeat/ICP), independent of VPR retrieval → fills the
  match gap → prior factors there → drift corrects. (RTAB-Map's `RGBD/ProximityBySpace`, the exact
  answer to slamko's viewpoint/recall-dead-zone problem.) **NEXT = implement E.**

## E SHIPPED + VALIDATED (2026-06-19) — proximity detection works
`XFeatRelocalizer::relocalizeNear(query, T_query_global, radius)` (refactored the per-candidate verify
into `verifyAgainst`; candidates picked by anchor-distance, NOT VPR cosine) + wired in `tryRelocalize`
(`proximity_radius` default 3 m) so once localized it ALSO geometric-verifies prior submaps near the
estimated global pose. Feeds the existing `addPriorFactor` (A). **Controlled A-vs-A+E (same rate 0.5,
stable OKVIS):** matches 15→45 (3×), **13 of them in the previously-EMPTY kf 271–776 recall-dead gap**
(kf 273/593/600 etc); alignment to suave **median 0.68→0.48, mean 0.71→0.52, p90 1.41→1.01 (−28%)**;
certainty (verified-vs-original) **24%→31%**. As predicted: the regions with real geometric overlap
connect+align; the genuinely-overlapless excursions (bottom y=−7..−9) **stay DANGLING — honest, not a
fake-coherent double**. This is the user's accepted model: some islands firm, some hang.
**Remaining refinements:** #12 (inflate the loss-bridged soft edge by the DR-gate magnitude — B fixed
the all-soft mislabeling but not yet the magnitude scaling); C (suppress duplicate-submap sealing in
covered prior regions); D (redundant-submap culling backstop). E is the headline coherence unlock; done.

## Key sources
ORB-SLAM3 Atlas (Campos 2021, arxiv 2007.11898) · maplab 1.0/2.0 (1711.10250, 2212.00654) · RTAB-Map
(Labbé & Michaud, T-RO 2013 + JFR 2019; proximity detection, ReduceGraph, IncrementalMemory) ·
Cartographer (Hess 2016) · Kimera-Multi (2106.14386) · EBN (Churchill & Newman IJRR 2013) · PCM
(Mangelson ICRA 2018) · GNC/TLS (Yang RA-L 2020) · switchable constraints (Sünderhauf 2012) · DCS
(Agarwal 2013) · IMU preintegration cov (Forster TRO 2016) · Reduced Pose Graph (Johannsson ICRA 2013)
· GLC/NFR sparsification (Carlevaris-Bianco TRO 2014; Mazuran RSS 2014) · Persistence Filter (Rosen
ICRA 2016) · REP-105.
