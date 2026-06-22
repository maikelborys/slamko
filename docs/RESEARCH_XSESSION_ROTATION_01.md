<!-- validated: fix SHIPPED + validated 2026-06-22 (d5719dd) — rotation 18.2°→0.0° -->

## ✅ FIX SHIPPED + VALIDATED (2026-06-22)

Implemented in 3 steps and validated end-to-end on casa40+casa100:
- **Step 1+2** (`baea772`): `PoseGraph::setFixed()` (multi-fixed gauge) so a loaded prior
  map's anchors stay rigid. The relative-between machinery (`addEdge`/`addLoopEdge` + Huber)
  already existed. Unit tests in slamko's own Ceres solver: ≥2 separated between-edges
  distribute the session's drift onto the fixed prior (far-end < 0.15 m, prior rigid); 1
  match leaves a drift tail (>0.3 m), 2 collapse it to <0.4× — proves the ≥2-matches insight.
- **Step 3** (`d5719dd`, `provider_fusion_node.cpp`): TWO bugs. (a) **Save frame** — the
  archive persisted `graph_.pose()` (SESSION frame) without `T_global_map_`, so a cross-session
  map sat `T_global_map_`-rotated from its prior (the visible ~18°). Fixed: save
  `T_global_map_ * graph_.pose()` (GLOBAL; single-session unchanged). (b) **Unary→binary**
  (`xsession_between_factor`, default on): each match adds a relative BETWEEN edge to the prior
  submap anchor held as a FIXED node, robust Huber, rotation σ inflated by 1/√inliers; dropped
  the frozen-frame + jump-gate that rejected corrective matches.

**Result (casa40 prior + casa100, 78 between-edges over 54 distinct keyframes):**
ICP rotation **18.2° → 0.0°**; residual **0.38 m → 0.06 m** (= the best-possible rigid fit =
voxel/HITNet reconstruction noise; **< nvblox truncation 0.2 m → clean fusion, bounded
voxels on revisit**). Combined single-volume map **500k → 405k verts** (doubling largely gone).
HONEST: both sub-fixes contribute — (a) removes the gross rotation, (b) distributes residual
drift; an A/B with `xsession_between_factor:=false` to isolate each is the remaining check.

# Cross-session map rotation — root cause + fix (research)

> **Symptom (user-spotted):** running casa100 with casa40 as prior, the combined map
> shows casa100 **rotated ~18° + offset** from casa40, residual **growing with distance**
> from the start. "No es normal — está directamente girado." Correct: this is a design
> bug, not inevitable drift.

## Three converging evidences for the root cause

1. **Empirical** (`scripts` ad-hoc ICP): aligning casa100→casa40 needs **ROTATION −18.2°**
   + 0.28 m translation; residual 0.38 m → 0.18 m after. Residual-vs-distance-from-start
   correlation **+0.56**; heatmap dark at the salon, bright (~0.5 m) at the extremities =
   lever-arm fan.
2. **Code** (provider_fusion_node.cpp ~1504–1599): on a cross-session match slamko adds a
   **UNARY per-keyframe `addPriorFactor(q_id, target_session, …)`** (Mode A,
   `xsession_prior_factor_` default on); `T_global_map_` is **FROZEN after the first match**
   (line ~1526); the accumulative **2-vote consensus refines TRANSLATION only, not rotation**
   (line ~1561). `T_global_q = prior_anchor_[submap] * r.T_query_match` (line 1514) takes the
   prior anchor's orientation as fixed truth.
3. **Theory** (pose-graph LS, gauge freedom): a unary prior is a **soft rigid SE3 re-base**
   toward an *externally estimated* frame Ĝ = G\*·Δ. The factor's residual is Log(Δ) = (δφ,δt);
   the rotation error **δφ is the REFERENCE, not a variable being optimized**. More priors
   reduce *variance* but not the *shared bias* δφ (every prior inherits the same frozen Ĝ).
   Surviving residual rotation × lever arm gives position error **‖Δp‖ ≈ ‖δφ‖·‖p−c‖** — grows
   with distance (1° → 1.7 cm/m → 35 cm at 20 m). Exactly the observed fan.

**Where δφ comes from:** the relocalizer's **PnP relative rotation is biased by the 40 cm vs
100 cm viewpoint** — fewer inliers + a biased 3D support subset (features visible/triangulable
from both heights) → a *systematic* rotation bias (not just noise) in the SVD/PnP solution.

## The fix — UNARY prior → BINARY per-match relative BETWEEN factor

A unary prior re-bases toward a possibly-biased frame and can't correct an error that lives
in the reference. A **binary relative factor** `e = Log(z_ab⁻¹ · T_a⁻¹ T_b)` between the
matched prior keyframe `a` and the query keyframe `b` makes the inter-map rotation a
**directly observed, gauge-fixing, unbiased** quantity (Jacobian non-zero w.r.t. both R_a,R_b;
it spans the gauge-null rotation mode the prior couldn't fix). Multiple spatially-diverse
matches then **average down** the per-match rotation noise (legitimate — independent looks,
unlike the shared prior bias) and **distribute** the correction through both pose chains,
absorbing each session's internal (non-rigid) drift.

**Recipe (theory §8, to implement in slamko):**
1. Per match: PnP/Umeyama-Kabsch **inside RANSAC**; record inlier count + spatial spread;
   reject coplanar/clustered/thin supports; set per-match covariance — **inflate rotation
   covariance when support is weak** (Hard rule #3: covariance, not `if`).
2. Add each match as a **binary BETWEEN factor** between the prior-keyframe node (or a fixed
   prior-submap anchor node) and the query node, in **one joint graph** — not a unary prior,
   not a single rigid Sim3 weld.
3. **Anchor session 1 as the gauge** (fix one pose); leave session 2 + cross-edges free.
4. **Robustify** cross-edges with **GNC** (or DCS / switchable constraints / max-mixtures) —
   auto-rejects wrong + viewpoint-biased matches.
5. **Co-optimize** (iSAM2 / LM over SE3/Sim3) → distributes correction, absorbs drift.
6. **Re-render the dense map from the optimized poses.**

A single rigid Sim3 weld is enough ONLY if both sessions are drift-free; with internal drift
you need many distributed cross-edges (non-rigid) — i.e. treat inter-map matches as ordinary
robust loop closures in one joint graph (maplab / Kimera-Multi / RTAB-Map multi-session
practice; ORB-SLAM3 Atlas = Sim3 weld + welding-window local BA).

## slamko change surface
- `provider_fusion_node.cpp` cross-session handler: replace/augment the unary
  `addPriorFactor(q_id, …)` with a binary relative factor between the matched prior keyframe
  and `q_id`. Requires the prior submap's keyframes to be nodes (or a fixed anchor node per
  prior submap) so a between-edge can attach.
- `slamko_loop/pose_graph.{hpp,cpp}`: ensure a robust BetweenFactor across components +
  a robust kernel (GNC/DCS) for cross-edges; gauge = fix session-1.
- `xfeat_relocalizer.cpp`: surface inlier count + 3D support spread per match → covariance.
- Consensus: refine ROTATION too (currently translation-only), or rely on ≥2–3 spatially
  diverse between-factors to fix it via co-optimization.

## SOTA confirmation (3rd agent) — what mature systems actually do

**THE crux (most actionable):** removing the rotation needs **≥2 spatially-separated
inter-session matches**, co-optimized. ONE match is degenerate — gauge-equivalent to a soft
prior, aligns a single point. TWO well-separated relative constraints **over-determine** the
relative SE3 (a rigid body has only 6 DoF); with internal drift no single rigid offset
satisfies both, so the optimizer's only way to cut cost is to **rotate AND bend both
trajectories** → inter-map rotation driven to zero, residual distributed. This is the precise
reason slamko's per-keyframe UNARY prior fails (each is a 1-point gauge pull) and the precise
fix (per-match BINARY between-factors).

**Canonical pattern (ORB-SLAM3 Atlas `MergeLocal`/`MergeLocal2`):** rigid Sim3/SE3 init
(Horn on 3 matches, RANSAC) is ONLY initialization → **welding-window local BA** jointly
re-optimizes both maps' covisible keyframes + fused points (this physically removes the
rotation in the overlap; duplicate points fused → no doubling) → **whole-map essential-graph
PGO** keeping the weld fixed (spreads the residual across both full trajectories). Order is
the point: rigid-init → local-BA-removes-rotation → graph-PGO-spreads-residual → re-render.

**Same skeleton everywhere:** RTAB-Map (loop + proximity links → GTSAM/g2o co-deform both
graphs; Vertigo robust); maplab `relax` (relative 6-DoF edges → global opt → `optvi` VI-BA);
Kimera-Multi/RPGO (GNC robust two-stage PGO, replaced PCM); COVINS-G (per-loop PGO from
2D-2D RANSAC relative pose). NONE uses a single global prior to merge.

**Explicit verdict on slamko's adopted design:** the "A = weighted PRIOR FACTOR not rigid
re-base" ([[slamko-lifelong-fusion-ABE]]) is still the WRONG factor type — a prior (even soft,
even per-keyframe) constrains B's ABSOLUTE GAUGE; it carries zero information about B's shape
RELATIVE to A, so it provably leaves the rotation + distance-growing residual. Convert the
cross-session matches from prior factors to **per-match robust BetweenFactors at ≥2 separated
keyframes**, co-optimize (iSAM2/LM), gauge = one anchored session-1 node.

## Sources
Grisetti pose-graph tutorial (gauge/node-fix); g2o (Kümmerle 2011); Umeyama 1991 / Horn 1987
(Kabsch/Sim3 closed-form + reflection fix); Switchable Constraints (Sünderhauf 2012); DCS
(Agarwal 2013); Max-Mixtures (Olson 2012); GNC (Yang/Carlone 2020); PCM (Mangelson 2018);
iSAM2 (Kaess 2012). Systems: ORB-SLAM3 Atlas (T-RO 2021), RTAB-Map multi-session (JFR 2019),
maplab 2.0, Kimera-Multi/RPGO, COVINS/COVINS-G. Dense re-warp: Sumner embedded-deformation
2007, ElasticFusion (RSS 2015), Hydra (RSS 2022), BAD-SLAM.
