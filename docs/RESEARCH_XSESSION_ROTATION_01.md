<!-- validated: diagnosis 2026-06-22, fix not yet implemented -->
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

## Sources
Grisetti pose-graph tutorial (gauge/node-fix); g2o (Kümmerle 2011); Umeyama 1991
(Kabsch/Sim3); Switchable Constraints (Sünderhauf 2012); DCS (Agarwal 2013); Max-Mixtures
(Olson 2012); GNC (Yang 2020). ORB-SLAM3 Atlas, RTAB-Map multi-session, maplab, Kimera-Multi
(SOTA agent — pending).
