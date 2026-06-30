# PLAN_DEPTH_ODOM_01 — depth as a 2nd odometry/geometry source (combo with cuVSLAM VIO + slamko)

> **STATUS 2026-06-29: Phase 1 (flavor b, depth ICP-to-TSDF loop closure) WIRED + VALIDATED LIVE ✅.**
> `tryDepthLoopRefine` at the XFeat loop weld; `DEPTH LOOP refine: kf 565 rms=0.072 m inliers=5158
> cond=0.0233 -> geometric edge`, `LOOP CLOSED [DEPTH-REFINED]`, optimized graph closes to 0.19 m.
> Opt-in `depth_loop_refine`. Detail in slamko_ros/docs/STATUS.md. NEXT = per-axis cov + Phase 2
> (small_gicp continuous depth-odom 2nd provider).

<!-- authored 2026-06-29 · synthesis of 2 web-research agents (libs + fusion), first-hand cited.
Motivation: the casa run's corridor "return-by-a-different-trajectory" = drift, because depth feeds
the MAP (nvblox TSDF) but NOT the ODOMETRY. Add a geometric witness. -->

**The gap (user's diagnosis, confirmed):** today the D455 depth feeds only the **map** (nvblox TSDF);
the **odometry** is the VIO provider (cuVSLAM / OKVIS = stereo+IMU), which ignores depth. So the
trajectory has no geometric constraint → in a low-texture corridor it drifts and the return path
doesn't overlap the outbound. Fix: use depth for the **trajectory** too — a geometric witness that
complements VIO. Architecturally slamko already supports it: a 2nd `OdometryProvider` → `ProviderChain`
→ between-factors in the iSAM2 graph (loose coupling, Hard Rule #3).

## Honest framing — two flavors, and which actually fixes the corridor

- **(b) Depth ICP-to-TSDF loop closure** = the TARGETED fix for the return-path symptom. On revisit,
  register the live depth to the **TSDF we already maintain** → a geometric loop constraint that snaps
  the return onto the outbound, exactly where visual place-recognition fails (low texture, opposite
  viewpoint). **Recommended FIRST** (cheapest, reuses infra, hits the exact symptom).
- **(a) Continuous depth-odometry** as a 2nd provider (VILENS) = general robustness in structured
  scenes where VIO's texture fails. **Recommended SECOND.**

**Critical caveat (don't over-promise):** a long STRAIGHT featureless corridor is degenerate for depth
odometry TOO — translation along the corridor axis is geometrically unobservable (Zhang/Kaess/Singh
degeneracy applies to ICP as much as to vision). So (a) does NOT magically fix the straight tube; the
along-axis is carried by IMU + the seal-on-doubt backstop. What makes depth strong is **complementary
degeneracy**: VIO fails on texture, depth fails along-corridor/open-space/glass — fuse with per-axis
covariance and where one is blind the other holds. The return-path snap is mostly (b)'s job (loop
closure on revisit, when you're back in structured space), not (a)'s.

## Shortlist (license-filtered: Apache/BSD/MIT only — Hard Rule #1)

### Continuous depth-odometry (flavor a)
1. **small_gicp (MIT)** — github.com/koide3/small_gicp. Header-only C++ (Eigen+nanoflann+Sophus),
   parallel ICP/point-to-plane/GICP/VGICP, CPU real-time. **Exposes the final Hessian → 6×6
   information/covariance + condition number** = the degeneracy-aware between-factor we need. **TOP
   PICK** — easiest to wrap behind the provider contract, and the only one that surfaces the corridor
   degeneracy numerically. (fast_gicp = same author, adds CUDA VGICP if wanted.)
2. **RTAB-Map `rgbd_odometry` (BSD-3)** — introlab/rtabmap, ROS 2 Jazzy node, F2M+F2F, emits
   covariance, already in your stack family. Run odom-only (don't let its SLAM role overlap).
3. **Open3D Park RGB-D (MIT)** — dense photometric+geometric, returns a 6×6 information matrix,
   tolerant of noisy D455 depth (0.3–6 m). CPU borderline at VGA / CUDA tensor backend.

### Frame-to-model / ICP-to-TSDF loop closure (flavor b)
4. **nvblox ICP-to-TSDF (Apache-2.0)** — **we already build this TSDF.** nvblox does NOT ship a
   tracker/reloc module, but we wire a depth→TSDF point-to-plane ICP against the map we already hold
   (directional-TSDF render / F2M-Reg recipe), deriving covariance from the ICP Hessian. Reuses GPU
   memory already paid for.

**Disqualified:** DVO (GPLv3), ORB-SLAM3 RGB-D (GPLv3), DROID-SLAM (BSD but ≥11 GB GPU — crushes a
contended GPU). **KISS-ICP (MIT)** works but its ROS "covariance" is a PLACEHOLDER (no real
condition-number) → blind to degeneracy → weakest fit for our covariance-weighted need.

## The fusion recipe (VILENS + Zhang/X-ICP), 4 rules

1. **Loose between-factor coupling** (not a tight joint re-estimator): each of depth-odom and VIO emits
   a relative-pose edge into the pose-graph. Keeps providers swappable, degrades by covariance not by
   `if(sensor_ok)`, isolates a diverging source. Accuracy lost vs tight is recovered by shaping
   covariance correctly (below). [VIN review arXiv:1906.02650]
2. **Per-axis degeneracy-aware covariance** (the key): eigendecompose the ICP registration Hessian
   (small_gicp/Open3D give it); **inflate covariance hugely along small-eigenvalue (degenerate)
   directions** so the corridor along-axis is carried by VIO+IMU, not poisoned by the ICP edge. Recipe
   = Zhang, Kaess & Singh ICRA 2016 (degeneracy factor = smallest eigenvalue / condition number;
   solution-remapping projects updates onto well-conditioned subspace); X-ICP T-RO 2023 (per-axis
   localizability from surface-normal alignment). NOTE: this adaptive covariance is the Zhang/X-ICP
   contribution — VILENS's own per-factor covariances are actually static/sensor-derived.
3. **Don't double-count the shared camera.** Depth-odom and stereo-VIO both use the camera → their
   errors are correlated. VILENS's discipline (compute each source from a distinct physical channel;
   treat as conditionally independent only then). Practical: make the VIO the **primary** pose chain;
   add the depth edge only where it INDEPENDENTLY constrains (scale/translation/geometry), or heavily
   inflate its shared-information axes. Optionally estimate an online **velocity/scale bias** per
   provider (VILENS b^v) to absorb systematic depth-odom drift. [VILENS arXiv:2107.07243, 1904.03048]
4. **Gate the depth loop-closure** (flavor b): accept a frame-to-TSDF match only on ICP
   fitness + inlier-ratio + the degeneracy factor (reject a rank-deficient match), then run **PCM**
   (Mangelson ICRA 2018) over the candidate set — slamko already has PCM consensus on visual loops, so
   reuse it for geometric ones. Precision ≫ recall (a false geometric weld is catastrophic).

## DE-RISK (offline, validated 2026-06-29 — measure before building C++)

Ran the depth-geometric-loop-closure on the casa run `/tmp/slamko_casa_g1` (raw VIO drifts **7.97 m**
start→end though the robot physically returned). Script
`scratchpad/depth_icp_derisk.py` (open3d, venv `/tmp/depthvenv`):

| Step | Result |
|---|---|
| [1] RAW ICP from the 8 m VIO prior | **fitness 0.000 — FAILS.** ICP is a REFINER, not a detector; 8 m is far outside its convergence basin. |
| [2] FPFH+RANSAC global registration (geometric place-recognition, NO prior) | **fitness 0.802, coarse correction 7.98 m** — pure-geometric recovery of the drift |
| [3] ICP refine (FPFH prior) | **fitness 0.930, inlier RMSE 0.063 m, correction 8.07 m** (vs 7.97 m drift) |

**Conclusions (now load-bearing for the build):**
1. **Depth gives a standalone geometric loop closure** — recovered the 8 m drift to within 0.1 m, 6 cm
   RMSE, 93% fitness. The corridor return-path misalignment IS fixable geometrically. ✅
2. **ICP needs a COARSE PRIOR** (raw ICP at 8 m = fitness 0). The pipeline MUST be **coarse → fine**:
   a global registration (FPFH+RANSAC) OR the existing XFeat-VPR / proximity-E provides the coarse
   alignment, then ICP refines. This is RTAB-Map's proximity+ICP and matches the §4-gate recipe.
3. **Gate** = fitness (≥~0.8) + inlier RMSE (≤~0.1 m) + PCM. A rank-deficient (degenerate) match is
   rejected by the §2 degeneracy factor.
4. HONEST scope: this start/end pair had enough viewpoint overlap for FPFH to work standalone; an
   opposite-facing revisit may still need the VPR prior (the same viewpoint ceiling as XFeat) — so
   keep VPR/proximity as the coarse-prior source, with FPFH as a VPR-independent bonus path.

## BIG DISCOVERY (2026-06-29): the depth-loop registrator is ALREADY BUILT — Phase 1 is WIRING

Phase 1 (flavor b) does NOT need a new registrator. Two components already exist + are tested:
- **`slamko_loop/include/slamko_loop/sdf_registration.hpp`** — point-to-SDF ICP (Gauss-Newton on SE3,
  Huber, drops outliers beyond the correspondence gate). Templated on a `BatchField` functor
  `f(pts_map)->SdfBatch{dist,grad,valid}`. Unit-tested (`test_sdf_registration.cpp`:
  RecoversKnownTransform err<1e-2, RejectsTooFewInliers, EmptyCloudSafe). This is the Voxgraph
  field-align channel — STRONGER than the point-cloud ICP I de-risked offline (field-gradient basin).
- **`VolumetricBackend::queryDistanceField(pts_map, dist, weight)`** — nvblox ESDF batch query
  (`slamko_tsdf/src/nvblox_backend.cpp`): `updateEsdf()` + `esdf.getVoxels()` → signed Euclidean
  distance per map point. Already implemented.

**Enabling plumbing added 2026-06-29 (builds green):**
- `SdfRegistrationResult` now returns `information` (the final GN Hessian H = Σ wJᵀJ) — the
  degeneracy-aware covariance + the gate that rejects a rank-deficient (corridor along-axis) match.
- `queryDistanceField` passthrough added on `VolumetricMapper` + `VolumetricLiveDriver` so the node
  (`vmap_`) can reach the ESDF.

**THE REMAINING WIRING (the next focused step — needs a validation run, do NOT skip it):**
Insertion point = `provider_fusion_node.cpp:2099`, the in-session XFeat loop weld
`graph_.addLoopEdge(a, q_id, r.T_query_match, loop_sigma_t_, loop_sigma_r_)`. When
`depth_loop_refine` enabled + `vmap_->backendAvailable()` + a live depth exists for `q_id`:
1. **Query cloud** (body frame): subsample the nearest depth to `q_id` (every ~8 px, z∈[0.3,6] m),
   unproject with `depth_fx_/fy_/cx_/cy_`, apply `depth_extrinsic_` → `std::vector<Vector3d>` body pts.
   (reuse the unprojection in `volumetricOnKeyframe`.)
2. **Coarse prior** `T_init` (map←query) = `graph_.pose(a) * r.T_query_match` (the loop-implied map
   pose of the query — the de-risk proved ICP needs this prior; XFeat/proximity provides it).
3. **BatchField** = lambda over `xs` (map pts): one `vmap_->queryDistanceField(xs,d,w)` for dist +
   a 6·n central-difference offset batch (±1 voxel per axis) for the gradient → `SdfBatch{dist,
   grad=∇/‖∇‖, valid=w>0}`. (The contract anticipates the finite-diff offsets.)
4. `auto res = slamko::registerToSdfBatch(cloud_body, T_init, field, cfg);`
5. **Gate**: `res.converged && res.rms < depth_loop_max_rms (~0.08 m) && res.inliers >= ~200 &&`
   smallest-eigenvalue(`res.information`) > τ (reject the corridor along-axis degenerate match).
6. **On pass**: refined a→query relative = `graph_.pose(a).inverse() * res.T_refined`; add the loop
   edge with THAT (tighter) measurement + covariance ∝ `res.information⁻¹` (per-axis, degeneracy-aware
   — inflate the small-eigenvalue axis). **On fail**: fall back to the current XFeat edge (no regress).
7. Params: `depth_loop_refine` (default OFF), `depth_loop_max_rms`, `depth_loop_min_inliers`.
8. **Validate**: re-run the casa bag, confirm the corridor return-path overlaps the outbound
   (`scripts/...` top-down + `slamko_eval` ch7); compare loops_closed + un-aligned divergence vs
   XFeat-only. This is the end-to-end gate — Phase 1 is not "done" until this run is green.

## Build order

1. **Phase 1 — depth ICP-to-TSDF loop closure (flavor b).** Register live depth to the existing nvblox
   TSDF on revisit (proximity-gated, VPR-independent — mirrors slamko's `relocalizeNear`), gated by
   fitness+inlier+degeneracy+PCM → a geometric loop constraint. Targets the corridor return-path
   directly, reuses the TSDF. Validate: the casa corridor return overlaps the outbound; `slamko_eval`
   channel 7 + a top-down trajectory plot.
2. **Phase 2 — small_gicp continuous depth-odom as a 2nd provider.** Wrap small_gicp behind
   `OdometryProvider` → 2nd `ProviderChain` → between-factors with per-axis covariance from its Hessian
   (rule 2) + no-double-count (rule 3). A/B: depth+VIO vs VIO-only on the casa bags (recall, ATE,
   un-aligned divergence). Default OFF (opt-in), like the immortal gates.
3. Keep the immortal **seal-on-doubt + HOLD** (A+B already built) as the BACKSTOP for when BOTH
   odometries degenerate at once.

## How it sits with cuVSLAM + slamko
- cuVSLAM stays the **primary VIO provider** (relative edges, untrusted-by-design).
- depth-odom (small_gicp) is the **2nd provider** — same contract, covariance-weighted, degeneracy-aware.
- nvblox TSDF is reused for **both** the dense map AND the (b) geometric loop closure.
- slamko's iSAM2 pose-graph fuses all relative edges + the depth loop constraints; XFeat VPR + the new
  geometric loop-closure are complementary recall paths.

**Sources:** VILENS arXiv:2107.07243 / 1904.03048; Zhang-Kaess-Singh ICRA 2016
(cs.cmu.edu/~kaess/pub/Zhang16icra.pdf); X-ICP arXiv:2211.16335; small_gicp github.com/koide3/small_gicp;
RTAB-Map arXiv:2403.06341; F2M-Reg arXiv:2405.00507; directional-TSDF ICP (ResearchGate 367557806);
PCM Mangelson ICRA 2018 (ieeexplore 8460217); loose-vs-tight arXiv:1906.02650.
