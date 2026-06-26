<!-- validated: 2026-06-26 · method: 5-agent stack-mapping workflow (wf_055435f3-5cc) reading the real repo · grounded in code file:line -->

> **Purpose:** the single accurate map of the CURRENT slamko stack — the TWO maps (sparse XFeat +
> dense volumetric TSDF) built from ONE provider stream, the pose-graph/reloc/Atlas spine that keeps
> both coherent, and the EXACT provider seam the OKVIS→cuVSLAM swap touches. Written because the swap
> ripples to BOTH maps, not just the sparse one. **Corrects doc-drift (§7):** live graph is Ceres
> `slamko_loop::PoseGraph` (NOT iSAM2/GTSAM as MASTER_PLAN says); live depth is D455 HW depth (NOT
> HITNet/ESS as PLAN_SLAMKO_TSDF says); `SYSTEM.md` package table is historical own-VIO — use
> `PIPELINE_STATUS_01.md` for the live pipeline. Companion to `PLAN_CUVSLAM_PROVIDER_01.md` (the
> adapter design) and the measured 7-A results (NEES + fps) in memory `slamko-cuvslam-next-pending`.

# slamko STACK MAP — full system, oriented for the OKVIS→cuVSLAM provider swap

*Author: lead architect re-orientation, 2026-06-26. Every claim grounded in the subsystem findings; live-vs-offline called out honestly.*

---

## 1. One-paragraph orientation

**slamko is a lifelong-map + multi-session-relocalization + LOOSE-fusion layer over an EXTERNAL odometry provider — it does NOT implement odometry.** From ONE provider odometry stream (OKVIS2-X pure-VIO today, run with its own loop closure OFF) it maintains **TWO co-registered maps** that both hang off the SAME corrected pose-graph node poses: (1) a **SPARSE XFeat landmark map** — per-keyframe XFeat features + stereo-triangulated landmarks sealed into `.smap` submaps, the substrate for visual relocalization; and (2) a **DENSE volumetric TSDF map** (`slamko_tsdf`, nvblox-backed) that re-integrates per-keyframe D455 hardware depth at corrected poses and exports a 2D occupancy/ESDF costmap for global A→B planning. Holding both coherent is the **lifelong pose-graph / reloc / Atlas spine**: a thin disposable Ceres SE3 `PoseGraph` (NOT iSAM2 on the live path) fed relative provider edges, corrected by XFeat/EigenPlaces relocalization + proximity + cross-session prior factors, with an Atlas break-on-loss → weld-on-feature-match state machine. The load-bearing property: **a correction (loop, cross-session anchor) simply shows up as moved node poses, and BOTH maps follow** — the sparse map reprojects landmarks, the dense map window-re-integrates the moved keyframes. The provider feeds only relative pose edges; it never sees depth (Hard Rule #4).

---

## 2. Complete data-flow diagram

```
                          ┌─────────────────────── RAW D455 STREAMS (provider-independent) ───────────────────────┐
                          │  /…/infra1 (left)   /…/infra2 (right)   /…/imu   /bno055/mag   /…/depth (16UC1 mm)      │
                          └───────┬───────────────────┬──────────────┬──────────┬───────────────────┬──────────────┘
                                  │ live              │ live         │ live     │ live              │ live
   EXTERNAL PROVIDER              │                   │              │          │                   │
   (OKVIS2-X pure-VIO,            ▼                   ▼              ▼          ▼                   ▼
    do_loop_closures=false)   [XFeat detect]    [stereo land-    [DR-gate]  [compass         [onDepth buffers
        │ live                 +VPR(EigenPlaces)  marks]          gyro        yaw prior]       depth_buf_ by ts]
        │ nav_msgs/Odometry         │                │                                            │
        │ /okvis/okvis_odometry     │                │                                            │
        ▼  (T_OB + 6x6 cov)         │                │                                            │
   ┌─────────────────┐              │                │                                            │
   │ onOdometry      │ live         │                │                                            │
   │ ProviderSample  │              │                │                                            │
   └────────┬────────┘              │                │                                            │
            │ live                  │                │                                            │
            ▼                       │                │                                            │
   ┌─────────────────┐              │                │                                            │
   │ ProviderChain   │ decimate     │                │                                            │
   │ .feed → edges   │ (kf thresh)  │                │                                            │
   └────────┬────────┘              │                │                                            │
            │ relative BETWEEN edges │                │                                            │
            │ + motion-prop 6x6 info │                │                                            │
            ▼                       │                │                                            │
   ╔═════════════════════════════╗  │                │                                            │
   ║  slamko_loop::PoseGraph     ║◄─┘ loop/anchor edges (XFeat-NN+PnP, proximity-E, x-prior)       │
   ║  (Ceres SE3, DISPOSABLE)    ║◄──── cross-session prior / proximity / yaw edges                │
   ║  optimize() → corrected     ║                                                                 │
   ║  node poses  graph_.pose(id)║                                                                 │
   ╚════════┬═══════════════╤════╝                                                                 │
            │ corrected      │ corrected poses snapshot                                            │
            │ poses          │ (every volumetric_correct_every kf)                                 │
            ▼                ▼                                                                      │
   ┌──────────────────┐   ┌────────────────────────────────────────┐                              │
   │ SPARSE map       │   │ VolumetricLiveDriver.applyCorrection    │◄─── DepthFrame (nearest      │
   │ refreshOcc:      │   │ diff poses vs last-integrated baseline  │     depth in tol 0.12s) ◄────┘
   │ reproject kept_  │   │ (>0.05m/0.02rad) → reintegrateWindow:   │     worldPose(id)=
   │ lm via T_global_ │   │ clear touched footprint + re-fuse moved │     T_global_map_·graph_.pose(id)
   │ map·pose(anchor) │   │ → enforceBudget seals oldest            │
   │ → seal .smap     │   └───────────────────┬────────────────────┘
   └────────┬─────────┘                       │ VolumetricBackend::integrate(DepthFrame, T_map_body)
            │                                  ▼  (nvblox PIMPL, SLAMKO_WITH_NVBLOX)
            │                       ┌──────────────────────────┐
            │                       │ nvblox TSDF  (BENDS)     │
            │                       │ → exportCostmap (ESDF    │
            │                       │   slice) / exportMesh    │
            │                       └───────────┬──────────────┘
            ▼                                   ▼
   OUTPUTS (live): map→odom→base TF (slewed 30Hz) · ~/fused_odometry · ~/volumetric_costmap (OccupancyGrid, transient_local)
           (dumps): graph.tum / fused.tum / provider.tum / global.tum · map/submap_*.smap · anchor_edges.csv
           (final): exportMesh .ply (live) ──┐
                                             │ OFFLINE alt path:
   OFFLINE volumetric: make_depth_skdf_frombag.py → kf_<id>.skdf → slamko_tsdf_export
                       (reintegrate ONCE at FINAL corrected poses = perfect bend, free) → map.ply + map.pgm
   VIZ: LIVE Rerun VizSink (camera FPV + sparse map, connect_grpc native viewer) │ OFFLINE rerun_show.py
        (.rrd mesh/cubes+landmarks+typed edges+anim traj), render_map_png.py, viz_tsdf_cubes.py
```

**Edge labels:** everything from raw streams → provider → chain → graph → sparse map → TF/odom is **LIVE** (validated on bag-replay @rate≤0.5). The volumetric **LIVE** path (`volumetric:=true`) is shipped/GPU-validated 2026-06-23. The volumetric **OFFLINE** path (`.skdf` → `slamko_tsdf_export`) is the older, perfect-bend route. The **live volumetric MESH viewer is OFFLINE-only**; the live viewer shows sparse map + camera FPV only.

---

## 3. Per-subsystem map

**slamko_core (contracts + infra)** — Defines the seams every other package depends on (Hard Rule #2). The two load-bearing contracts for this swap: `odometry_provider.hpp` (`ProviderSample{t, T_OB, 6x6 cov}` → `ProviderChain` decimates to one `ProviderEdge` per keyframe = relative `T_from_to` + motion-proportional 6×6 information, `edgeInformation` maps provider cov diagonal → bounded quality multiplier ≤25×, `odometry_provider.hpp:90,123`) and `volumetric_map.hpp` (`DepthFrame` = re-poseable per-kf depth source with pose **deliberately not stored**, `:58-69`; `VolumetricBackend` integrate/reset/clearRegion/exportCostmap/exportMesh, `:136-165`). Also owns `HealthSignal.odom_stale_gap_s` (`health.hpp:27`) — the loss trigger. **Status: live/unit-tested** (core 26 gtests). Note: `FactorGraphBackend`/`SensorFrontend`/`LocalSmoother` in `DECOUPLING.md` are own-VIO-era seams, NOT used by the live loose-fuser.

**slamko_fusion (loose chain-fuser)** — Contract-wise the relative-edge + global-constraint fuser; in practice the live fusion *runs inside* `provider_fusion_node`, and the global graph is **`slamko_loop::PoseGraph` solved by Ceres** (SPARSE_NORMAL_CHOLESKY, AutoDiff, Huber on loops/priors; `pose_graph.cpp:174-274,269`), **NOT** the GTSAM/iSAM2 the `slamko_fusion` README + `MASTER_PLAN` describe (GTSAM smoothers are secondary/planned, measured 15× worse and die on tracking loss). **Status: SHIPPED/live**; iSAM2-poses-only (P-C′) is aspirational/unbuilt. Key: edge information is motion-proportional (σ = k·motion + floor), so a cuVSLAM provider that reports flat/zero cov collapses every edge to the floor.

**slamko_loop (Atlas / reloc / supervisor)** — The never-lost global-consistency spine. Reloc: EigenPlaces VPR retrieval (top-N≥10) → XFeat-NN + PnP-RANSAC verify, with a separate cross-session relocalizer (`reloc_prior_`, LighterGlue, lower inlier bar) + proximity-E (`relocalizeNear` by anchor distance, VPR-independent, 3-tier candidate→soft→weld vote). Atlas: break-on-loss/quality → disjoint `component_id` → weld-on-feature-match via `connectedComponents` union-find (priors do NOT connect — source of truth for fused-vs-dangling) → dangle if no match. **Status: SHIPPED/live** — but the current never-lost logic lives **inline in `provider_fusion_node.cpp`** (seal/break/quality state machine); the old `NeverLostSupervisor`/`AnchorGate` class stack (~2.5k LOC) was **DELETED 2026-05-29**. The recall ceiling (viewpoint coverage, not descriptor quality) is the recurring real limiter, not solver/gate tuning.

**slamko_tsdf (volumetric)** — Dense nvblox-TSDF map that **bends** with the pose-graph + exports a 2D costmap for global planning. Three layers: contract (`slamko_core/volumetric_map.hpp` — the only coupling), engine `VolumetricMapper` (`volumetric_mapper.cpp`: offline `reintegrate()` = reset+re-fuse-all `:50-66`; live `integrateLive()` `:68-79` + `reintegrateWindow()` touched-window clear+re-fuse, full-fallback past 0.6 moved-fraction `:81-130`; `sealFrame()` drops depth `:132-142`), and policy `VolumetricLiveDriver` (`live_driver.cpp`: `applyCorrection` diffs corrected vs last-integrated poses → moved set → window re-integrate `:32-55`; `enforceBudget` seals oldest `:57-74`). nvblox hidden behind PIMPL (`nvblox_backend.cpp`, `#ifdef SLAMKO_WITH_NVBLOX`) so `slamko_ros` builds CUDA-free. **Status: BOTH offline (since 2026-06-22) AND live (`volumetric:=true`, GPU-validated 2026-06-23: 581 kf, loop closed, 1 fused component, 25 MB mesh) SHIPPED.** Load-bearing truth: the bend is an ARCHITECTURE property (keep depth source re-poseable), not an nvblox feature.

**slamko_ros (composition root + viz)** — The only package that wires all others; the heart is the ~2556-line `provider_fusion_node.cpp` (single node, single-threaded executor). Subscribes one provider odom topic (`:135`) + raw D455 image/right/imu/mag/depth; from the SINGLE provider pose stream drives BOTH maps via `onKeyframe` (sparse: VPR/XFeat/seal `:1371+`; then `if(volumetric_enable_) volumetricOnKeyframe` `:1450`). A live bag run = TWO processes (included OKVIS launch + this node). Outputs `~/fused_odometry`, slewed map→odom→base TF (30Hz), TUM dumps, `.smap`, `~/volumetric_costmap`, final mesh, optional live Rerun `VizSink` (connect-only, never spawn). **Status: P-A/P-B live-validated on real casa bags; live volumetric GPU-validated.** Frame convention is OKVIS-specific (`body_t_cam_xyz`/`depth_extrinsic_xyz` default to OKVIS rsD455 T_SC `{-0.03022, 0.0074, 0.01602}`, identity rotation).

**slamko_vio (provider adapters)** — Thin adapters wrapping external odometry behind the `slamko_core` provider contract (OKVIS first; klt_vo validated 2nd at Suave 0.080 m / Escaleras 0.070 m; **cuVSLAM planned**). The legacy own-VIO (XFeat/KLT/IMU) is **deprecated, slated for deletion**. **Status: live for OKVIS via plain topic wiring (no adapter node needed today — the node eats `nav_msgs/Odometry` directly); a cuVSLAM C++ adapter is the later covariance-fidelity step, not required for the first remap-based integration.**

---

## 4. The provider seam — the SINGLE thing cuVSLAM changes

**Both maps depend ONLY on the corrected pose-graph node poses**, and those poses are built from provider relative edges. Therefore swapping OKVIS→cuVSLAM is **ONE seam** — the odometry stream + frames/extrinsics — and it ripples to **both maps automatically**:

- The **sparse map** reprojects landmarks through `T_global_map_ · graph_.pose(anchor)` (`refreshOcc`, `:1958-1967`).
- The **dense map** integrates depth at `worldPose(id) = T_global_map_ · graph_.pose(id)` (`:1184-1186`), and re-integrates from corrected poses on every correction — **no optimizer hook, the provider/loop correction just "shows up" as moved poses** (`applyCorrection` diffs them). The depth SOURCE (D455 HW depth) is provider-agnostic; the provider never sees depth.

At the ROS level the swap is a one-line remap (`odom_topic:=/visual_slam/tracking/odometry`) + replacing the included OKVIS launch with a cuVSLAM node — **no new slamko node, no change to `onOdometry`/`onKeyframe`.** But for it to be *correct*, not just *connected*, exactly these must line up:

| What must align | OKVIS today | cuVSLAM requirement |
|---|---|---|
| **Odom topic** | `/okvis/okvis_odometry` (`:135`) | remap to `/visual_slam/tracking/odometry` |
| **Body frame** | pose = T_odom_body, body = IMU frame, identity body→cam rotation | cuVSLAM (Isaac ROS) publishes odom→base_link per REP-105 (x-forward), tracking frame = camera/base ≠ OKVIS IMU frame |
| **Camera extrinsics** | `body_t_cam_xyz` + `depth_extrinsic_xyz` default `{-0.03022, 0.0074, 0.01602}`, **identity rotation** (OKVIS rsD455 T_SC cam0) | MUST be re-derived (likely a **non-identity rotation**) OR cuVSLAM configured to output in the same body frame — else every VPR keyframe, landmark, AND depth integration is mis-placed, silently bending both maps |
| **Depth source** | D455 on-ASIC HW depth, decoupled, via splitter `/nvblox/depth` | UNCHANGED — provider supplies odometry only |
| **Images** | `/okvis/cam0/image_raw` (VPR) from splitter | UNCHANGED (topic name becomes a misnomer but survives) |
| **IMU** | `/camera/camera/imu` (DR-gate, stale-gap gyro-coast) | cuVSLAM must IMU-bridge short gaps OR stale-gap thresholds re-tuned |
| **Covariance** | often-zero (slamko leans on isotropic loop/prior sigmas + speed/jump heuristics) | cuVSLAM cov/quality MUST map into `ProviderSample.cov` — it's every chain-edge information + the `quality_break_cov_` gate |
| **Loop closure** | `do_loop_closures=false` (pure VIO; slamko owns global) | MUST run cuVSLAM **loop-closure OFF** — feeding a globally-corrected pose corrupts the relative-edge contract (double-loop-closure rule) |

---

## 5. Where the cuVSLAM swap stands RIGHT NOW

- **Covariance is the gate.** cuVSLAM emits **real, varying** covariance, but it is **flat/instantaneous, not drift-accumulating** → over-confident at long horizons, under-confident at 1 frame (measured EuRoC MH_03: relative-error / predicted-σ ratio ≈ **0.4× @ lag-1 → 12× @ lag-30**). slamko's `edgeInformation` is **motion-proportional** and expects cov that grows with accumulated motion, so cuVSLAM's flat cov **cannot be fed raw** — it must be re-shaped (accumulate over the keyframe interval, or synthesize motion-proportional σ) before it correctly weights chain edges and the quality-break gate.
- **Throughput is the motivation.** cuVSLAM sustains **≥120 fps vs OKVIS ~31 fps (4×)** on the RTX 4070 Laptop. OKVIS's ~31 fps is a **BY-DESIGN serial ceiling** (30 ms Ceres budget), **not GPU contention** (GPU idle ~5%) — so cuVSLAM removes the rate≤0.5 live-vs-offline trap, but **total GPU load must be re-measured** (cuVSLAM becomes a 2nd/3rd GPU job alongside XFeat reloc + nvblox TSDF).
- **Two-stage integration plan.** Stage 1 (fast first integration): **topic remap** — `provider_fusion_node` eats `/visual_slam/tracking/odometry` directly, no code change. Stage 2 (covariance fidelity): the **C++ `libcuvslam` PIMPL adapter** in `slamko_vio` that reads native covariance/quality and shapes it into the provider contract.
- **The casa-bag blocker.** cuVSLAM needs the **D455 static TF** (extrinsics, which OKVIS already encodes in rsD455_odom848): infra1 @ `(-0.03022, 0.0074, 0.01602)`, infra2 @ `(0.0648, …)`, **baseline 0.095 m**, `T_BS = I`. Until that static transform tree is published for cuVSLAM, it can't run on the replayed casa bags (which also already need `cam_info_inject_848.py` since they lack CameraInfo).
- **Honest status:** **NOT started.** Memory `slamko-cuvslam-next-pending` is PENDING/not-analyzed; the explicit pre-wire gate is (a) map cov/quality into the contract, (b) design how to VERIFY map coherence (Sim3 ATE/RPE + un-aligned divergence, Hard Rule #5) before trusting it, (c) write the `slamko_vio` adapter.

---

## 6. Recommended next concrete step (ordered) — a cuVSLAM run that exercises BOTH maps

The cleanest path to a run where the user can SEE coherence of **both** the sparse XFeat map and the volumetric TSDF, on the **same bag-replay surface** that's already validated:

1. **Publish the D455 static TF for cuVSLAM** from the known OKVIS rsD455_odom848 extrinsics (infra1/infra2 @ the listed offsets, baseline 0.095 m, T_BS=I). This unblocks cuVSLAM on the casa bags and is also what you'll diff against for frame correctness.
2. **Bring up cuVSLAM (Isaac ROS visual_slam) on a casa bag with loop-closure OFF**, fed the same splitter outputs (`/okvis/cam0`, `/okvis/cam1`) + IMU, publishing `/visual_slam/tracking/odometry`. Keep `cam_info_inject_848.py` + `d455_splitter_auto.py` (content-split, NOT NVIDIA C++) exactly as in `run_slamko_casa_flashbag.sh`. **Validate cuVSLAM alone first** (Hard Rule #5): dump its odom to TUM, Umeyama-align against OKVIS graph.tum, check ATE/RPE + un-aligned y-span — confirm it isn't diverging before slamko trusts it.
3. **Stage-1 topic remap:** launch `provider_fusion_node` with `odom_topic:=/visual_slam/tracking/odometry`, **sparse only first** (`volumetric:=false`). Before trusting the map, **re-derive `body_t_cam_xyz`** (and confirm rotation — cuVSLAM's base_link frame likely needs a non-identity SO3 vs OKVIS's identity-rotation T_SC). Verify landmarks land coherently (render_map_png.py) and a within-session loop still closes.
4. **Shape covariance:** add a minimal cov-conditioning step (accumulate cuVSLAM's flat per-frame cov over the keyframe interval into a motion-proportional `ProviderSample.cov`, or set a motion-proportional floor) so `edgeInformation` doesn't collapse every edge to the floor. Re-check the loop closes and the chain doesn't hinge.
5. **Turn on volumetric** (`volumetric:=true`), with `depth_extrinsic_xyz` re-derived to the same frame as step 3. Gate the bag on "reloc ready" (TRT build can take minutes) and run @rate≤0.5 first to isolate map coherence from any new GPU contention. Confirm the dense TSDF integrates at corrected poses and **bends** on the loop closure.
6. **Coherence A/B vs OKVIS:** same bag, OKVIS-driven vs cuVSLAM-driven, compare (a) sparse: graph.tum Umeyama-aligned ATE + loop count + landmark placement; (b) dense: mesh vert count + visual wall-doubling (reloc residual vs voxel truncation). This is the "SEE both maps coherent" deliverable.
7. **Then re-measure GPU contention** at rate 1.0 (cuVSLAM + XFeat + nvblox) — the whole point of the swap is escaping OKVIS's 31 fps ceiling, so confirm cuVSLAM actually sustains higher effective fps end-to-end without starving the other GPU jobs. Only after this is green is the Stage-2 `libcuvslam` PIMPL adapter worth building.

---

## 7. Open questions / risks the full-stack view surfaces

- **Does the volumetric reintegrate path care which provider?** Architecturally NO — it only consumes `T_map_body` from the graph. BUT volumetric fidelity is **bounded by POSE quality, not depth quality** (a 0.1 m pose error dominates ±2% depth error), so any cuVSLAM pose-quality difference shows up **directly** as wall-doubling / thinned reconstruction. The cross-session clean-fusion rule (reloc residual < voxel truncation ~0.2 m) means a noisier provider can tip coherent fusion into doubled walls.
- **Frame mismatch between cuVSLAM body frame and OKVIS body frame** is the single highest-risk silent failure: `body_t_cam_xyz` AND `depth_extrinsic_xyz` both default to OKVIS's identity-rotation T_SC. cuVSLAM's REP-105 base_link (x-forward) almost certainly needs a **non-identity rotation**. Get this wrong and landmarks + depth integrate at the wrong body pose — the map looks plausible but is systematically bent. Must be validated geometrically (step 3/5 above), not assumed.
- **Covariance contract impedance mismatch** (Section 5): flat/instantaneous cov vs slamko's motion-proportional expectation. Raw passthrough breaks edge weighting AND the quality-break gate. Needs explicit shaping; this is the real reason Stage-2 (PIMPL adapter) exists.
- **Keyframe cadence shift:** cuVSLAM's higher fps feeds `chain_.feed` faster; keyframe count depends on `chain_` thresholds, not provider rate, so cadence is bounded — BUT more raw frames change depth-association density (`volumetric_depth_tol_s=0.12`) and how often corrections fire (`volumetric_correct_every=10`). May need re-tuning.
- **Loss semantics differ:** stale-gap + quality-break + DR-gate are tuned to OKVIS's IMU-bridged coasting (a blackout PAUSES odom, doesn't inflate cov). cuVSLAM's loss/recovery behavior is different — `stale_gap_s` and pos-cov-trace thresholds need re-tuning, or the Atlas break/weld state machine may fire spuriously or miss real losses.
- **GPU contention is unproven for cuVSLAM:** it becomes the 2nd/3rd GPU consumer alongside XFeat reloc + nvblox TSDF. The rate≤0.5 ceiling is OKVIS-specific (BY DESIGN) and may not bind cuVSLAM, but total GPU load could — must be re-measured, not assumed away.
- **Seal-vs-late-correction tension is provider-independent but amplified by throughput:** `enforceBudget` seals old frames (946 MB unbounded @848 → drops depth); a late loop closure can't re-pose sealed frames, and a neighbour's `clearRegion` can wipe voxels only the sealed frame covered (−56% mesh in early regions). A faster provider = more keyframes = the store bound bites sooner.
- **No real-robot LIVE end-to-end yet** — every casa "run" is a bag replay (`ros2 bag play --rate 0.5`). cuVSLAM must be proven on the same bag-replay surface first; live-robot is a separate, later gate.
- **Doc/code drift to watch:** `slamko_fusion` README + `MASTER_PLAN` say iSAM2/GTSAM; the live graph is **Ceres `slamko_loop::PoseGraph`**. `PLAN_SLAMKO_TSDF_01.md` + `slamko_tsdf/README` say depth = HITNet/ESS/SGBM; the shipped live path uses **D455 HW depth**. `SYSTEM.md`'s package table is **historical own-VIO** — read `PIPELINE_STATUS_01.md` for the current pipeline. Don't plan the swap off the stale docs.