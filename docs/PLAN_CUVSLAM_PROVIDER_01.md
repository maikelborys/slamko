<!-- validated: pending (research+design only, no code) · 2026-06-26 · method: 13-agent workflow (4 code/API readers + 5 SOTA web + 3 adversarial verify + synthesis), grounded in cuvslam2.h read directly -->
<!-- supersedes the 'NOT analyzed' status of memory slamko-cuvslam-next-pending: the 3 gating questions are now RESOLVED below (odom-only=by-construction, covariance=real-but-uncalibrated, blackout=nullopt-not-coast) -->

> **What this is:** the decision-grade design for swapping the OKVIS front-end → cuVSLAM as a slamko
> odometry provider, with the map-correctness / pose-jump guards the user demanded. Research provenance:
> workflow `wf_e8046a4d-b37` (transcripts under `subagents/workflows/`). Three agents errored mid-run
> (prior-findings, cuvslam-covariance-web, untrusted-vio-fusion) but their partial findings fed synthesis;
> the load-bearing facts here were re-verified directly against `cuvslam2.h`. **UPDATE 2026-07-09:
> the adapter is SHIPPED and the §7 experiments were RESOLVED** (cuVSLAM went open source — several
> answered by reading source, the rest by measurement): see `RESEARCH_CUVSLAM_OPENSOURCE_01.md`.
> This doc remains the design record the implementation followed (§6 steps, §2.3 traps — both held).

# DESIGN DOSSIER — Replacing the OKVIS front-end with cuVSLAM under a map-correctness guarantee

**Project:** slamko (lifelong map + relocalization + LOOSE-fusion over an external odometry provider)
**Author role:** lead SLAM architect
**Date:** 2026-06-26
**Status:** decision-grade proposal — gated on de-risking experiments (§7) before any adapter code lands
**Scope note:** slamko does NOT implement odometry. cuVSLAM is evaluated strictly as a *pluggable provider* behind the `slamko_core` `ProviderSample`/`ProviderChain` contract. Degradation is expressed as covariance inflation, never `if(ok)`. Health reports BOTH a Sim3-aligned ATE/RPE channel (offline, GT only) AND an un-aligned divergence channel (online, GT-free) — hard-rule #5.

---

## 1. Verdict & headline recommendation

**GO — conditionally, as an ODOMETRY-ONLY provider, in a DUAL-PROVIDER configuration where OKVIS is *not* a live referee.** cuVSLAM is the right tool to break OKVIS's verified ~31 fps serial-compute ceiling (memory `slamko-okvis-realtime-ceiling`: OKVIS caps by design via serial `while(processFrame)` + 30 ms Ceres budget, GPU idle at 5%), because it is GPU-native and exposes a genuine per-frame 6×6 `PoseWithCovariance` (`cuvslam2.h:283-290`) — strictly more uncertainty signal than OKVIS, which emits *zero* covariance on `/okvis_odometry` (the single biggest blocker named across the gating research). The headline architecture is: run cuVSLAM's `Odometry` class only (`enable_localization_n_mapping=0`), ignore its built-in `Slam`/loop-closure/LMDB layer entirely, feed consecutive valid `world_from_rig` poses as relative `ProviderEdge`s into the *existing* `ProviderChain`, and let `slamko_loop` remain the sole owner of loop closure, Atlas, and relocalization (honoring the double-loop-closure rule, `odometry_provider.hpp:9-11`). This avoids the "racing estimators" failure the OKVIS refactor already taught us. **The "dual-provider-with-OKVIS-referee" option is REJECTED as a live architecture** — running a second full VIO re-imposes the exact serial-CPU/GPU cost we are escaping, and there is zero evidence in the findings that a concurrent OKVIS referee is viable under budget (the only contention datapoint is nvblox-as-3rd-consumer at rate≤0.5). The defensible referee is **IMU pre-integration (Motion Consistency Check)** computed directly from the D455 IMU — no second estimator. OKVIS stays in the picture as the **offline A/B baseline** and as a fallback provider selectable by config, not as an online cross-check. The single hard gate before this ships: **cuVSLAM's covariance must be measured for statistical consistency (NEES≈DOF) on a casa bag**, because the only field reading we have (RTAB-Map) shows a practitioner felt compelled to rescale the rotation block ×10 and bypass the covariance for loss gating — so it is real and varying, but **not verified-calibrated**.

---

## 2. cuVSLAM contract mapping

slamko's provider seam is a plain struct, not a virtual class: an adapter fills `slamko::ProviderSample{double t; SE3 T_OB; Eigen::Matrix<double,6,6> cov}` (`odometry_provider.hpp:38-44`), calls `ProviderChain::feed(s)`, and on a returned `ProviderEdge` calls `PoseGraph::addEdge(from,to,T_from_to,information,/*is_loop=*/false)` (`pose_graph.hpp ~:69`). The chain itself does keyframe decimation and edge-information math; the adapter only supplies pose + covariance + frame. The existing OKVIS template is `provider_fusion_node.cpp::onOdometry()` (ingestion at lines 861-866, `chain_.feed` at 1001, `addEdge` at 1041).

### 2.1 API-field → slamko-field map

| slamko field | cuVSLAM source (`cuvslam2.h` C++ API) | Notes |
|---|---|---|
| `ProviderSample.t` | `PoseEstimate` frame timestamp / camera-0 timestamp (Track uses cam-0 ts, frames within 1 ms, `:498`) | nanoseconds in cuVSLAM → seconds for slamko |
| `ProviderSample.T_OB` (body in drifting odom frame O) | `std::optional<PoseWithCovariance> world_from_rig` in `PoseEstimate` (`:300-303`) — the **rig** pose in cuVSLAM's world frame (world ≡ rig at first frame, `:297-298`) | Must be re-expressed into the SAME body frame OKVIS uses (resolve cuVSLAM rig→body extrinsic; the node currently hard-codes `body_t_cam_xyz` for OKVIS) and through the OpenCV(x-right,y-down,z-fwd)→ROS basis change (`cuvslam_ros_conversion.hpp:43-52`) |
| `ProviderSample.cov` (6×6, **[trans(3);rot(3)]** = nav_msgs order) | `PoseWithCovariance.covariance = PoseCovariance = Array<6*6>` float32 (`:93, :283-290`) — but cuVSLAM order is **[rx,ry,rz,x,y,z]** (rotation-first) | REORDER required (§2.3); reference converter `FromcuVSLAMCovariance` (`cuvslam_ros_conversion.hpp:76`) |
| body twist (for DR coasting + republished `/odom`) | NOT a cuVSLAM output. cuVSLAM exposes no native twist covariance | Synthesize from pose differences (the Isaac ROS node does exactly this: `pose_cache.hpp:43,58`, numerical diff over last 10 poses). Do **not** mistake this fabricated twist covariance for a cuVSLAM uncertainty |
| hard tracking-lost | `world_from_rig == nullopt` (`:302`) | The ONLY trustworthy loss signal (§3). Track doc: invalid pose returned if VO can't recover after several calls (`:490-494`) |
| synthesized quality scalar (0..1) | `f(cov trace, GetLastLandmarks().size(), Slam::Metrics lc/pnp/good counts)` | No native 0..1 scalar exists anywhere — must be synthesized |

### 2.2 Covariance-when-placeholder policy (the synthesis model)

There are three covariance regimes and each maps to a distinct slamko action:

1. **Healthy track:** `world_from_rig` holds a varying 6×6 → reorder + basis-change → feed as `ProviderSample.cov`. The chain's quality multiplier `q = clamp((cov_a+cov_b).diag.head<3>().mean()/nominal_var_t, 1.0, quality_mult_max)` (`odometry_provider.hpp:123-140`) then down-weights motion-proportional stiffness. **`nominal_var_t` (currently 2e-3, OKVIS-calibrated) MUST be re-fit to cuVSLAM's measured healthy-trace distribution** (§7-A) or `q` mis-fires.
2. **Pre-motion init transient:** cuVSLAM returns a near-identity/near-zero covariance (RTAB-Map floors `|diag|<1e-7 → 1e-4`, `OdometryCuVSLAM.cpp:457`). Treat as low-confidence: **inflate**, do not accept as a confident estimate.
3. **`nullopt` (lost):** zero uncertainty information is carried. This is a HARD trigger for the never-lost supervisor (soft-bridge / quality-break / island), **not** a graceful covariance-growth case — cuVSLAM gives no covariance-inflation signal during dropout, it just stops returning poses.

**Hard-rule #3 compliance — the one thing we will NOT copy from RTAB-Map:** when its velocity check passes but covariance is "invalid," the RTAB-Map wrapper *overwrites* the matrix with a synthetic over-confident `eye*1e-4` (`OdometryCuVSLAM.cpp:529`). slamko must **inflate on low confidence, never fabricate confidence**. Where cuVSLAM gives no usable covariance, synthesize an *inflated* `R` (large trace) scaled by the synthesized quality scalar, so a degraded edge weakens, never strengthens.

### 2.3 Two ordering traps that silently corrupt every edge weight (unit-test FIRST)

- **Block reorder:** cuVSLAM is rotation-first `[rx,ry,rz,x,y,z]` (`:287`); slamko/nav_msgs/GTSAM are translation-first `[x,y,z,rx,ry,rz]`. Without the 6×6 permutation the off-diagonal cross-covariance blocks are transposed and every `d²` / `q` is wrong. Mirror RTAB-Map's reorder + 6×6 block-diagonal basis change (`OdometryCuVSLAM.cpp:1062-1078`) and **unit-test against `FromcuVSLAMCovariance`** before trusting any number.
- **Tangent-space convention:** the covariance is float32, tangent-space at the mean via matrix-exp `mean*exp(u)` (`:285-289`), not a naive Euclidean world-frame 6×6. Feeding it to a GTSAM noise model needs the exp/log convention matched; treating it as plain world-frame covariance is subtly wrong in the rotation block specifically.

> **API-generation caveat (load-bearing):** the header slamko builds against is the **C++ cuVSLAM2 API** (`cuvslam2.h`, classes `Odometry`/`Slam`, `std::optional` validity). The RTAB-Map reference wrapper uses the **older C API** (`CUVSLAM_*` symbols, `CUVSLAM_PoseEstimate.covariance`). RTAB-Map is a correct template for *call-flow, GPU upload, frame constants, covariance reorder* — but the literal function names, the validity mechanism (status int vs `std::optional`), and the config struct differ. Port the *logic*, not the symbols.

---

## 3. The quality-management gate (the heart of the ask)

**Re-framing first (adversarial verdict, accepted):** in the chosen odometry-only architecture the provider chain only ever emits *relative* edges (`last_kf.T_OB.inverse()*s.T_OB`). **A loop-closure map-jump never enters that stream** — global corrections arrive through `slamko_loop`'s reloc as a separate constraint, gated by PCM/cycle-consistency (§3.4). So "distinguish legitimate loop-jump from bad odom-jump" is handled **by construction (separate code paths)**, not by a clever innovation signal on the per-frame stream. The per-frame gate's job is narrower and clearer: catch a *bad relative odom edge* before it enters the graph.

The gate is a four-signal state machine — **TRUST → SUSPECT → REJECT/INFLATE → RELOCALIZE** — layered exactly per the SOTA "four-layer defense" (front-end plausibility → fusion-input chi-square → back-end robust kernel → set-level cycle-consistency). No single layer is sufficient; RANSAC inlier-count and cuVSLAM `vo_state` are *provably untrustworthy* "tracking-good" signals (measured 1.41% wrong-model-with-many-inliers; RTAB-Map: status "never reports lost," `OdometryCuVSLAM.cpp:427`).

### 3.1 Signals and where each sits

| Signal | Source | Cost | Layer / placement | What it catches |
|---|---|---|---|---|
| `world_from_rig == nullopt` | cuVSLAM, free | none | front, per-frame | hard tracking loss |
| **IMU pre-integration disagreement (MCC)** | D455 IMU vs provider relative edge | cheap | **front-end plausibility (the referee)** | smooth false trajectory, scale error, the "never trust tracking-good" mode |
| velocity/jerk motion-model bound | provider edge norm / dt | free | front-end plausibility | teleport / impossible motion |
| **NIS chi-square** `d²=eᵀS⁻¹e`, `S=HPH^T+R` | provider edge residual + 6×6 `R` + pose-graph marginal `P` | cheap | fusion input | statistical outlier edge — **only after R is validated** |
| min-eigenvalue / condition number of information | backend Hessian | cheap | fusion input | degeneracy / ill-conditioning |
| DCS soft scale `s=min(1, 2Φ/(Φ+χ²))` | per-edge residual | cheap | **back-end kernel (preferred)** | absorbs a surviving wrong edge by inflation |
| GNC-TLS weight (GTSAM `GncOptimizer`) | discrete loop/x-session edges | moderate | back-end, discrete edges | wrong weld w/o manual threshold |
| PCM max-clique cycle-consistency | inter-session weld candidates | moderate | set-level, pre-optimization | spurious cross-session edges |
| synthesized quality + cov trace | cuVSLAM landmarks/Metrics + trace | cheap (export cost!) | feeds atlas-break | gradual degradation |

### 3.2 The state machine

- **TRUST** (TRUST): `world_from_rig` valid, NIS `d² < χ²(0.99,6)=16.81`, MCC within sigma bound, cov trace near calibrated nominal. → feed edge at full `q≈1`.
- **SUSPECT** (covariance inflation, hard-rule #3): NIS in `[χ²(0.95,6)=12.59, χ²(0.99,6)]` **OR** MCC disagreement above 1σ **OR** cov trace elevated **OR** landmark count below bootstrap floor (RTAB-Map uses 30, `OdometryCuVSLAM.cpp:614`). → **do NOT reject**; apply DCS scale `s=min(1,2Φ/(Φ+χ²))` to the edge information (single tunable Φ). Soft down-weight degrades gracefully where a hard EKF gate would "dead-reckon off a cliff" (the self-reinforcing P-collapse failure).
- **REJECT/INFLATE** (REJECT): NIS `> χ²(0.99,6)` AND MCC disagreement above 3σ → the edge is an outlier. Per "never-lost," do **not** discard the graph (VINS-Mono-style reset would destroy the pose-graph). Instead **BREAK/ISLAND** via the existing `atlas_break_on_quality` state machine and replace `edge.information` with a soft floor (the existing inflation path `provider_fusion_node.cpp:1028-1033`).
- **RELOCALIZE** (RELOCALIZE): `nullopt` persists past the stale-gap, OR a sustained false-trajectory break, OR kidnapping detection (§5). → seal current submap, open a disjoint island, trigger `slamko_loop` relocalization (XFeat+LighterGlue verify, proximity-E), re-anchor on match (anchor-don't-weld), dangle honestly if no overlap.

### 3.3 The false-drift blind spot (the gap none of the obvious legs cover)

A Mahalanobis/jump gate catches a *discontinuity*. It does **not** catch cuVSLAM dead-reckoning a *smooth* false trajectory with small covariance and no time-gap — the exact mode that forced `atlas_break_on_quality` onto the OKVIS path (memory `slamko-quality-break`). And the slamko loss trigger is the **odom stale-gap** (`stale_gap_s=0.5`), not a covariance spike (`health.hpp:10-14`): if cuVSLAM keeps publishing poses while internally lost, the stale-gap never fires. **Mitigation:** the MCC referee (IMU pre-integration disagreement) is the primary detector here — IMU integrates a physically-real motion that a smooth visual hallucination will diverge from; plus port `atlas_break_on_quality`'s speed/jump heuristics onto the cuVSLAM edge stream. This is the load-bearing reason the referee is IMU, not "trust cuVSLAM's status."

### 3.4 Legitimate loop-jump vs bad odom-jump — the explicit routing

- **Provider relative edges** (per-frame): vetted by §3.1-3.3. A discontinuity here is a *bad odom-jump* → SUSPECT/REJECT.
- **Global corrections** (loop closures, cross-session welds): never enter the provider chain. They arrive as discrete `is_loop=true` edges from `slamko_loop` reloc and are vetted by **GNC-TLS** (`barc²=χ²(0.99,dof)`) on the single edge + **PCM max-clique** on the *set* of inter-session candidates (compose each weld with the trusted provider chain, χ² against identity, accept the maximum clique). This is exactly the existing I2 never-false-merge discipline and the cross-session A/B/E path. **PCM's hidden assumption** — that the provider chain baseline is outlier-free — is *why* §3.1-3.3 must run on provider edges FIRST: PCM trusts a chain that the per-frame gate has already vetted.

---

## 4. Map-correctness verification

**Critical correction (adversarial verdict, accepted):** "alignment-independent" ≠ "ground-truth-free." Two metrics commonly listed as online actually need GT and must be moved to the offline column. The dossier therefore splits verification into a **GT-required offline column** (benchmark bags only) and a **GT-free online column** (the only thing available on casa/real-robot, which have no GT).

### 4.1 GT-FREE / online (the live coherence channels — hard-rule #5's second half)

These run live on casa/real-robot with no ground truth:

- **(a) Provider/DR-vs-fused trajectory divergence** — the *workhorse*. Compare `provider.tum` (drifting odom dead-reckoning) vs `fused.tum` (optimized) — two *estimates*, no GT. With no global constraints the P-A acceptance test is ~0 divergence; a growing divergence is the honest "are we diverging" signal. This is the only entry in the un-aligned channel that survives the GT test.
- **(b) NIS / per-edge Mahalanobis residuals + PCM max-clique** — GT-free **but require a valid, calibrated edge covariance.** Non-functional on the OKVIS path (zero covariance) and unit-mismatched on cuVSLAM until §2.3 reorder + §7-A NEES calibration land. Until then, fall back to residual-magnitude DCS soft gating (no calibrated S required).
- **(c) Information-matrix min-eigenvalue / condition number** — GT-free and **functional now**; degeneracy monitor (Zhang & Singh degeneracy factor).
- **(d) IMU shock/jerk spikes + pose-update discontinuity (kidnapping)** — GT-free, functional now (§5).
- **(e) Volumetric voxel-collision / re-integration consistency after deformation** — GT-free, costly. slamko's volumetric map re-integrates the touched window on pose correction (memory `slamko-live-volumetric-tsdf`: `reintegrateWindow` clear+refuse, not whole-map). Post-shock coherence check = the mesh bends with the graph and no double-surface/voxel-collision appears.

> **Use NIS, never NEES, in the online column.** Literal NEES `eᵀP⁻¹e` needs the true state = GT. The online innovation-based form is NIS `yᵀS⁻¹y`. The findings mislabeled this; the design doc must say NIS.

### 4.2 GT-REQUIRED / offline (the A/B qualification of cuVSLAM — hard-rule #5's first half)

Runs only in `scripts/bench_ate.sh` on **EuRoC / TUM-VI** (GT bags). casa/real-robot have no GT, so this column is unavailable there:

- **Sim3-aligned ATE** (needs GT + alignment) and **RPE-vs-GT over fixed sub-segments** (alignment-free but still needs the GT relative motion to difference against — it is NOT a live metric).
- **NEES per block** — the decisive calibration check: per-frame `eᵀΣ⁻¹e` against GT-aligned reference, separately for the translation-3 and rotation-3 blocks. Calibrated ⇒ mean ≈ 3/block; >3 over-confident, <3 under-confident (the RTAB-Map ×10 rotation fudge predicts under-confidence in rotation).

**The A/B qualification protocol vs OKVIS** (run before trusting cuVSLAM as the default provider):

1. Same EuRoC/TUM-VI + casa bags through both providers, OKVIS as the frozen baseline.
2. Report on GT bags: ATE, RPE, per-block NEES. On all bags: un-aligned provider-vs-fused divergence, map density (landmark/mesh-vertex count), and **real-time fps** (the whole motive — must clear OKVIS's ~31 fps ceiling at native frame rate, not at rate≤0.5).
3. Acceptance = ATE/RPE ≤ OKVIS within 5% (regress ≥5% → reject, per benchmark-driven rule), un-aligned divergence ≈ 0 with loops off, map density not degraded, **and** sustained ≥45 fps at native rate. Do **not** accept cuVSLAM's internal `Slam::Metrics` loop-closure counts or its own covariance as proof of coherence.

---

## 5. Lifelong robustness

How the gate ties into slamko's *existing* primitives, so this is integration, not new invention:

- **Collision / shock detection:** IMU acceleration/jerk spike with a **~30 ms minimum-duration gate** to reject transients (the concrete, reusable window from the verification research). On a confirmed shock → *inflate covariance* on the affected edges (hard-rule #3), do not branch on a boolean. A shock is a prior that the *next* provider edges are suspect → bias the gate toward SUSPECT.
- **Kidnapped-robot detection:** three GT-free signals — (i) a pose-update **discontinuity** (correction jump) beyond a motion bound, (ii) a collapse in localization confidence (synthesized quality scalar / landmark count floor), (iii) a **Mahalanobis spike** of the next constraint vs prediction. Any → RELOCALIZE state.
- **Recovery / relocalization triggers:** `nullopt` past stale-gap, sustained false-trajectory break, or kidnapping → seal submap, open a **disjoint Atlas island** (per-component gauge), trigger `slamko_loop` relocalization (EigenPlaces/XFeat retrieval → XFeat+LighterGlue verify → proximity-E by anchor-distance for the VPR-recall-dead gap). **anchor-don't-weld:** re-anchor via a reversible gated weld on a feature match (not spatial overlap); if no overlap, **dangle honestly** (never a fake-coherent double) — the validated A/B/E model.
- **never-lost vs cuVSLAM's expensive reset:** cuVSLAM's recovery is *not* automatic — on lost it destroys the tracker (`CUVSLAM_DestroyTracker` + GPU re-warm) and waits for an external reset. slamko's supervisor must own the reset policy and **keep the GPU context warm** (warmup in ctor) so re-init latency is bounded. A hard reset must NOT propagate to the slamko pose-graph — it BREAKS/ISLANDS, it does not discard.
- **Quality-break & soft-bridge interplay:** `atlas_break_on_quality` is the false-trajectory catcher (ported to cuVSLAM edges via MCC + speed/jump). soft-bridge keeps good chunks connected via inflated-covariance edges on a TRUE odom stale-gap (the stiff-chain B fix: soft only on a real gap, never on every image-miss). The choice between pure-break (isolates bad, fragments good) and soft-bridge (keeps good connected, down-weights bad) is unchanged from the OKVIS path — soft-bridge recommended.
- **Volumetric re-integrate-on-correction:** when a loop/weld corrects poses, the live nvblox-in-loop driver diffs poses → moved-window set → `reintegrateWindow` on only the touched blocks (decoupled from the optimizer). After any shock/kidnapping recovery, the volumetric map bends with the graph and §4.1(e) confirms no distortion.

---

## 6. Adapter implementation plan

Ordered, concrete. **Do not fork the 2556-line `provider_fusion_node.cpp`** — extract the ingestion slice into a reusable class so OKVIS and cuVSLAM share chain+graph plumbing and differ only in how they fill `ProviderSample`.

**Step 0 — extract the shared seam (refactor, no behavior change).**
Pull `onOdometry`'s `ProviderSample → chain_.feed → graph_.addEdge` slice (`provider_fusion_node.cpp:861-1049`) into a small `ProviderIngest` helper in `slamko_ros` (or `slamko_vio`). OKVIS keeps working; cuVSLAM reuses it. Validate against the offline driver `slamko_ros/tools/provider_chain_offline.cpp`.

**Step 1 — build/link plumbing.**
- New package contents in `slamko_vio/` (it is "thin provider adapters" by charter).
- Link against `/opt/ros/jazzy/lib/libcuvslam.so`; include `/opt/ros/jazzy/share/isaac_ros_nitros/cuvslam/include/cuvslam/cuvslam2.h`. CMake: find the lib, add include dir, `target_link_libraries(... cuvslam)`. CUDA toolkit required (GPU upload path).
- Hide cuVSLAM behind a PIMPL so `slamko_ros` stays CUDA-free (the nvblox pattern, memory `slamko-live-volumetric-tsdf`). NO-OP unless a build flag (e.g. `-DSLAMKO_WITH_CUVSLAM`) is set.

**Step 2 — files to create.**
- `slamko_vio/include/slamko_vio/cuvslam_provider.hpp` — adapter class interface (PIMPL).
- `slamko_vio/src/cuvslam_provider.cpp` — the impl: ctor warms GPU; lazy `Odometry(rig, config)` construction on first frame; per-frame `Track(images, {}, depths)` → `PoseEstimate`; covariance reorder+basis (port `OdometryCuVSLAM.cpp:1062-1078`, unit-test vs `FromcuVSLAMCovariance`); emit `ProviderSample`.
- `slamko_vio/src/cuvslam_calib.cpp` — calibration plumbing: per-camera `[cx,cy,fx,fy]` `num_parameters=4` `distortion_model='pinhole'`; stereo extrinsics via the frame constants (`cuvslam_pose_canonical * localTransform * baseline * optical_pose_cuvslam`); D455 baseline sign validated against actual extrinsics. IMU calibration struct (`ImuCalibration`: `rig_from_imu`, noise densities, random walks, frequency) if Inertial mode.
- `slamko_ros/nodes/cuvslam_provider_node.cpp` (or a provider-select param on the fusion node) — subscribes D455 stereo (+IMU), drives the adapter, calls `ProviderIngest`.
- `slamko_vio/test/test_cuvslam_cov_reorder.cpp` — **the first deliverable's unit test** (§2.3).
- `slamko_vio/launch/cuvslam_provider.launch.py` + a config preset (analog of `rsD455_odom848`).

**Step 3 — mode + IMU decision (mutually exclusive, document it).**
- `OdometryMode::Inertial` (single stereo + IMU) gives VIO — feed IMU via `RegisterImuMeasurement(0, {ts, accel m/s², gyro rad/s})`, sensor_index MUST be 0, interleave Track→ImuN→Track (`:522-533`). Gravity available via `State.gravity`.
- `OdometryMode::Multicamera` / `RGBD` is vision-only (the RTAB-Map template proves only the stereo-VO path; **IMU is unimplemented there** — the VIO-beats-OKVIS claim is unvalidated by that wrapper and must be verified directly in §7).
- **Recommendation:** start Inertial (matches the OKVIS VIO comparison fairly); document the choice — it changes covariance characteristics.

**Step 4 — config (odometry-only).**
`enable_localization_n_mapping=0` (no internal SLAM); `use_gpu=1`; export flags (`enable_observations_export`/`enable_landmarks_export`) **OFF by default** (they "slow down execution," `:417-426`; turn on only if §7-B shows the fps headroom). Drive cuVSLAM's loop closure OFF — consume drifting odom only (double-loop-closure rule). Keep `map_cache_path`/`SaveMap`/`LocalizeInMap` OUT of slamko's lifelong path; `slamko_loop` owns Atlas/reloc.

**Step 5 — covariance + quality mapping (gate (a) from memory).**
Make cov mapping the first functional deliverable: reorder → basis-change → `ProviderSample.cov`; synthesize quality from cov-trace + landmark count + Metrics; `nullopt` → hard-lost. Re-tune `nominal_var_t`, `quality_mult_max`, `cov_soft_thresh`, `stale_gap_s` from measured data (§7-A) — OKVIS's magic numbers will mis-fire.

**Step 6 — dual-provider wiring.**
Provider selection by config param (`provider:=okvis|cuvslam`). Both fill the same `ProviderSample` and feed the same `ProviderChain`/`PoseGraph`. NO concurrent referee — OKVIS used offline as the A/B baseline. (Optional future: an offline-only OKVIS cross-check on recorded bags for regression, never live.)

**Step 7 — validate end-to-end.**
Run `provider_chain_offline` + `scripts/bench_ate.sh` per §4.2; confirm `fused.tum` vs `provider.tum` ≈ 0 (loops off), then loops on, then casa coherence + volumetric. Update `slamko_vio/docs/STATUS.md` with numbers on green; commit code + docs together.

---

## 7. Risks, unknowns, and the de-risking experiments to run FIRST (measure before build)

Per slamko method: measure offline before writing the adapter. Ordered by blocking-ness.

- **A — cuVSLAM covariance is real but NOT verified-calibrated (SHAKY per adversarial verdict).** The only field reading rescales the rotation block ×10 "to make it more realistic" (`OdometryCuVSLAM.cpp:1031,1089`) and bypasses covariance for loss gating. *Experiment:* run cuVSLAM-as-provider on a casa bag, compute per-block NEES against a GT-aligned/OKVIS reference; log the steady-tracking trace distribution. Settles `nominal_var_t` + the chi-square thresholds. **Until this lands, every NIS/DCS/GNC threshold is a formula without a valid input — prefer residual-magnitude DCS soft gating.**
- **B — export-flag fps cost vs the 31 fps motive.** Per-frame landmark/observation counts are gated behind export flags the doc warns "slow down execution." *Experiment:* benchmark fps with export ON vs OFF at native frame rate. If costly, derive quality from covariance alone.
- **C — IMU/VIO path unproven.** The RTAB-Map template is stereo-VO only; IMU unimplemented there. *Experiment:* run cuVSLAM `OdometryMode::Inertial` on EuRoC, confirm it ingests the IMU and beats OKVIS on ATE+fps. The "beats OKVIS" claim is unvalidated until this runs.
- **D — false-trajectory blind spot.** No NIS/jump gate catches smooth dead-reckoned hallucination with small covariance and no time-gap. *Mitigation/experiment:* validate the MCC referee + ported `atlas_break_on_quality` on the casa brutal bags (IMU-blackout, wall-pointing) — confirm it flags the user's known fast-maneuver LOST events.
- **E — covariance reorder/basis silent bug.** An unverified reorder transposes cross-covariance and makes every `d²` wrong. *Mitigation:* unit-test vs `FromcuVSLAMCovariance` is Step 2's first test, not an afterthought.
- **F — body-frame extrinsic offset.** Wrong rig→body resolution shows as a constant per-edge rotation/translation bias. *Mitigation:* validate via un-aligned divergence (hard-rule #5), not just Sim3-aligned ATE (which hides a constant offset).
- **G — GPU contention with nvblox + reloc.** cuVSLAM as a GPU consumer alongside the live volumetric TSDF and XFeat/TRT reloc. The one datapoint (nvblox 3rd consumer at rate≤0.5 did NOT starve OKVIS) is encouraging but not the same load. *Experiment:* measure the 3-consumer load at native fps.
- **H — expensive lost-recovery latency.** cuVSLAM destroys+rebuilds the tracker on lost. *Mitigation:* keep GPU context warm; budget re-init latency in the supervisor.

---

## 8. Open decisions for the user

1. **Architecture: odometry-only provider, OR also use cuVSLAM's built-in Slam/loop-closure?**
   **Recommendation: odometry-only** (ignore class `Slam`). Running cuVSLAM's PGO+LMDB alongside `slamko_fusion`/`slamko_loop` re-creates the racing-estimators failure; cuVSLAM's `max_map_size` default of 300 poses also conflicts with lifelong/out-of-core ambitions. slamko owns all global constraints.

2. **Referee: live OKVIS cross-check, OR IMU pre-integration (MCC) only?**
   **Recommendation: MCC only.** A live OKVIS referee re-imposes the exact 31 fps ceiling we are escaping and is unbudgeted. OKVIS stays as the offline A/B baseline.

3. **Mode: Inertial (stereo+IMU VIO), OR Multicamera/RGBD (vision-only)?**
   **Recommendation: Inertial**, to make the OKVIS comparison fair and recover gravity — but gated on experiment 7-C confirming the IMU path works (the only reference wrapper has IMU unimplemented). Fall back to Multicamera if 7-C fails.

4. **Covariance trust: gate on cuVSLAM's 6×6 now, OR DCS-soft until NEES-calibrated?**
   **Recommendation: DCS-soft (residual-magnitude) until 7-A proves NEES≈DOF**, then enable the NIS chi-square gate. Never adopt RTAB-Map's synthetic `eye*1e-4` confidence fabrication (hard-rule #3).

5. **Quality export flags: ON (richer signals) or OFF (fps headroom)?**
   **Recommendation: OFF by default**, decided by experiment 7-B. Derive quality from covariance trace if export is too costly.

6. **Rollout: replace OKVIS as default, or ship cuVSLAM as a selectable second provider first?**
   **Recommendation: selectable second provider** (`provider:=okvis|cuvslam`) behind a build flag, OKVIS remains default until the §4.2 A/B passes (ATE/RPE within 5%, ≥45 fps native, un-aligned divergence ≈ 0, map density preserved). Flip the default only on green.

---

### Key file/line anchors (for the implementer)

- cuVSLAM C++ API: `/opt/ros/jazzy/share/isaac_ros_nitros/cuvslam/include/cuvslam/cuvslam2.h` — `PoseCovariance`:93, `PoseWithCovariance`:283-290, `world_from_rig`/`PoseEstimate`:300-303, `Odometry`:331/473, `Track`:515, `RegisterImuMeasurement`:535, `Slam`:633, `Metrics`:690-698, `OdometryMode`:357-364, frame conv:83. Lib: `/opt/ros/jazzy/lib/libcuvslam.so`. ROS status: `.../isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus.msg` (`vo_state` 0/1/2).
- Reference wrapper (C API, port logic not symbols): `/home/maikel/coding/RTABmap/src/rtabmap/corelib/src/odometry/OdometryCuVSLAM.cpp` — covariance reorder/basis 1062-1078, ×10 rotation fudge 1031/1089, synthetic-confidence fallback (DO NOT COPY) 529, "never reports lost" 427, init floor 457, bootstrap landmark gate 614, config 778-812.
- slamko contract: `slamko_core/include/slamko_core/odometry_provider.hpp` — `ProviderSample`:38-44, `feed`:90-115, `edgeInformation`:123-140, double-loop rule:9-11; `health.hpp`:10-14,24-45; `slamko_ros/nodes/provider_fusion_node.cpp` — ingestion 861-866, soft-gate 998, inflation 1028-1033, feed/addEdge 1001/1041; `slamko_loop/include/slamko_loop/pose_graph.hpp`:~69 (`addEdge`).