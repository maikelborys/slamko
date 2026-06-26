<!-- validated: 2026-06-26 · 9-agent reflection workflow (wf_43c16d07), grounded in code file:line + this session findings + SOTA -->
> The strategic reflection: the GOAL (never-crash + never-lie + super-recoverable over an UNTRUSTED provider — NOT never-distorted, which is information-theoretically impossible without recognized revisits), the honest scorecard, and the prioritized improvement plan. #1 = P0 the untrusted-provider COHERENCE GATE (slamko asserts untrusted but does not gate it). Dense SDF-ICP (built) = P1, a narrow NEES-gated anisotropic refiner, NOT a planar-degeneracy solver.

# slamko — Reflection + Improvement Plan toward IMMORTAL-SLAM

*Lead-architect reflection. Decision-grade. Grounds every claim in the current code (file:line), the cross-session memory, the external research, and — honestly — the adversarial verdicts that say two of the four "obvious" upgrades are partly refuted and the true #1 is none of them.*

---

## 1. The GOAL, stated precisely + measurably

slamko is **not an odometry system**. It is the **lifelong-map + multi-session-relocalization + loose-fusion layer over an untrusted, pluggable external provider** (OKVIS2-X pure-VIO today; cuVSLAM/klt_vo behind the same `ProviderSample` seam later). Founding proof the pains live in *this* layer, not in odometry: OKVIS magistrale **5.91 cm** ATE vs slamko-own-VIO **10.06 m** (`MASTER_PLAN.md:13-23`). The user's acceptance criterion is verbatim *"que sea bien estable sin romperse"* — **result + stability over novelty**, accuracy explicitly secondary.

The goal decomposes into **three NEVER-invariants** (`PLAN_ROBUSTNESS_01.md:18-29`), each with a *machine-checkable* measure (no "looks fine in rviz"):

| Invariant | Contract | How it is measured (HR#5: BOTH aligned + un-aligned) |
|---|---|---|
| **I1 — never ingest garbage** | input contract: reject unobservable motion / degraded submaps before they enter the graph | seal-quality gate pass-rate; DR-informed bar on degraded submaps as reloc targets (`provider_fusion_node.cpp:402-406,909-924`); un-aligned divergence on the ingested segment |
| **I2 — never make an irreversible false connection** | merge contract: every weld covariance-gated, inertially sanity-checked, **reversible** | `audit_i2.py` over real welds (162 welds, 0 teleports today); **and** an *adversarial aliasing-injection* false-positive rate — **not yet run** |
| **I3 — never permanently lost** | recovery contract: always tracking / recovering / honestly-disconnected-but-recoverable | quality-break fire+recover rate (5/5 on casa brutal); blackout recovery (3.12 → 0.16 m); connected-components count (fused vs honestly-dangling) |

**The measurability rule that governs all of it (HR#5, `CLAUDE.md`):** report BOTH a Sim3-aligned ATE/RPE *and* an un-aligned divergence/health metric — because **Sim3 alignment hides catastrophic divergence**. Any verdict must be an exit code, never an eyeball.

**The result we actually seek, in one line:** a map that *never crashes, never lies about what it knows, and recovers when the world is re-recognized* — over a provider we are not allowed to trust.

---

## 2. The architecture as it IS

### 2.1 Two maps from one provider, on one spine

From **one** odometry stream (`ProviderSample{t, T_OB, 6×6 cov}` → `ProviderChain` decimates to one `ProviderEdge`/keyframe = relative `T_from_to` + **motion-proportional** 6×6 information, `odometry_provider.hpp:90,123`) slamko builds **two co-registered maps that both hang off the SAME corrected pose-graph node poses** (`STACK_MAP_01.md:20`):

- **SPARSE** XFeat landmark `.smap` submaps — the substrate for relocalization.
- **DENSE** nvblox volumetric TSDF — bends via re-integrate at corrected poses, exports a 2D ESDF costmap.

**The load-bearing property:** a correction is just **moved node poses**, and both maps follow — sparse reprojects landmarks through `T_global_map · graph_.pose(anchor)` (`refreshOcc :1958-1967`), dense re-integrates depth at `worldPose(id) = T_global_map · graph_.pose(id)` (`:1184-1186`). No optimizer hook; the bend is an **architecture property** (keep the depth source re-poseable), not an nvblox feature — externally vindicated: NVIDIA states nvblox is **local-only and does NOT deform** under loop closure, so a globally-coherent nvblox map is *impossible* without exactly this external re-integration layer.

### 2.2 The pose-graph spine (and a doc-drift correction)

The live global graph is a **thin, disposable Ceres SE(3) `BetweenFactor` PoseGraph** (`slamko_loop/src/pose_graph.cpp:27-58,174-274`; `SPARSE_NORMAL_CHOLESKY`, AutoDiff, Huber on loops/priors). It is **NOT the iSAM2/GTSAM** that `MASTER_PLAN §0.5` and the `slamko_fusion` README still claim — GTSAM smoothers measured **15× worse and die on tracking loss** (`STACK_MAP_01.md:93`). **This doc drift is itself a live hazard** (next session could plan the iSAM2 work off stale altitude). `addEdge` accepts a **full 6×6 information matrix** (`pose_graph.hpp:69-70`) — anisotropic weighting is *supported* but used today only by the soft-bridge.

### 2.3 The never-lost machinery (inline, single-threaded)

The old ~2.5k-LOC `NeverLostSupervisor`/`AnchorGate` class stack was **deleted** (2026-05-29); the state machine now lives **inline in `provider_fusion_node.cpp`** (~2556 lines):

- **Edge logic per keyframe** (`:1019-1050`): stiff provider-cov odom edge (normal); high-cov DR-gate **soft** edge on a *true* odom stale-gap (`:1036-1047`); **break** → `++component_id_`, a disjoint island (`:1025-1035`).
- **Quality-break LOST→RECOVERED** (`:941-995`): incoherent transition (speed > 6 m/s OR jump > 1.5 m/s OR cov spike) → seal good map; BREAK isolates the bad stretch, SOFT-BRIDGE keeps it connected with high cov; RECOVERED after sustained-coherent samples. **Detects FALSE trajectory even with NO odom gap** (OKVIS coasts on IMU) — user-confirmed 3/3 on casa brutal.
- **Atlas weld**: `connectedComponents()` union-find (`pose_graph.cpp:283-299`) is the **source of truth for fused-vs-dangling**; a weld that joins two islands merges gauges → one bends onto the other for free.

### 2.4 The channels (how a correction is *proposed*)

Loop closure is a **disjunctive-retrieval → single shared PnP-verify → single graph-edge** pipeline (`xfeat_relocalizer.cpp`). Every channel only differs in how it **retrieves** candidate submap-ids; all converge on **one** `verifyAgainst` → `pnpRansac` (P3P, min_inliers=15, `:85-151`):

| Channel | Retrieval | Status |
|---|---|---|
| **Appearance VPR** (EigenPlaces) | per-KF cosine top-N (`:368-390`) | solid, **viewpoint-limited** |
| **BoW** | fallback only if VPR empty (`:393-394`) | works |
| **Proximity-E** | anchor within radius, VPR-independent (`relocalizeNear :336-347`) | shipped; **3-tier candidate→promote** gate (`:1626-1678`) |
| **ScanContext (NEW, sparse geometric)** | yaw-invariant per-KF descriptor (`geometricCandidates :244-266`) | **retrieval-only, 0 demonstrated recall gain** on casa; ~90° FOV gives thin overlap |
| **Dense point-to-SDF ICP (NEW)** | `registerToSdfBatch` (`sdf_registration.hpp`) | **validated in selftest, NOT wired into the live node** — grep finds zero usage in `slamko_ros`/`tryRelocalize` |

**Critical structural fact:** channels fuse at the **retrieval** level (OR-union of submap-ids), **never at the constraint level**. ScanContext retrieves a different-heading candidate but it is still **PnP-verified by appearance descriptors** → a true different-heading revisit it finds still fails verify (no image overlap). Reloc welds use **isotropic** information (`addLoopEdge sigma_t/sigma_r`, `:1747/1859`) even though the anisotropic path exists.

### 2.5 Honest scorecard vs the goal

| Goal facet | State | Evidence / caveat |
|---|---|---|
| **never-crash** | ✅ solid | S1/S2 stress never crash; S3 volumetric **segfault = build-config regression** (`--allow-shlib-undefined` nvblox workaround), not logic |
| **never-lost (sparse)** | ✅ solid | quality-break 5/5, 1 coherent component via soft-bridge, blackout 3.12→0.16 m |
| **never-false-merge (I2)** | 🟢 bounded | 162 real welds, 0 teleports — **but the gating stack in front of the backend earns this**, not the kernel; **adversarial aliasing test never run** |
| **map-bounded (sparse, by area)** | 🟢 bounded | ORB-style data-assoc cull: +100%→+3%/visit; dedup at seal |
| **map-bounded (dense)** | 🔴 missing | per-keyframe depth store **751–946 MB @848, unbounded**; `enforceBudget` seals oldest but a late loop can't re-pose sealed frames |
| **recall** | 🟡 understood, not solved | reframed as **viewpoint coverage**, not descriptor quality — opposite-facing = no overlap = unmatchable by any model; geometric channels built, **0 gain demonstrated** |
| **never-distorted globally** | ⛔ impossible by theory | see §3 |
| **untrusted-provider verification** | 🔴 **the hole** | cuVSLAM's 90 m flashbag jump was caught **by eyeball, not a gate** |
| **graded GOOD/DEGRADED/LOST hysteresis** | 🟡 partial | LOST→RECOVERED shipped; DEGRADED regime (wall-pointing → pause growth) absent |
| **IMU-saturation gate (R0.4)** | 🔴 hole | OKVIS plausibility/jerk gates exist but **DISABLED** (`a_jerk_max=0, a_plausible_max=0`) |
| **GPS/compass-yaw factor** | 🟡 instrumented only | raw-mag present, unary yaw factor + GNSS anchor not in graph |
| **out-of-core map** | 🔴 missing | whole map + reloc DB in RAM; no WM/LTM disk tier |
| **live health monitor** | 🔴 missing | no `/slamko/health` topic + panel |
| **real-robot end-to-end** | 🔴 never run | every "run" is `bag play --rate 0.5` |

---

## 3. The honest theoretical floor

The single most important thing to be honest about. "Distorted" must be decomposed or the promise is a lie:

1. **Global metric fidelity (ATE).** With stereo+IMU you observe scale, roll, pitch (gravity-anchored), and local motion. **Global position (3 DOF) and global yaw (1 DOF) are unobservable** without an external absolute (GPS, compass, recognized revisit, or a trusted prior map). Drift in those 4 DOF is **unbounded and typically super-linear**. **No algorithm over a non-revisiting untrusted provider can bound this — provably impossible.** Loop closure / GPS / compass do not remove the floor; they convert *unbounded* drift into *bounded-between-anchors* error, and the bound is only as good as **anchor density × anchor correctness**.

2. **Local metric consistency** (within a submap / short window). **Achievable**, bounded by the provider's *local* accuracy.

3. **Self-consistency / non-contradiction** (no fold, no double wall, no fake-coherent merge). **Achievable but conditional** on two disciplines that are not free: **register-before-integrate** (dense) and **never-false-merge** (sparse).

**Therefore slamko should HONESTLY promise:**
- ✅ never-crash;
- ✅ never-permanently-lost (always tracking / recovering / honestly-disconnected);
- ✅ locally-consistent submaps;
- ✅ **honest dangling** — overlap → connect+align, no-overlap → hang, **never a confident double**.
- ⛔ **NOT** never-distorted globally. In the no-recall limit the honest global statement degrades to *"a set of locally-consistent islands whose relative placement is honestly unknown."*

**Tagline:** *"slamko never lies about what it knows"* — **not** *"slamko is never wrong about global geometry."*

**The uncomfortable adversarial correction (accepted):** the "never-lie/dangle" promise we want to bank is itself **not yet fully delivered** — it has three leaks that turn a dangle into a lie:
- **(a) Perceptual aliasing → false merge.** Robust backend is **Huber-only** (down-weights, never rejects); consensus is **within-channel** only (PCM streak, proximity 2-vote), with **no cross-channel agreement gate**; and the **adversarial aliasing-injection test (S5) was never run.** "162 welds, 0 teleports" validates the *easy* case (real welds), not a self-similar corridor/garage where a confident wrong weld folds the map.
- **(b) Smooth sub-threshold provider bias evades every gate.** Quality-break fires on *kinematic incoherence*; a **consistent scale error or slow yaw ramp** is smoothly-distorted-but-plausible, flagged by nothing, corrected by no revisit if recall fails. "Untrusted provider" is currently **asserted, not enforced** — a flat-cov provider collapses every edge to the floor (`STACK_MAP_01.md:93,122`).
- **(c) Dense integrate-before-register.** TSDF self-consistency is conditional on register-before-integrate, which is a guard to *add*, not one in place — integrating depth at a drifted-but-unconfirmed pose welds a double wall no later pose nudge can remove.

So: the floor argument is **SOLID**; the fallback promise is the right target but is **partially aspirational today.**

---

## 4. The prioritized improvement plan (leverage-vs-effort)

The four "menu" items asked about (provider calib / GNC / dense+appearance fusion / wider FOV) are addressed honestly below — but **the adversarial verdict is correct: the true #1 is none of them.** Ranked:

### P0 — Untrusted-provider COHERENCE GATE + the two cheap gate-holes it lives beside *(near-zero hardware cost; the binding gap slamko's own premise demands)*
The architecture treats the provider as untrusted but has **no automated check** that decides whether to trust an edge *before* fusing. cuVSLAM exposed this: a 90 m jump caught by eyeball. By slamko's own **"gates before anchor-edges"** doctrine (`PLAN_ROBUSTNESS §5` — propagating corrections through an ungated graph spreads corruption faster), this **precedes** everything else.
- **(P0.1) Provider-coherence/NEES gate.** An automated HR#5 check (Sim3 ATE/RPE + **un-aligned divergence / jump detector**) on the provider stream that **down-weights or breaks** a divergent segment instead of silently dangling it. Tooling **already exists**: `scripts/cuvslam_nees.py`, `scripts/cuvslam_traj_check.py`. Wire it as a live edge-acceptance gate.
- **(P0.2) Enable the disabled OKVIS IMU-saturation/jerk gates (R0.4).** `a_jerk_max=0, a_plausible_max=0, g_plausible_max=0` in `rsD455_map_odom` — turn them on; the saturated-IMU/drone-flip/kidnap interval should be declared **unobservable → clean break** (I1).
- **(P0.3) IMU-shock/jerk DETECT channel** in `slamko_loop` — raw-accel-jerk + gyro-spike thresholds, the *canonical kidnap trigger* the literature names that slamko **lacks**. Catches a clean lift/knock that produces **no** speed-jump (the provider coasts on IMU). Complements quality-break.
- **Test:** post-fix, the cuVSLAM flashbag must **break/down-weight** the 90 m segment automatically (exit code), not dangle.

### P1 — Wire the dense SDF-ICP as an anisotropic refine-and-weight factor *(best ROI of the four offered; pure integration, infra exists — but billed honestly)*
`registerToSdfBatch` is **built and validated** and already produces a Hessian; `addEdge` already accepts a full 6×6 info matrix. Wire it into `processRelocResult` as a **refine stage on every accepted appearance match**: PnP `T_query_match` → ICP → emit **one** `BetweenFactor` with information = **appearance-Hessian (strong tangential) ⊕ SDF-ICP-Hessian (strong normal)**. Use the SDF Hessian's **observable subspace** (eigen-analysis), pass only well-conditioned directions, let appearance/IMU carry the tangential null-space (degeneracy-aware fusion: Voxgraph, LP-ICP, Probabilistic Degeneracy Detection 2410.10784). This is **consistent with HR#3** (degradation = covariance inflation, never an `if`).
- **Honest scope (adversarial-corrected):** this fixes the **textured-but-coplanar** slice — where appearance has ≥15 inliers (tangential) but PnP is depth-weak (normal). It does **NOT** resolve the **textureless** planar room: there appearance returns <15 inliers → no accepted match → the refine stage **never fires** → dense-alone is degenerate → the in-plane DOF is **physically unobservable** and the correct behavior is **declare unobservable + soft-chain/dangle**, NOT weld. And it canNOT recover a **zero-overlap opposite-facing** revisit (no geometry to register either). Bill it as a **narrow normal-direction refiner + viewpoint-free re-anchor backstop for has-overlap dangling islands**, not a planar-degeneracy solution.
- **Gate before trusting it:** the two Hessians live in different units (pixel-reprojection vs metric-distance off a 0.15 m TSDF gradient). Summing requires correct relative scaling + a **NEES/chi-square check**, else an overconfident SDF normal pulls the weld with false certainty.

### P2 — Cross-channel agreement gate *(generalizes the proximity 3-tier; cheap)*
Today consensus is *within* one channel. Add: weld immediately when **≥2 of {appearance-VPR, ScanContext, dense-SDF, proximity}** agree on the same submap + correction; hold single-channel matches in the existing candidate tier. **Caveat:** worthless if the channels' false positives are *correlated* (same aliasing) — **measure cross-channel independence first** (a casa A/B).

### P3 — GNC/DCS robust back-end *(defense-in-depth, ship WITH a widened union, not before)*
`pose_graph.hpp:43` already names GNC-TLS as the upgrade. **But I2 is already validated by the gating stack in front of the backend, not the kernel** — so GNC **becomes load-bearing only once P1/P2 widen the multi-channel union and grow the false-positive surface.** Ship it *with* the wider union; do not build it as a standalone win.

### P4 — Register-before-integrate + bound the dense store *(closes leak (c) + the 🔴 dense liability)*
Never integrate a frame whose pose the optimizer has not confirmed for that window. Then apply an **RTAB-style WM/LTM tier to the depth store**: recent depth in RAM for re-integration, sealed/old regions **summarized to a frozen mesh + paged to disk** (no longer re-integrated). Ship pending **C** (suppress duplicate-submap sealing) + **D** (cull backstop) as the area-bound guarantee.

### P5 — Provider fix: D455 IMU calibration → trustworthy cuVSLAM *(odometry/fps win, mis-labeled as immortality)*
**Honest reframe (adversarial-accepted):** slamko is **provider-agnostic and does not do odometry**, so making *one* provider trustworthy is an **fps/quality win, not an immortality-layer win.** It only matters if OKVIS's **~31 fps by-design serial ceiling** (GPU idle ~5%) is your operational blocker. Sequence: `rs-imu-calibration.py` (intrinsic, writes EEPROM) → Kalibr cam-IMU extrinsic + noise model (`allan_variance_ros`) → **re-derive `body_t_cam`/`depth_extrinsic` rotation for the REP-105 base_link** (default identity-rotation `T_SC` is likely wrong → silently bends both maps) → re-record casa → **gate on `cuvslam_nees.py` chi-square + bounded flashbag un-aligned divergence + ATE-neutral vs OKVIS** before mapping cuVSLAM's **flat/instantaneous** covariance into the contract. This is a **prerequisite for a performance upgrade gated by NEES**, not the calibration alone.

### P6 — Wider FOV / 2nd rear camera *(the ONLY true fix for the zero-overlap ceiling — strategic endgame)*
The literature is unanimous (Early Bird, GPR survey, MCOO-SLAM, PAL-SLAM): opposite-facing revisit has **no shared image content → no descriptor recovers it.** FOV is the only physical fix. But it is **hardware**: re-rig, re-calibrate, re-record every bag, and — critically — **unvalidatable on the current `bag play` corpus** (forces the first real-robot dependency). The endgame, not the next move. *(Descriptor swaps — EigenPlaces/MixVPR — are a marginal oblique-revisit bump, a logged stopgap, never the fix; the SALAD/LoFTR builds were already correctly cancelled.)*

### Already-have (don't rebuild)
Honest dangling model · quality-break LOST→RECOVERED · soft-bridge inflated-cov chain · proximity-E 3-tier · ORB-style area-bounding cull · I2 gating stack (PCM streak + teleport bound + coverage gate + 2-vote alias defense) · re-integrate-touched-window dense bend (Family B, externally vindicated) · `cuvslam_nees.py` consistency tooling · the full 6×6 anisotropic-info path in `addEdge`.

---

## 5. The ONE thing to do next

**Build P0.1 — the untrusted-provider coherence gate — wired to break/down-weight rather than silently dangle, and ship it together with P0.2 (enable the disabled OKVIS jerk gates).**

**Why:** it is the binding gap **slamko's own premise demands** ("untrusted provider"), the one cuVSLAM exposed, near-zero hardware cost (the scripts exist), and by slamko's own doctrine it **precedes** every constraint-channel change (P1–P3). Every other improvement propagates corrections through the graph; if the graph ingests a divergent provider segment unflagged, those improvements spread corruption *faster*. It also directly hardens the **only promise slamko can honestly make** — never-lie — by closing leak (b).

**De-risking experiment (measure before build, slamko method):**
1. Replay the **cuVSLAM flashbag** (90 m jump, known divergence) and a **clean OKVIS casa bag** through `cuvslam_nees.py` + `cuvslam_traj_check.py` **offline**.
2. Confirm a **threshold on un-aligned divergence / NEES chi-square separates them cleanly** (flashbag flagged, clean bag passes) — exit-code, not eyeball.
3. Only then wire that threshold as a live edge-acceptance gate in `provider_fusion_node.cpp`. If offline separation is poor, the gate is not yet trustworthy — iterate the metric before spending build effort.

---

## 6. Risks / what to measure

- **Cross-channel false-positive independence (gates P2 & P3).** Do appearance and geometric channels alias on the *same* scenes? If correlated, the cross-channel vote and GNC buy little. **Measure:** disjunctive-union false-positive rate *before* the verification cascade on a self-similar casa segment.
- **P1 Hessian relative-scaling (the silent corruptor).** SDF-ICP normal can be **overconfident** (voxel discretization + TSDF gradient noise). **Measure:** combined-factor **NEES within chi-square**; eigen-spectrum of `H_sdf` (confirm rank-deficient as claimed) + condition number of `H_app+H_sdf` (confirm now well-conditioned) on a textured-coplanar revisit. **Plus the honest negative:** a blank-wall revisit showing 0 inliers → no weld → documented dangle, not "resolved."
- **Adversarial aliasing-injection (S5) — the standing regression slamko owes itself.** I2 is unproven against a synthesized look-alike. **Measure:** false-merge rate on a deliberately self-similar map; differential golden-map T2 overlay.
- **Smooth sub-threshold provider bias (leak b).** A slow scale/yaw ramp passes quality-break. **Measure:** does the P0 coherence gate catch a *smooth* divergence, not just a *jump*? If not, the gate is incomplete.
- **Total live GPU load.** Every run is `--rate 0.5`. cuVSLAM + XFeat reloc + nvblox = three concurrent consumers, **unmeasured on a real robot.** OKVIS's ~31 fps is by-design serial (not contention) — but the aggregate is unproven.
- **cuVSLAM extrinsics (P5 silent-failure).** Wrong `body_t_cam` rotation bends both maps *plausibly-but-systematically* even after IMU calibration. **Measure:** geometric extrinsic validation before any A/B.
- **Recall is the recurring hard limiter, and is hardware-bounded.** Until P6, accept that opposite-facing revisits **dangle honestly** — that is correct behavior (never-lie), not a bug to engineer away with a better descriptor.
- **Doc drift is a live planning hazard.** Reconcile now: `MASTER_PLAN` iSAM2 → Ceres; `PLAN_SLAMKO_TSDF` HITNet/ESS → D455 HW depth; `SYSTEM.md` own-VIO table → provider-adapter. **`STACK_MAP_01.md` is the current source of truth.**

---

### Bottom line
The floor is real and slamko's reframe toward *never-crash + never-lie + super-recoverable* is the **correct, honest goal** — global non-distortion over an untrusted provider is information-theoretically impossible without recognized revisits or absolute references. The architecture (two maps on one re-poseable spine, honest dangling, inline never-lost machinery) is **sound and largely built on the sparse path**. The gap is not novelty — it is **enforcement**: the system *asserts* "untrusted provider" but does not yet *gate* it. Close that gate first (P0), then fuse the already-built dense channel as a **narrow, NEES-gated, anisotropic refiner** (P1) — not as the planar-degeneracy solution it was almost oversold as — and treat wider FOV (P6) as the strategic endgame for the one ceiling no software can break.