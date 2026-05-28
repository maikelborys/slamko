# PLAN — Global landmark BA (OKVIS-class alignment, the 1.9 m → cm gap)

<!-- status: A+B v1 implemented + tested (synthetic green); D validated on V1_03 (visual-only BA DEGRADES ATE → IMU factors B.2 required) · 2026-05-28 -->

**Read first:** `docs/PLAN_VPR_RELOC.md` (the LighterGlue verifier — landed),
`slamko_loop/docs/STATUS.md` 2026-05-28 (the verify path + walls),
`slamko_vio/docs/STATUS.md` 2026-05-28 (determinism fix — *prerequisite* for being able
to MEASURE BA improvement reproducibly; memory `slamko-vio-replay-nondeterministic`).

## Why this exists

slamko's loop closure is **anchors-only pose-graph** (chain of SE3 nodes + identity
sequential edges + loop edges; optimize over poses only). On EuRoC V1_03 it improves
raw 58 cm → 39 cm (~33%) reproducibly. But that's still ~5× OKVIS's cm-class numbers.

The dominant gap (per the PLAN_VPR_RELOC investigation of OKVIS root):
**OKVIS un-marginalizes the loop keyframes back into REPROJECTION ERRORS and reoptimizes
LANDMARKS + poses + IMU** (`ViSlamBackend::optimiseFullGraph` / `addLoopClosureFrame` /
`convertToObservations`). slamko has no such structure-level pass — closures only nudge
anchors, leaving landmarks where they were. That's the 39 cm vs cm gap.

slamko_fusion already hosts GTSAM (`GtsamLocalSmoother`). The BA goes there — NOT in
the loop pose-graph (which should stay disposable, Hard Rule #4 of the master plan).

## Prerequisite that's also reusable (Phase A)

The SubMap currently stores **flat landmarks** (3D submap-local + ONE descriptor each)
and **keyframe poses** (now populated, post-LighterGlue work). It does NOT store
**per-keyframe 2D observations** of those landmarks. BA needs them — each landmark `l`
seen in keyframes `K = {k1, k2, ...}` becomes a reprojection factor `||π(T_W_k · X_l) - uv_lk||`
in each `k ∈ K`. The same per-KF observation structure is also what makes **real
two-image LightGlue** match query↔keyframe (the in-distribution way to use LightGlue,
the hard-revisit recall fix). So Phase A unblocks BOTH paths.

Phase A — `SubMap` schema (slamko_core):
```cpp
struct KeyframeObservations {       // aligned 1:1 with SubMap.keyframes
  std::vector<std::uint64_t> landmark_ids;                                       // N
  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> uv;                   // N×2 (left)
  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> uv_right;             // N×2 (stereo, optional)
};
struct SubMap { ...existing...; std::vector<KeyframeObservations> kf_obs; };
```
Descriptor lookup goes via the landmark's existing `descriptor_row` (one descriptor per
landmark — slamko's "moment-of-attachment" snapshot). Per-KF descriptor variants are a
later concern. `uv_right` is optional (empty Matrix when stereo obs absent) so a mono-only
future surface still fits.

submap_io bump **SMP2 → SMP3**: append `nk` `KeyframeObservations` blocks after the global
descriptor. Load: accept SMP1/SMP2 with empty `kf_obs`; SMP3 reads the per-KF arrays.
Round-trip test in `slamko_core` matching the existing schema tests.

VIO capture (`slamko_vio/src/vio_pipeline.cpp`): the data is already there — the
`observations` vector at KF insertion (`StereoObservation`) carries `landmark_id`, `uv`,
`uv_right`. Per-KF append to `kf_observations_` (member, epoch-tagged like `kf_poses_`).
`buildSubMap()` emits the current epoch's `kf_obs` aligned with `keyframes`.

## Phase B — GlobalSmoother in slamko_fusion (the BA itself)

New class `slamko_fusion::GtsamGlobalSmoother` (sibling of `GtsamLocalSmoother`).
Interface (slamko_core contract TBD):

```cpp
class GlobalSmoother {
 public:
  virtual ~GlobalSmoother() = default;
  virtual void setStereoCalib(const StereoCalib&) = 0;
  virtual void setExtrinsics(const SE3& T_BS) = 0;
  // Build the graph from a set of submaps + a loop closure constraint
  // (T_q_match between two KFs in different submaps) and run LM.
  // Refined poses/landmarks are written back into the submaps in place.
  virtual bool optimizeLoopClosure(std::vector<SubMap>& submaps,
                                   std::uint64_t kf_id_from, std::uint64_t kf_id_to,
                                   const SE3& T_q_match) = 0;
};
```

GTSAM graph (visual-only first; IMU factors deferred to Phase B2):
- Variables: `Pose3` per KF, `Point3` per landmark (anchored to one — fix the first KF's pose).
- Factors: `GenericStereoFactor<Pose3, Cal3_S2Stereo>` for each `(kf, landmark) ∈ kf_obs`,
  using the stored uv/uv_right + the stereo calib. Noise model = unit-pixel covariance.
- The loop closure: a `BetweenFactor<Pose3>` between `kf_id_from` and `kf_id_to` with
  small noise (high-confidence weld), OR — better — re-project each matched landmark
  from the query KF using the loop's matched 3D and let the reprojection factors do
  the work (the OKVIS "un-marginalize" approach).
- Optimizer: Levenberg-Marquardt, capped iterations (e.g., 30), early termination.
- Write back: refined `T_WB` per KF, refined landmark `position`. Anchors recomputed
  from the refined first-KF poses of each submap.

Phase B2 (later): IMU factors between consecutive KFs. Needs per-KF IMU samples stored
in SubMap (schema bump SMP4→SMP5; SMP4 is per-KF VPR, see V.1 below) + bias variables.
Visual-only BA already improves geometry; IMU factors close the scale/bias drift.

## Phase C — Wiring (never_lost_supervisor)

On a successful weld (`attemptWeld` → true), the supervisor currently runs the
anchor-only `pose_graph.optimize()`. Add an optional global-BA path: after pose-graph
distribution, call `global_smoother->optimizeLoopClosure(...)` to reproject + refine
landmarks. Gated by a config flag `use_global_ba` so it's opt-in until validated.

Cost: BA is heavier than pose-graph (seconds, not milliseconds on the affected KFs +
landmarks). Run **async** (off the supervisor thread); the live anchor stays valid via
the cheap pose-graph until BA finishes and updates anchors atomically. Same async
pattern OKVIS uses (`optimiseFullGraph` runs on a separate thread).

## Phase D — Validation (now actually measurable, post-determinism fix)

EuRoC V1_03 corrected ATE target: 39 cm (pose-graph baseline) → **single-digit cm**.
EuRoC MH_05 corrected ATE: from 22 cm raw → cm-class (welds + BA together).
Magistrale: still GT-unmeasurable until a valid-segment mask is built (see memory
`slamko-tumvi-magistrale-gt-roomonly`).

Reproducibility is now real (post-determinism fixes): two runs of the same config
agree on corrected ATE to ~0.1 cm, so a BA improvement of even a few cm will be
clearly distinguishable from run noise.

## Risks / open questions

- **Anchor consistency across submaps.** BA refines KF poses (submap-local); the
  anchors must be recomputed so the corrected global map stays consistent. Cleanest:
  fix the first KF of the FIRST submap as the global origin; every other submap's
  anchor = (its first KF pose in the global frame) · (its first KF pose in submap-local)⁻¹.
- **Re-marginalization vs full BA.** OKVIS uses reversible Schur marginalization
  (`TwoPoseGraphError`) — un-marginalize loop KFs back to reprojection factors at
  closure time. v1 = plain full BA on the affected KFs (no marginalization); add the
  Schur path later if BA cost becomes prohibitive.
- **GTSAM ISAM2 vs LM.** ISAM2 is incremental (faster for online); LM is a one-shot
  batch (cleaner for a closure event). v1 = LM batch on closure; ISAM2 later for live
  incremental BA.
- **Landmark observation count.** Each KF has ~500-1500 stereo obs → ~10 KFs/submap ×
  ~500 obs = ~5k factors per submap. Two submaps in a closure = ~10k factors. LM at
  this scale runs in seconds — acceptable async.

## Phases (incremental, each ends with a green commit)

| Phase | Scope | Deliverable | Status |
|-------|-------|-------------|--------|
| A.1   | Schema: per-KF observations in SubMap + submap_io SMP3 + round-trip test | slamko_core change | ✅ `9b5ccbd` |
| A.2   | VIO capture: vio_pipeline populates `kf_obs` from existing `observations` | slamko_vio change + e2e test | ✅ `9b5ccbd` (V1_01 → 1053 obs persisted) |
| B.1   | GlobalSmoother contract in slamko_core; GtsamGlobalSmoother (visual-only, LM) in slamko_fusion | new class + unit test | ✅ `73889a7` (2 synthetic tests → truth) |
| C.1 (offline) | Per-submap BA tool `offline_ba` + calib sidecar from VIO | validates BA on real data without supervisor surgery | ✅ this commit (Huber on stereo residuals) |
| D.1   | EuRoC V1_03 corrected-ATE A/B on the offline tool | **NEGATIVE result — see below** | ✅ measured |
| V.1   | **Per-KF VPR retrieval** — SMP4 (per-KF EigenPlaces descriptor) + relocalizer max-cosine ranking | granularity fix for hard revisits | ✅ this commit (3 new tests; EuRoC V1_01→V1_02 cross-session PASS 27.1 cm; magistrale1: 5 welds early, 0 return — same as baseline → V.2) |
| V.2   | Diagnose per-KF VPR on magistrale return: dump cosine scores + reloc-attempt counts during the return | data to decide model swap vs supervisor fix | NEXT |
| B.2   | IMU factors (schema SMP5 + factor construction) — **the actual ATE fix** | per-KF IMU samples + CombinedImuFactor | parallel |
| C.live | Wire BA into never_lost_supervisor (post-Phase B.2) | supervisor change, async | TBD |

## V.1 — Per-KF VPR (granularity fix, IMPLEMENTED this commit)

The b07b484 finding ("real-KF LightGlue is dense but submap 0 never enters the VPR top-30 at the
magistrale return") had two interpretations: (a) swap EigenPlaces for a stronger model, or (b)
keep EigenPlaces but change retrieval granularity. **The data said (b) first** — and that's what
this phase landed:

**Why granularity, not the model:**
- The offline validation in memory `slamko-loopclosure-recall-bottleneck` proved
  **EigenPlaces Recall@5 = 1.0 / 0-of-30 false matches** on real magistrale1 return frames.
  The model CAN distinguish places on this data when given clean per-frame queries.
- The live pipeline was storing **ONE aggregated EigenPlaces descriptor per submap** (a single
  signature over 10 m of trajectory, mixing room + corridor + hallway). Per-keyframe retrieval
  (each KF its own descriptor) gives ~10× finer granularity AND matches the offline-validated
  usage pattern.

**What landed (this commit):**
- `slamko_core` SMP3 → **SMP4** (additive). `KeyframeObservations` gains
  `Eigen::VectorXf global_descriptor` (per-KF VPR vector, L2-normalized). Codec writes
  `kf_gdim · floats` at the end of each kf_obs block; old SMP3 reads still work (per-KF empty).
- `slamko_vio` `vio_pipeline.cpp` captures `current_global_desc_` into the EpochKf at KF
  insertion. `buildSubMap` already copies `kf_obs`, so per-KF descriptors flow into the saved
  Atlas via the existing path.
- `slamko_loop` `xfeat_relocalizer` caches `kf_global_desc` per Entry; the VPR ranker scores
  `score(submap) = max_k cosine(query, kf_global_desc[k])`. Per-submap `global_descriptor`
  stays as the SMP3-fallback for legacy Atlases.
- 3 new gtests (SMP4 round-trip + per-KF top-N ranking + per-submap fallback) pass.
- **EuRoC V1_01→V1_02 cross-session smoke**: PASS, corrected ATE 27.1 cm (no regression).
- **magistrale1 (1500 s replay, 88 submaps sealed)**: 5 welds — **all in the first 90 s**
  (start-room re-visits at submap 0/1/3), **0 on the return**. Same shape as the baseline
  (6 welds, 0 returns). Per-KF VPR is now the substrate but **doesn't move the magistrale
  needle on its own**: the next-step diagnosis (V.2) instruments the per-KF cosine scores
  during the return to decide whether granularity alone is insufficient (→ swap model) or
  the relocalizer never even gets asked (supervisor throttle / state issue).

**If per-KF EigenPlaces still falls short on hard revisits:** swap the model. Top candidates
ranked by indoor robustness + license + ONNX-exportability:
1. **AnyLoc** (DINOv2 ViT-L/14, Apache-2.0) — generalist foundation model, no fine-tune needed.
2. **SALAD** (DINOv2 + SALAD aggregator, Apache) — SOTA on most indoor benchmarks.
3. **MixVPR** (Apache) — strong + smaller than DINOv2-based options.

EigenPlaces is trained on outdoor city scenes (SF, Tokyo) so it IS technically out-of-distribution
on TUM VI indoor, but the offline data shows the OOD gap isn't a blocker when granularity is right.
Swap is the bigger commit (new ONNX export pipeline, new model param). Keep it on the back burner
until per-KF EigenPlaces is benched on the magistrale return.

## D.1 result — visual-only BA degrades ATE (= the case for IMU factors)

**V1_03_difficult, per-submap visual-only BA on the saved Atlas (8 submaps):**
- Original Atlas → TUM → ATE: **Sim3-ATE 36.95 cm, scale 1.0138, SE3-ATE 37.01 cm.**
- BA-refined Atlas (Huber stereo loss) → ATE: **Sim3-ATE 71.91 cm, scale 0.7817, SE3-ATE 82.09 cm.**
- BA mechanically converges (total cost 8.8 M → 2.7 M, every submap improves locally).

**Diagnosis (the textbook "VI-BA, not BA" finding):** stereo factors pin metric scale
in principle (baseline 0.110 m), but on short visual baselines (10 m intra-submap, small
parallax) the visual-only cost surface has near-degenerate directions for rotation +
scale. Without IMU regularizing those, LM finds a lower-cost optimum that has WORSE
metric pose accuracy — the trajectory shrinks by ~25 % (scale 1.01 → 0.78) systematically
across all submaps. Huber loss handles the bootstrap-submap stale-pose outliers (initial
cost 165 B → 6 M, no rescue needed) but doesn't fix the structural degeneracy.

**The fix is IMU factors (Phase B.2):** persist per-KF IMU samples (`SubMap` schema bump
SMP4 → SMP5), construct `CombinedImuFactor` between consecutive KFs (the same factor
`GtsamLocalSmoother` already uses live), gauge-anchor the first KF + first velocity +
first bias. IMU constrains: scale (gravity magnitude), rotation about gravity (yaw
through accel bias), and angular velocity (gyro). With those constraints the visual
factors land on the metric optimum. OKVIS's 8.6 cm magistrale1 is VI-BA, not visual.

## Conventions / Hard rules respected

- slamko_fusion hosts the BA (GTSAM dependency stays there, Hard Rule #2 — no
  cross-package internals; loop only sees a `slamko_core::GlobalSmoother` interface).
- The global pose-graph stays disposable; BA refines poses + landmarks but anchors
  remain the cross-submap coupling (Hard Rule #4).
- Apache/BSD only (GTSAM = BSD-3 ✓). No new GPL deps.
- Hard Rule #5: report BOTH Sim3-aligned and un-aligned ATE in the validation table.
