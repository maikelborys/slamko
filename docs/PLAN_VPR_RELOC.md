# PLAN — Global VPR front-end for loop-closure relocalization (EigenPlaces)

<!-- status: ONNX exported+validated (GO); C++ integration in progress · 2026-05-28 -->

**Read first:** `slamko_loop/docs/STATUS.md` (the recall study) +
`~/.claude/.../memory/slamko-loopclosure-recall-bottleneck.md`. This plan is
self-contained so a fresh session can finish the integration.

## Why (the proven root cause)
slamko closes loops only via the never-lost seal→weld path, and the weld needs the
relocalizer to recognize a revisited place. **It was proven (offline on the saved Atlas +
3 live magistrale1 runs) that XFeat-64 local descriptors carry NO place-level signal**: a
genuine start-room return scores <1% geometric inliers, indistinguishable from places
80 m away; positive control (sm0 vs sm0 under a known SE3) = 100% → the XFeat-NN + PnP
geometric verifier is perfect, only RETRIEVAL fails. XFeat is too self-similar (intra-set
NN cosine 0.92). **No matcher config fixes this** (ratio 0.9, top_k 25, mutual-NN,
all-submap PnP all gave 0 real return welds). The fix is a global VPR descriptor.

## The validated solution — EigenPlaces (GO)
**EigenPlaces ResNet18, 512-D, MIT** (`gmberton/EigenPlaces`), a learned GLOBAL descriptor.
De-risked offline on the REAL magistrale1 rectified frames (start-room vs return vs far):
**Recall@1 = 0.85, Recall@5 = 1.000, Recall@10 = 1.000**, negative control 0/30 false
matches, separation gap +0.059 (XFeat had +0.02 = none). Caveat: absolute cosines are low
(~0.25, indoor/outdoor domain gap) → retrieve **top-N by ranking + geometric verify**, never
an absolute threshold. EigenPlaces over CosPlace: trained for viewpoint robustness (the
loop-return-from-another-angle case).

Architecture (rebuilt pure-torch; torchvision ABI-mismatches torch 2.9+cu130):
`backbone = ResNet18 children[:-2]` (Sequential conv1,bn1,relu,maxpool,layer1..4) +
`aggregation = Sequential(L2Norm, GeM(p=3), Flatten, Linear(512,512), L2Norm)`.

## Design (per the design agent — the chosen path)
Two-stage retrieval (HF-Net pattern): **global descriptor retrieves candidates → existing
XFeat+PnP verifies geometry.** The geometric back-end and the `Relocalizer`/`RelocResult`
contract + the never-lost supervisor are UNCHANGED. We only add the retrieval stage and
make it **per-keyframe** (the current per-submap reloc is the structural blocker — a
whole-submap aggregate over 30–80 m washes out the place signal).

## Steps + status

### ✅ DONE — model deployment foundation
- `scripts/export_eigenplaces_onnx.py`: rebuilds the arch, loads the released MIT weights
  (zero missing/unexpected keys), exports **single-file ONNX** at fixed 1×3×512×512 →
  512-D. **Must use `dynamo=False`** (torch 2.9 dynamo exporter mis-translates GeM's
  pow-with-learnable-p → parity cos 0.47; legacy gives cos 1.0000, max|Δ| 2.6e-7).
- `slamko_vio/models/eigenplaces.onnx` (43.6 MB, gitignored like xfeat.onnx; regenerate
  with the script + cached weights `~/.cache/torch/hub/checkpoints/ResNet18_512_eigenplaces.pth`).

### TODO — C++ integration
1. **TRT wrapper** `slamko_vio/src/eigenplaces.{cpp,h}` — clone `xfeat.cpp`'s
   build/deserialize/infer skeleton (FP16, engine cache). Simpler: 1 input
   (1×3×512×512), 1 output (1×512), no keypoint head. `process_input`: grayscale →
   resize 512×512 → replicate to 3ch → ImageNet normalize (mean[.485,.456,.406]
   std[.229,.224,.225]) → CHW float. Output already L2-normed.
2. **Compute per keyframe** in `vio_pipeline.cpp` at the KF-insertion site (~line 923 where
   B3 stamps landmark descriptors). Store the 512-D vector on the keyframe. Add a
   `global_desc` (Eigen::VectorXf, 512) to the keyframe/frame representation; also compute
   it for the live QUERY frame each reloc attempt.
3. **Carry it across the contract:** add `Eigen::VectorXf global_descriptor` to
   `slamko_core` `Features` (query) and a per-keyframe global-desc block to `SubMap`
   (`Eigen::Matrix<float,Dyn,512>` aligned with `SubMap.keyframes`); bump `submap_io`
   schema (write/read the block) so prior maps carry VPR vectors cross-session.
4. **GlobalDescriptorDB** in `slamko_loop` (sibling of `BowDatabase` in `bow.hpp`): on
   `addSubMap`, push each keyframe's 512-D vector tagged `(submap_id)`. `query(qvec, topN)`
   = cosine NN (single GEMM) → top-N submap ids (dedup).
5. **Relocalizer retrieval swap** in `xfeat_relocalizer.cpp::relocalize`: replace the BoW
   pre-selector with `GlobalDescriptorDB::query(query.global_descriptor, top_n=10)` →
   candidate submaps → the UNCHANGED XFeat-NN + PnP-RANSAC verify. Keep all-submap fallback.
   (Retire/disable the hand-rolled BoW — it's dead weight per the recall study.)
6. **Wiring:** `vio_node` builds the query Features with the global descriptor; pass the
   EigenPlaces engine handle (composition root). Add a node param/launch arg for the model
   path + an enable flag (`reloc_use_vpr`, default on when the engine is present).

### Validation gate (Hard Rule #5: report BOTH)
Re-run TUM VI magistrale1. Target: welds fire on the **return** to the start room
(end-room ATE ≈ start-room ATE), Sim3-ATE + un-aligned SE3-ATE approaching OKVIS's
**8.6 cm** (vs the current ~8–24 m no-close). Then the magistrale1↔magistrale2 cross-session
fuse (the original goal).

## NEXT (verification fix) — LighterGlue matcher (the start-room return needs this)

VPR retrieval works (ATE 24m→1.9m) but the start-room return doesn't re-close: the
geometric VERIFICATION (brute-force XFeat-NN + Lowe ratio) can't match the same place
across a large viewpoint/time gap. Fix = a learned matcher: **LighterGlue** (LightGlue
arch + verlab XFeat-64 weights), the user's matcher in `orbslam3_xfeat`/`AirSLAM_XFEAT`.

**Facts (from the orbslam3_xfeat investigation):**
- Model: `lighterglue.pt` (TorchScript, 4.6 MB, Apache-2.0) — in-repo at
  `~/coding/AirSLAM_XFEAT/output/lighterglue.pt` and `~/coding/orbslam3_xfeat/orbslam3_xfeat_core/weights/lighterglue.pt`.
  Source weights `~/.cache/.../xfeat-lighterglue.pt`. Re-export: `AirSLAM_XFEAT/scripts/export_lighterglue_torchscript.py`.
- **Runtime = LibTorch TorchScript, NOT ONNX/TRT** (kornia LightGlue uses negative-index
  ops + FlashAttention → ONNX export abandoned in both repos). Clone target =
  `~/coding/AirSLAM_XFEAT/src/lighter_glue.cc` (+ `include/lighter_glue.h`), Apache-2.0.
- I/O: in `kpts0(1,N,2) f32`, `desc0(1,N,64) f32` (L2-normed), `kpts1`, `desc1`; out
  TorchScript tuple `matches0(1,N) int64` (index into img1 or −1), `mscores0(1,N) f32`.
  **N=512 baked into the trace; image_size (752,480) baked** (matches rectified TUM VI).
- slamko `Features.descriptors` is N×64 row-major = `(N,64)` byte layout → `from_blob`
  directly, NO transpose. keypoints = cols 0-1 of the N×3 block.
- Precedent gate (`orbslam3_xfeat .../LighterGlueRefiner.cc:203-242`): LG match (score≥0.2)
  → reproject via the pose hypothesis → inlier if residual <5px → accept if ≥15 inliers.
  Same as slamko's pnpRansac inlier logic.

**Steps:**
1. `slamko_loop/CMakeLists.txt`: optional `SLAMKO_LOOP_WITH_TORCH` target (`find_package(Torch)`,
   `-DSLAMKO_HAVE_TORCH`); guard the matcher with `#ifdef` so the default build stays torch-free.
2. `LightGlueMatcher` (`slamko_loop/src/lightglue_matcher.cpp` + hpp) implementing
   `slamko::Matcher` — near-verbatim port of `lighter_glue.cc`: ctor `torch::jit::load`,
   `match(query,train)` → pad to N=512 → `from_blob` 4 tensors → `forward` → decode →
   `DescMatch{query_idx,train_idx,score}`.
3. Wire into `xfeat_relocalizer.cpp::relocalize`: when enabled, replace `matchDescriptors()`
   with `LightGlueMatcher::match(query, entry.train)` → build `uv`/`X` from the matches →
   feed the UNCHANGED `pnpRansac()` + inlier gates. Cache a `Features train` in `Entry` at
   `addSubMap()` (LightGlue needs the train side's keypoint geometry, not just descriptors).
4. Config gate `use_lightglue` (default off → brute-force fallback). A/B on the known-failing
   start-room return: inlier count must clear `min_inliers=15`.

**Gotchas:** trace N=512 but slamko XFeat `max_corners=1500` → **top-K to 512 by score**
before matching (or re-export at higher N, ~4× slower). Run off the hot path (reloc stage only).

## Beyond verification: OKVIS-class GLOBAL ALIGNMENT (the road from 1.9 m → cm)

LighterGlue makes loops FIRE; getting the map as *aligned* as OKVIS2-X (8.6 cm magistrale1)
needs the alignment mechanism too. From the OKVIS root investigation — **supereight submap
stabilization is NOT the trick** (it consumes corrected poses; the optional submap-ICP
feedback is off by default; OKVIS's 3.22 cm is the SPARSE binary, no supereight). What
matters, ranked:

1. **Global BA on LANDMARKS (dominant gap).** slamko's loop closure is anchors-only
   pose-graph (chain of SE3 nodes); OKVIS un-marginalizes the loop keyframes back into
   **reprojection errors** and reoptimizes landmarks+poses+IMU (`ViSlamBackend::optimiseFullGraph`
   1971-2003, `addLoopClosureFrame`→`convertToObservations` 1445-1460). This is why slamko
   lands at 1.9 m not cm. **slamko_fusion already has GTSAM** — host a global VI-BA there
   (structure+pose factors), not in the loop pose-graph.
2. **Closed-form segment-rigid weighted distribution** applied instantly before the async
   BA (`attemptLoopClosure` 2700-2734): translation weighted by per-segment distance,
   rotation uniform, previously-closed loops kept RIGID via a `loopId` group so a new
   closure can't smear old good geometry.
3. **Reversible Schur marginalization** (`TwoPoseGraphError` 354-385/399): dropped KFs →
   relative-pose factors, re-expandable to reprojection errors when a later closure touches
   them (so the global BA can reoptimize old structure). + freeze the window edge (FEJ).
4. **Distance-scaled drift gate** (`attemptLoopClosure` 2655-2691): reject a closure if
   `‖dr_W‖/dist > 1.35%/100 + 2%·scale + 8%/√numSteps` (+ 3σ covariance test). Quality gate
   — keep slamko's separate place-rec path for the catastrophic-jump never-lost case.

This is a multi-step roadmap (LighterGlue verify → global landmark BA is the big one). The
substrate (auto-seal + chain pose-graph + VPR retrieval) is done; #1 (BA) is the major build.

## Risks
- **Domain gap** (main): outdoor-trained weights on indoor TUM VI — de-risk said GO
  (ranking is decisive even with compressed cosines), but if live recall is weak, fine-tune
  on EuRoC/TUM-VI (license-clean) or fall back to AnyLoc/DINOv2 (Apache).
- **Input-format parity**: grayscale→3ch + resize + normalize in C++ MUST match the export
  preprocessing or recall collapses silently — verify the C++ wrapper output numerically
  against the python reference on one frame before trusting a full run.
- **Threshold**: gate by top-N ranking + geometric verify, NOT an absolute cosine.
