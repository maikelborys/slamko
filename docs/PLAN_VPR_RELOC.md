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

## Risks
- **Domain gap** (main): outdoor-trained weights on indoor TUM VI — de-risk said GO
  (ranking is decisive even with compressed cosines), but if live recall is weak, fine-tune
  on EuRoC/TUM-VI (license-clean) or fall back to AnyLoc/DINOv2 (Apache).
- **Input-format parity**: grayscale→3ch + resize + normalize in C++ MUST match the export
  preprocessing or recall collapses silently — verify the C++ wrapper output numerically
  against the python reference on one frame before trusting a full run.
- **Threshold**: gate by top-N ranking + geometric verify, NOT an absolute cosine.
