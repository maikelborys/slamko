// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// LightGlueMatcher — the viewpoint-robust descriptor matcher for relocalization
// / loop closure (implements slamko_core::Matcher). It is LighterGlue: the
// kornia/cvg LightGlue attention architecture loaded with the verlab XFeat-64
// weights, run through a TorchScript .pt via libtorch (NOT ONNX/TRT — kornia
// LightGlue uses negative-index ops + FlashAttention that the ONNX exporter
// can't lower; torch.jit.trace records the raw aten ops). The .pt is produced by
// AirSLAM_XFEAT/scripts/export_lighterglue_torchscript.py.
//
// WHY this exists: XFeat-64 + brute-force NN (Lowe ratio) — the previous verify
// path — cannot match the SAME place across a large viewpoint/time gap (proven:
// a genuine start-room return scores <1% inliers, the crux of the loop-closure
// recall study). LighterGlue's learned cross-attention solves exactly that hard
// revisit. It REPLACES the brute-force NN inside the relocalizer's verify stage;
// the PnP-RANSAC geometric back-end is unchanged.
//
// Build: optional. Only compiled with -DSLAMKO_LOOP_WITH_TORCH (sets
// SLAMKO_HAVE_TORCH + links libtorch). Without it, build() returns false and
// match() returns {} — a working no-op so the relocalizer falls back to
// brute-force NN and the default torch-free build (incl. the gtest suite) is
// unaffected. The header is torch-free (pimpl) so it includes anywhere.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "slamko_core/matcher.hpp"

namespace slamko {

struct LightGlueConfig {
  std::string model_path;        // TorchScript .pt (lighterglue.pt)
  int   trace_n = 512;           // keypoint count baked into the export trace
  float score_thresh = 0.10f;    // mscores0 cut (kornia/cvg default)
  bool  use_cuda = true;         // run on GPU when available
};

class LightGlueMatcher : public Matcher {
 public:
  explicit LightGlueMatcher(const LightGlueConfig& cfg);
  ~LightGlueMatcher() override;

  std::string name() const override { return "lighterglue"; }

  // Load + deserialize the TorchScript module. Returns false (and stays
  // unusable) when libtorch is absent or the model fails to load — callers
  // must check this and fall back. Cheap to call once at construction.
  bool build();
  bool ready() const { return ready_; }

  // Match query → train. Uses cols 0,1 of each Features.keypoints as pixel
  // coords and the N×64 descriptors. If either side has more than trace_n
  // keypoints, the top trace_n by keypoint score (col 2) are used; returned
  // DescMatch.{query_idx,train_idx} always index the ORIGINAL passed rows.
  std::vector<DescMatch> match(const Features& query,
                               const Features& train) override;

 private:
  LightGlueConfig cfg_;
  bool ready_ = false;
  struct Impl;                   // holds the torch module; defined only under
  std::unique_ptr<Impl> impl_;   // SLAMKO_HAVE_TORCH (else nullptr).
};

}  // namespace slamko
