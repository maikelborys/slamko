// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// FeatureSource — the swappable detector/descriptor seam (MASTER_PLAN P0,
// "feature compare-all"). Shi-Tomasi (descriptor-less), XFeat (TensorRT), and
// LiftFeat-m1 (libtorch) each implement this; the VIO frontend and the
// relocalizer pick one by registration, never by an if-branch. Concrete impls
// (and their CUDA/TRT/torch deps) live in slamko_vio as optional build targets,
// behind this Eigen-only contract.

#pragma once

#include <string>

#include "slamko_core/features.hpp"
#include "slamko_core/image_view.hpp"

namespace slamko {

class FeatureSource {
 public:
  virtual ~FeatureSource() = default;
  virtual std::string name() const = 0;

  // Detect (and, if the backend supports it, describe) features in `image`.
  // descriptorDim() == 0 for descriptor-less sources (Shi-Tomasi).
  virtual Features detect(const ImageView& image) = 0;

  // Descriptor dimensionality (0 = none, 64 = XFeat/LiftFeat).
  virtual int descriptorDim() const = 0;
};

}  // namespace slamko
