// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// SensorFrontend — Tier-1 plugin. Turns a Measurement into nodes + Factors on
// the backend (MASTER_PLAN §1). Adding a sensor is registerFrontend(...), never
// a rewrite. The frontend supplies its own covariance (via each Factor's
// √information); it never touches the solver internals.

#pragma once

#include <string>

#include "slamko_core/custom_data.hpp"
#include "slamko_core/factor_graph_backend.hpp"
#include "slamko_core/node_key.hpp"

namespace slamko {

// A timestamped sensor reading. Concrete frontends downcast / read from
// custom_data; the contract only fixes the stamp so the timeline can align it.
struct Measurement {
  double timestamp = 0.0;
  CustomData custom_data;
  virtual ~Measurement() = default;
};

// Maps measurement time to the graph nodes the frontend should attach to. The
// backend/estimator implements it (it owns keyframe insertion); the frontend
// only queries it. Kept minimal — grows as multi-sensor timing needs land.
class KeyframeTimeline {
 public:
  virtual ~KeyframeTimeline() = default;
  // The most recent keyframe pose node (or an invalid key if none yet).
  virtual NodeKey latestPose() const = 0;
  // The pose node at/just-before `stamp`; frontends interpolate as needed.
  virtual NodeKey poseAt(double stamp) const = 0;
  virtual bool empty() const = 0;
};

class SensorFrontend {
 public:
  virtual ~SensorFrontend() = default;
  virtual std::string name() const = 0;
  // Create nodes + emit factors for this measurement onto `backend`.
  virtual void process(const Measurement& m, const KeyframeTimeline& timeline,
                       FactorGraphBackend& backend) = 0;
};

}  // namespace slamko
