// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// AnchorGate — the lazy-anchor cluster gate, THE defense against a false
// relocalization corrupting the map (slamko's analog of OKVIS2-X's drift-budget
// gate). A weld is applied only once `weld_min_matches` place-rec candidates
// AGREE — their relative-pose estimates cluster within a translation+rotation
// radius. It is MULTI-CLUSTER (RANSAC-like): each candidate joins an agreeing
// cluster or seeds a new one, so a bad candidate arriving first can't poison the
// consensus — scattered hits (perceptual aliasing) form singleton clusters that
// never reach the threshold. Runs OUTSIDE the estimator graph (DigiForest).

#pragma once

#include <cstdint>
#include <vector>

#include "slamko_core/se3.hpp"

#include "slamko_loop/supervisor_state.hpp"

namespace slamko {

// One place-rec hit, ready for the gate: where the active(query) frame sits
// relative to a sealed submap, plus the verification strength.
struct WeldCandidate {
  std::uint64_t sealed_submap_id = 0;
  SE3    T_active_sealed;  // active(query) → sealed-local (== RelocResult.T_query_match)
  double confidence = 0.0;
  int    inliers = 0;
};

class AnchorGate {
 public:
  explicit AnchorGate(const SupervisorConfig& cfg) : cfg_(cfg) {}

  // Push one candidate. Returns true once any cluster of agreeing candidates
  // reaches weld_min_matches, outputting the cluster's manifold-mean transform +
  // the sealed submap it welds to, then resets. Candidates failing the
  // inlier/confidence floor are dropped before clustering.
  bool tryCluster(const WeldCandidate& c, SE3& consensus_out,
                  std::uint64_t& sealed_id_out);

  void reset() { clusters_.clear(); }
  std::size_t clusterCount() const { return clusters_.size(); }

 private:
  struct Cluster {
    std::uint64_t   sealed_id = 0;
    std::vector<SE3> members;  // members[0] is the seed
  };

  SupervisorConfig     cfg_;
  std::vector<Cluster> clusters_;
};

}  // namespace slamko
