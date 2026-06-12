// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// LoopConsensusGate — the PCM-lite acceptance gate for loop-closure / anchor
// candidates (MASTER_PLAN §0.4 "covariance-gated, inertially sanity-checked").
//
// THE lesson it encodes (CASA1_Suave loop3/4, 2026-06-12): an ABSOLUTE
// disagree-with-the-graph gate rejects exactly the loops that matter — the
// graph is wrong BY the drift the loop is supposed to correct. The
// drift-magnitude-agnostic test is PAIRWISE CONSISTENCY: require K consecutive
// candidates against the same target whose measurements track the provider's
// relative odometry between the query frames:
//     meas_k ≈ meas_{k-1} ∘ T_{q_{k-1} q_k}(provider).
// True matches move with the robot; PnP locks on aliased / self-similar
// structure jitter and never build a streak. Pure header (Eigen + SE3 only),
// no ROS — unit-tested in test_loop_consensus.cpp; the ROS node is a shell.

#pragma once

#include <cstdint>
#include <unordered_map>

#include "slamko_core/se3.hpp"

namespace slamko {

struct LoopConsensusConfig {
  double tol_t = 0.30;        // pairwise translation tolerance [m]
  double tol_r = 0.15;        // pairwise rotation tolerance [rad]
  int required_consec = 3;    // streak length that accepts
  double cooldown_s = 2.0;    // min time between ACCEPTED loops (any target)
};

struct LoopCandidate {
  std::uint64_t target_id = 0;  // matched submap (or prior-map submap)
  std::uint64_t q_id = 0;       // query keyframe
  double t = 0.0;               // query timestamp [s]
  SE3 meas;                     // measured query pose in the target's frame
  SE3 T_OB;                     // query pose in the provider's odom frame
};

class LoopConsensusGate {
 public:
  explicit LoopConsensusGate(LoopConsensusConfig cfg = {}) : cfg_(cfg) {}

  // Feed one verified candidate. Returns true exactly when this candidate
  // completes a consistent streak AND the cooldown allows an acceptance —
  // the caller then turns THIS candidate into a graph edge / anchor.
  bool feed(const LoopCandidate& c) {
    auto& s = streaks_[c.target_id];
    if (s.consec > 0) {
      const SE3 pred = s.meas * (s.T_OB.inverse() * c.T_OB);
      const SE3 d = pred.inverse() * c.meas;
      if (d.translation().norm() < cfg_.tol_t && d.so3().log().norm() < cfg_.tol_r)
        s.consec += 1;
      else
        s.consec = 1;  // inconsistent — restart the streak from this candidate
    } else {
      s.consec = 1;
    }
    s.meas = c.meas;
    s.T_OB = c.T_OB;
    if (s.consec < cfg_.required_consec) return false;
    if (c.t - last_accept_t_ < cfg_.cooldown_s) return false;
    last_accept_t_ = c.t;
    return true;
  }

  int streak(std::uint64_t target_id) const {
    const auto it = streaks_.find(target_id);
    return it == streaks_.end() ? 0 : it->second.consec;
  }
  void reset() { streaks_.clear(); last_accept_t_ = -1e18; }

 private:
  struct Streak {
    int consec = 0;
    SE3 meas, T_OB;
  };
  LoopConsensusConfig cfg_;
  std::unordered_map<std::uint64_t, Streak> streaks_;
  double last_accept_t_ = -1e18;
};

}  // namespace slamko
