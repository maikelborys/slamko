// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// AnchorGate — see header. Consensus is a one-shot tangent (manifold) mean of the
// agreeing cluster: mean = T0 · exp( (1/N) Σ log(T0⁻¹·Ti) ). For a tight cluster
// this is exact to machine precision (and exact when all members are identical —
// the deterministic test relies on that).

#include "slamko_loop/anchor_gate.hpp"

namespace slamko {

namespace {
// Geodesic disagreement between two poses: translation + rotation of the relative
// transform. Decides cluster membership.
void relErr(const SE3& a, const SE3& b, double& trans, double& rot) {
  const SE3 rel = a.inverse() * b;
  trans = rel.translation().norm();
  rot   = rel.so3().log().norm();
}

SE3 tangentMean(const std::vector<SE3>& v) {
  Vector6d acc = Vector6d::Zero();
  for (const auto& T : v) acc += (v.front().inverse() * T).log();
  acc /= static_cast<double>(v.size());
  return v.front() * SE3::exp(acc);
}
}  // namespace

bool AnchorGate::tryCluster(const WeldCandidate& c, SE3& consensus_out,
                            std::uint64_t& sealed_id_out) {
  // Pre-cluster verification floor (OKVIS2-X min-inlier gate). Reject weak matches
  // before they form a cluster at all.
  if (c.inliers < cfg_.weld_min_inliers || c.confidence < cfg_.weld_min_confidence)
    return false;

  // Join the first agreeing cluster (same sealed map, within radius of its seed),
  // else seed a new one. RANSAC-like: outlier ordering can't poison the consensus.
  Cluster* hit = nullptr;
  for (auto& cl : clusters_) {
    if (cl.sealed_id != c.sealed_submap_id) continue;
    double trans = 0.0, rot = 0.0;
    relErr(cl.members.front(), c.T_active_sealed, trans, rot);
    if (trans <= cfg_.weld_cluster_radius_m && rot <= cfg_.weld_cluster_radius_rad) {
      hit = &cl;
      break;
    }
  }
  if (hit) {
    hit->members.push_back(c.T_active_sealed);
  } else {
    clusters_.push_back(Cluster{c.sealed_submap_id, {c.T_active_sealed}});
    hit = &clusters_.back();
  }

  if (static_cast<int>(hit->members.size()) >= cfg_.weld_min_matches) {
    consensus_out = tangentMean(hit->members);
    sealed_id_out = hit->sealed_id;
    reset();  // one weld per consensus; start fresh for the next recovery
    return true;
  }
  return false;
}

}  // namespace slamko
