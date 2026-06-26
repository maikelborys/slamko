// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// GTSAM Levenberg-Marquardt backend for PoseGraph::optimize() — the step toward the MASTER_PLAN
// P-C′ iSAM2 incremental smoother. GTSAM is purpose-built for robotics factor graphs (Bayes-tree
// incremental updates, mature robust kernels, the iSAM2 path), which is why it is the planned
// global backend. This file proves the SAME pose graph (poses + relative BetweenFactors + loop
// closures + cross-session PriorFactors + per-component gauge) optimizes IDENTICALLY under GTSAM as
// under Ceres — validated A/B in test_pose_graph_gtsam — so the swap is behaviour-neutral and the
// next step (iSAM2) is incremental-only, not a re-derivation.
//
// THE load-bearing detail: slamko's 6×6 information is in [trans(3); rot(3)] order; GTSAM's Pose3
// tangent (and hence its noise models / Logmap) is [rot(3); trans(3)]. We PERMUTE the information
// block-wise (P = [[0,I],[I,0]]) so the whitened cost r·I·r is identical — the cost is a scalar and
// permutation-invariant, so both solvers minimise the same objective and reach the same minimum.
//
// Built only with -DSLAMKO_LOOP_WITH_GTSAM=ON (keeps the default build Ceres-only + GTSAM-free).
// Without the macro this file is a working stub: optimizeGtsam_() warns and falls back to Ceres.

#include "slamko_loop/pose_graph.hpp"

#ifdef SLAMKO_HAVE_GTSAM

#include <cstdio>
#include <unordered_map>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <set>

namespace slamko {
namespace {

gtsam::Pose3 toGtsam(const SE3& T) {
  return gtsam::Pose3(gtsam::Rot3(T.so3().unit_quaternion()), T.translation());
}

// slamko sqrt_info [trans;rot] -> a GTSAM noise model in [rot;trans] order. info = sqrt_infoᵀ·sqrt_info,
// then block-swap so it matches GTSAM's tangent convention; the resulting cost is identical to Ceres.
gtsam::SharedNoiseModel noiseFromSqrtInfo(const Eigen::Matrix<double, 6, 6>& sqrt_info,
                                          bool robust, double huber_delta) {
  Eigen::Matrix<double, 6, 6> info = sqrt_info.transpose() * sqrt_info;
  Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
  P.block<3, 3>(0, 3).setIdentity();  // [rot;trans] <- [trans;rot]
  P.block<3, 3>(3, 0).setIdentity();
  const Eigen::Matrix<double, 6, 6> info_g = P * info * P.transpose();
  gtsam::SharedNoiseModel base = gtsam::noiseModel::Gaussian::Information(info_g);
  if (robust && huber_delta > 0.0)
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(huber_delta), base);
  return base;
}

}  // namespace

PoseGraph::Result PoseGraph::optimizeGtsam_() {
  // SAFETY: the GTSAM backend doesn't yet have a yaw-only factor → rather than silently DROP an
  // active compass constraint, fall back to Ceres whenever yaw priors are present. So GTSAM is safe
  // as the default: a compass-active run transparently uses Ceres, everything else uses GTSAM.
  if (!yaw_priors_.empty()) return optimizeCeres_();
  Result res;
  res.num_nodes = static_cast<int>(nodes_.size());
  for (const auto& e : edges_) (e.is_loop ? res.num_loops : res.num_odom)++;
  if (nodes_.empty() || (edges_.empty() && priors_.empty())) return res;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  for (const auto& [id, blk] : nodes_) {
    // blk = [tx ty tz qx qy qz qw] (the same packed layout Ceres optimises)
    const Eigen::Vector3d t(blk[0], blk[1], blk[2]);
    const Eigen::Quaterniond q(blk[6], blk[3], blk[4], blk[5]);  // w,x,y,z
    initial.insert(static_cast<gtsam::Key>(id), gtsam::Pose3(gtsam::Rot3(q.normalized()), t));
  }

  // Relative BetweenFactors (loop edges get the Huber kernel; odometry trusted).
  for (const auto& e : edges_) {
    if (!nodes_.count(e.from) || !nodes_.count(e.to)) continue;
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        static_cast<gtsam::Key>(e.from), static_cast<gtsam::Key>(e.to), toGtsam(e.meas),
        noiseFromSqrtInfo(e.sqrt_info, e.is_loop, cfg_.loop_huber_delta));
  }
  // Unary absolute-pose priors (cross-session / GPS) — robust by default.
  for (const auto& pr : priors_) {
    if (!nodes_.count(pr.id)) continue;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        static_cast<gtsam::Key>(pr.id), toGtsam(pr.target),
        noiseFromSqrtInfo(pr.sqrt_info, pr.robust, cfg_.loop_huber_delta));
  }
  // (yaw priors are handled by the early Ceres fall-back above — never reach here.)

  // Per-CONNECTED-COMPONENT gauge — identical policy to optimizeCeres_(): union-find the edges,
  // pin every FIXED node, and auto-pin the lowest-id node of each component WITHOUT a fixed node
  // (an honest dangling island floats at its own gauge). GTSAM has no "constant variable" in batch
  // LM, so the gauge is a VERY TIGHT prior (σ≈1e-6) — rigid to numerical precision.
  std::unordered_map<std::uint64_t, std::uint64_t> parent;
  parent.reserve(nodes_.size());
  for (const auto& kv : nodes_) parent[kv.first] = kv.first;
  std::function<std::uint64_t(std::uint64_t)> find = [&](std::uint64_t x) {
    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
    return x;
  };
  for (const auto& e : edges_) {
    if (!nodes_.count(e.from) || !nodes_.count(e.to)) continue;
    parent[find(e.from)] = find(e.to);
  }
  auto tight = gtsam::noiseModel::Isotropic::Sigma(6, 1e-6);
  std::unordered_map<std::uint64_t, bool> comp_has_fixed;
  for (std::uint64_t id : fixed_) {
    if (!nodes_.count(id)) continue;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        static_cast<gtsam::Key>(id), initial.at<gtsam::Pose3>(static_cast<gtsam::Key>(id)), tight);
    comp_has_fixed[find(id)] = true;
  }
  std::unordered_map<std::uint64_t, std::uint64_t> gauge;
  for (const auto& kv : nodes_) {
    const std::uint64_t r = find(kv.first);
    if (comp_has_fixed.count(r)) continue;
    auto g = gauge.find(r);
    if (g == gauge.end() || kv.first < g->second) gauge[r] = kv.first;
  }
  if (has_anchor_ && nodes_.count(anchor_id_) && !comp_has_fixed.count(find(anchor_id_)))
    gauge[find(anchor_id_)] = anchor_id_;
  for (const auto& kv : gauge)
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        static_cast<gtsam::Key>(kv.second),
        initial.at<gtsam::Pose3>(static_cast<gtsam::Key>(kv.second)), tight);

  gtsam::LevenbergMarquardtParams params;
  params.maxIterations = cfg_.max_iters;
  if (cfg_.verbose) params.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
  gtsam::LevenbergMarquardtOptimizer opt(graph, initial, params);
  res.initial_cost = graph.error(initial);
  const gtsam::Values result = opt.optimize();
  res.final_cost = graph.error(result);
  res.iterations = static_cast<int>(opt.iterations());
  res.converged = true;

  // Write the optimised poses back into the packed node blocks.
  for (auto& [id, blk] : nodes_) {
    const gtsam::Pose3 p = result.at<gtsam::Pose3>(static_cast<gtsam::Key>(id));
    const Eigen::Vector3d t = p.translation();
    const Eigen::Quaterniond q = p.rotation().toQuaternion();
    blk[0] = t.x(); blk[1] = t.y(); blk[2] = t.z();
    blk[3] = q.x(); blk[4] = q.y(); blk[5] = q.z(); blk[6] = q.w();
  }
  return res;
}

// ---- iSAM2 incremental backend --------------------------------------------------------------
// The actual "GTSAM is better for robotics" win: keep a persistent Bayes tree and feed it only the
// factors added SINCE the last optimize() — isam.update() relinearises just the affected sub-tree
// (O(touched), not O(graph)), so a loop closure in a million-pose lifelong map is still cheap.
namespace {
struct Isam2State {
  gtsam::ISAM2 isam;
  std::size_t edges_applied = 0;   // how many of edges_ are already in the tree
  std::size_t priors_applied = 0;  // how many of priors_ are already in the tree
  std::set<std::uint64_t> nodes_applied;
  std::set<std::uint64_t> gauged;  // nodes that already carry a gauge/fixed prior
};
}  // namespace

PoseGraph::Result PoseGraph::optimizeGtsamIsam2_() {
  if (!yaw_priors_.empty()) return optimizeCeres_();  // SAFETY: keep an active compass yaw (see LM).
  Result res;
  res.num_nodes = static_cast<int>(nodes_.size());
  for (const auto& e : edges_) (e.is_loop ? res.num_loops : res.num_odom)++;
  if (nodes_.empty()) return res;

  if (!isam2_state_) {
    gtsam::ISAM2Params p;
    p.relinearizeThreshold = 0.01;  // tighter than the 0.1 default = pose-graph-accurate
    p.relinearizeSkip = 1;
    isam2_state_ = std::make_shared<Isam2State>(Isam2State{gtsam::ISAM2(p), 0, 0, {}, {}});
  }
  auto& st = *std::static_pointer_cast<Isam2State>(isam2_state_);

  // roots = nodes that are never an edge's `to` (chain heads + Atlas-break island roots) → they
  // need a gauge prior or their component floats (singular). fixed_ nodes are also pinned.
  std::set<std::uint64_t> has_incoming;
  for (const auto& e : edges_) has_incoming.insert(e.to);

  gtsam::NonlinearFactorGraph newFactors;
  gtsam::Values newValues;
  auto tight = gtsam::noiseModel::Isotropic::Sigma(6, 1e-4);

  for (const auto& [id, blk] : nodes_) {
    if (st.nodes_applied.count(id)) continue;
    const Eigen::Vector3d t(blk[0], blk[1], blk[2]);
    const Eigen::Quaterniond q(blk[6], blk[3], blk[4], blk[5]);
    const gtsam::Pose3 P(gtsam::Rot3(q.normalized()), t);
    newValues.insert(static_cast<gtsam::Key>(id), P);
    st.nodes_applied.insert(id);
    const bool is_root = !has_incoming.count(id) || fixed_.count(id) ||
                         (has_anchor_ && id == anchor_id_);
    if (is_root && !st.gauged.count(id)) {
      newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(static_cast<gtsam::Key>(id), P,
                                                                  tight);
      st.gauged.insert(id);
    }
  }
  for (std::size_t i = st.priors_applied; i < priors_.size(); ++i) {
    const auto& pr = priors_[i];
    if (!st.nodes_applied.count(pr.id)) continue;
    newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        static_cast<gtsam::Key>(pr.id), toGtsam(pr.target),
        noiseFromSqrtInfo(pr.sqrt_info, pr.robust, cfg_.loop_huber_delta));
  }
  st.priors_applied = priors_.size();
  for (std::size_t i = st.edges_applied; i < edges_.size(); ++i) {
    const auto& e = edges_[i];
    if (!st.nodes_applied.count(e.from) || !st.nodes_applied.count(e.to)) continue;
    newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        static_cast<gtsam::Key>(e.from), static_cast<gtsam::Key>(e.to), toGtsam(e.meas),
        noiseFromSqrtInfo(e.sqrt_info, e.is_loop, cfg_.loop_huber_delta));
  }
  st.edges_applied = edges_.size();

  try {
    st.isam.update(newFactors, newValues);
    const gtsam::Values est = st.isam.calculateEstimate();
    for (auto& [id, blk] : nodes_) {
      if (!est.exists(static_cast<gtsam::Key>(id))) continue;
      const gtsam::Pose3 P = est.at<gtsam::Pose3>(static_cast<gtsam::Key>(id));
      const Eigen::Vector3d t = P.translation();
      const Eigen::Quaterniond q = P.rotation().toQuaternion();
      blk[0] = t.x(); blk[1] = t.y(); blk[2] = t.z();
      blk[3] = q.x(); blk[4] = q.y(); blk[5] = q.z(); blk[6] = q.w();
    }
    res.converged = true;
  } catch (const std::exception& ex) {
    // GLIM disposable-graph principle: a corrupted incremental update → discard the tree and rebuild
    // batch (LM) for this call; next call re-inits the iSAM2 from scratch.
    std::fprintf(stderr, "[pose_graph_isam2] update threw (%s) — rebuilding batch (LM).\n", ex.what());
    isam2_state_.reset();
    return optimizeGtsam_();
  }
  return res;
}

}  // namespace slamko

#else  // SLAMKO_HAVE_GTSAM not defined — stub that falls back to Ceres.

#include <cstdio>

namespace slamko {
PoseGraph::Result PoseGraph::optimizeGtsam_() {
  static bool warned = false;
  if (!warned) {
    std::fprintf(stderr,
                 "[pose_graph] GTSAM backend requested but slamko_loop was built WITHOUT GTSAM "
                 "(-DSLAMKO_LOOP_WITH_GTSAM=ON) — falling back to Ceres.\n");
    warned = true;
  }
  return optimizeCeres_();
}
PoseGraph::Result PoseGraph::optimizeGtsamIsam2_() { return optimizeGtsam_(); }
}  // namespace slamko

#endif  // SLAMKO_HAVE_GTSAM
