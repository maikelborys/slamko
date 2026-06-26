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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

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
  Result res;
  res.num_nodes = static_cast<int>(nodes_.size());
  for (const auto& e : edges_) (e.is_loop ? res.num_loops : res.num_odom)++;
  if (nodes_.empty() || (edges_.empty() && priors_.empty() && yaw_priors_.empty())) return res;

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
  if (!yaw_priors_.empty())
    std::fprintf(stderr,
                 "[pose_graph_gtsam] %zu yaw prior(s) ignored (not yet ported to the GTSAM backend; "
                 "use the Ceres backend if compass yaw is active)\n",
                 yaw_priors_.size());

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

}  // namespace slamko

#else  // SLAMKO_HAVE_GTSAM not defined — stub that falls back to Ceres.

#include <cstdio>

namespace slamko {
PoseGraph::Result PoseGraph::optimizeGtsam_() {
  static bool warned = false;
  if (!warned) {
    std::fprintf(stderr,
                 "[pose_graph] GtsamLM backend requested but slamko_loop was built WITHOUT GTSAM "
                 "(-DSLAMKO_LOOP_WITH_GTSAM=ON) — falling back to Ceres.\n");
    warned = true;
  }
  return optimizeCeres_();
}
}  // namespace slamko

#endif  // SLAMKO_HAVE_GTSAM
