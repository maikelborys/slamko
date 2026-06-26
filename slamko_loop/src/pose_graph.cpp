// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// PoseGraph implementation. Ceres is confined to this translation unit; the
// public header stays solver-free (Hard Rule #2). The between-factor is the
// standard SE(3) relative-pose residual (cf. Ceres pose_graph_3d example):
//   t_ab_est = q_a^{-1} (t_b - t_a),   q_ab_est = q_a^{-1} q_b
//   r_trans  = t_ab_est - t_meas
//   r_rot    = 2 · (q_meas^{-1} q_ab_est).vec()
//   residual = sqrt_info · [r_trans; r_rot]
// whitened by the per-edge sqrt information (so the Huber half-width on loop
// edges is in sigma units).

#include "slamko_loop/pose_graph.hpp"

#include <stdexcept>
#include <unordered_map>

#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/manifold.h>

namespace slamko {
namespace {

// pose block layout: p[0..2] = translation, q[0..3] = quaternion (x,y,z,w).
struct BetweenFactor {
  BetweenFactor(const SE3& meas, const Eigen::Matrix<double, 6, 6>& sqrt_info)
      : q_meas_(meas.so3().unit_quaternion()),
        t_meas_(meas.translation()),
        sqrt_info_(sqrt_info) {}

  template <typename T>
  bool operator()(const T* const p_a, const T* const q_a_ptr,
                  const T* const p_b, const T* const q_b_ptr,
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_a(p_a);
    Eigen::Map<const Eigen::Quaternion<T>>   q_a(q_a_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_b(p_b);
    Eigen::Map<const Eigen::Quaternion<T>>   q_b(q_b_ptr);

    const Eigen::Quaternion<T> q_a_inv = q_a.conjugate();
    const Eigen::Matrix<T, 3, 1> t_ab_est = q_a_inv * (t_b - t_a);
    const Eigen::Quaternion<T> q_ab_est = q_a_inv * q_b;
    const Eigen::Quaternion<T> dq = q_meas_.template cast<T>().conjugate() * q_ab_est;

    Eigen::Matrix<T, 6, 1> raw;
    raw.template head<3>() = t_ab_est - t_meas_.template cast<T>();
    raw.template tail<3>() = T(2.0) * dq.vec();
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    residuals = sqrt_info_.template cast<T>() * raw;
    return true;
  }

  Eigen::Quaterniond q_meas_;
  Eigen::Vector3d t_meas_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};

// Unary absolute-pose prior: residual = sqrt_info · [t - t_target; 2·(q_target⁻¹ q).vec()].
// Used by cross-session re-anchoring to pull a keyframe toward the pose a prior-map
// match implies, letting the optimizer bend (not rigidly re-base) the session.
struct PriorFactor {
  PriorFactor(const SE3& target, const Eigen::Matrix<double, 6, 6>& sqrt_info)
      : q_t_(target.so3().unit_quaternion()),
        t_t_(target.translation()),
        sqrt_info_(sqrt_info) {}

  template <typename T>
  bool operator()(const T* const p, const T* const q_ptr, T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(p);
    Eigen::Map<const Eigen::Quaternion<T>>   q(q_ptr);
    const Eigen::Quaternion<T> dq = q_t_.template cast<T>().conjugate() * q;
    Eigen::Matrix<T, 6, 1> raw;
    raw.template head<3>() = t - t_t_.template cast<T>();
    raw.template tail<3>() = T(2.0) * dq.vec();
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    residuals = sqrt_info_.template cast<T>() * raw;
    return true;
  }

  Eigen::Quaterniond q_t_;
  Eigen::Vector3d t_t_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};

// Unary YAW-only prior: residual = sqrt_info · wrap(yaw(q) − yaw_target). yaw is the
// rotation about world-Z, yaw = atan2(R10, R00) (pitch-safe except at ±90° pitch). Only
// the quaternion block participates → position + roll + pitch stay free (the compass
// constrains exactly the one unobservable DOF). See COMPASS_RTABMAP.md §2.
struct YawPriorFactor {
  YawPriorFactor(double yaw_target, double sqrt_info)
      : yaw_t_(yaw_target), si_(sqrt_info) {}

  template <typename T>
  bool operator()(const T* const q_ptr, T* residual) const {
    Eigen::Map<const Eigen::Quaternion<T>> q(q_ptr);  // (x, y, z, w)
    const Eigen::Matrix<T, 3, 3> R = q.toRotationMatrix();
    const T yaw_est = ceres::atan2(R(1, 0), R(0, 0));
    T d = yaw_est - T(yaw_t_);
    d = ceres::atan2(ceres::sin(d), ceres::cos(d));  // wrap to (−π, π]
    residual[0] = T(si_) * d;
    return true;
  }

  double yaw_t_, si_;
};

std::array<double, 7> toBlock(const SE3& T) {
  const Eigen::Quaterniond q = T.so3().unit_quaternion();
  const Eigen::Vector3d t = T.translation();
  return {t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
}

SE3 fromBlock(const std::array<double, 7>& b) {
  const Eigen::Quaterniond q(b[6], b[3], b[4], b[5]);  // (w, x, y, z)
  return SE3(SO3(q), Eigen::Vector3d(b[0], b[1], b[2]));
}

}  // namespace

Eigen::Matrix<double, 6, 6> PoseGraph::sqrtInfoFromSigmas(double sigma_t, double sigma_r) {
  Eigen::Matrix<double, 6, 6> s = Eigen::Matrix<double, 6, 6>::Zero();
  const double it = 1.0 / std::max(sigma_t, 1e-9);
  const double ir = 1.0 / std::max(sigma_r, 1e-9);
  s.diagonal() << it, it, it, ir, ir, ir;  // sqrt of a diagonal information
  return s;
}

void PoseGraph::addKeyframe(std::uint64_t id, const SE3& T_W_body) {
  nodes_[id] = toBlock(T_W_body);
}

void PoseGraph::addEdge(std::uint64_t from, std::uint64_t to, const SE3& T_from_to,
                        const Eigen::Matrix<double, 6, 6>& information, bool is_loop) {
  // sqrt_info: information = Lᵀ L  ⇒  use Lᵀ (upper) so residualᵀ·info·residual.
  Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(information);
  Eigen::Matrix<double, 6, 6> sqrt_info;
  if (llt.info() == Eigen::Success)
    sqrt_info = llt.matrixU();  // information = Lᵀ L ⇒ whiten with U = Lᵀ
  else
    sqrt_info = Eigen::Matrix<double, 6, 6>::Identity();
  edges_.push_back(Edge{from, to, T_from_to, sqrt_info, is_loop});
}

void PoseGraph::addOdometryEdge(std::uint64_t from, std::uint64_t to,
                                const SE3& T_from_to, double sigma_t, double sigma_r) {
  edges_.push_back(Edge{from, to, T_from_to, sqrtInfoFromSigmas(sigma_t, sigma_r), false});
}

void PoseGraph::addLoopEdge(std::uint64_t from, std::uint64_t to,
                            const SE3& T_from_to, double sigma_t, double sigma_r) {
  edges_.push_back(Edge{from, to, T_from_to, sqrtInfoFromSigmas(sigma_t, sigma_r), true});
}

void PoseGraph::addPriorFactor(std::uint64_t id, const SE3& T_W_body_target,
                               double sigma_t, double sigma_r, bool robust) {
  priors_.push_back(Prior{id, T_W_body_target, sqrtInfoFromSigmas(sigma_t, sigma_r), robust});
}

void PoseGraph::addYawPrior(std::uint64_t id, double yaw_target_rad, double sigma_yaw_rad,
                            bool robust) {
  yaw_priors_.push_back(
      YawPrior{id, yaw_target_rad, 1.0 / std::max(sigma_yaw_rad, 1e-6), robust});
}

PoseGraph::Result PoseGraph::optimize() {
  // Dispatch to the configured backend. GtsamLM is the step toward the MASTER_PLAN P-C′ iSAM2
  // smoother; it falls back to Ceres (with a warn) when slamko_loop was built without GTSAM.
  return cfg_.backend == PoseGraphBackend::GtsamLM ? optimizeGtsam_() : optimizeCeres_();
}

PoseGraph::Result PoseGraph::optimizeCeres_() {
  Result res;
  res.num_nodes = static_cast<int>(nodes_.size());
  for (const auto& e : edges_) (e.is_loop ? res.num_loops : res.num_odom)++;
  if (nodes_.empty() || (edges_.empty() && priors_.empty() && yaw_priors_.empty()))
    return res;

  ceres::Problem problem;
  for (auto& [id, blk] : nodes_) {
    problem.AddParameterBlock(blk.data(), 3);
    problem.AddParameterBlock(blk.data() + 3, 4, new ceres::EigenQuaternionManifold);
  }

  // Unary cross-session/GPS priors (robust by default — an aliased match must not
  // tear the graph; PCM already gated which matches reach here).
  for (const auto& pr : priors_) {
    auto it = nodes_.find(pr.id);
    if (it == nodes_.end()) continue;
    auto* cost = new ceres::AutoDiffCostFunction<PriorFactor, 6, 3, 4>(
        new PriorFactor(pr.target, pr.sqrt_info));
    ceres::LossFunction* loss =
        (pr.robust && cfg_.loop_huber_delta > 0.0) ? new ceres::HuberLoss(cfg_.loop_huber_delta)
                                                   : nullptr;
    problem.AddResidualBlock(cost, loss, it->second.data(), it->second.data() + 3);
  }

  // Unary yaw-only (compass/BNO) priors — constrain ONLY the heading of each node.
  for (const auto& yp : yaw_priors_) {
    auto it = nodes_.find(yp.id);
    if (it == nodes_.end()) continue;
    auto* cost = new ceres::AutoDiffCostFunction<YawPriorFactor, 1, 4>(
        new YawPriorFactor(yp.yaw, yp.sqrt_info));
    ceres::LossFunction* loss =
        (yp.robust && cfg_.loop_huber_delta > 0.0) ? new ceres::HuberLoss(cfg_.loop_huber_delta)
                                                   : nullptr;
    problem.AddResidualBlock(cost, loss, it->second.data() + 3);
  }

  for (const auto& e : edges_) {
    auto it_a = nodes_.find(e.from);
    auto it_b = nodes_.find(e.to);
    if (it_a == nodes_.end() || it_b == nodes_.end()) continue;
    auto* cost = new ceres::AutoDiffCostFunction<BetweenFactor, 6, 3, 4, 3, 4>(
        new BetweenFactor(e.meas, e.sqrt_info));
    ceres::LossFunction* loss =
        (e.is_loop && cfg_.loop_huber_delta > 0.0)
            ? new ceres::HuberLoss(cfg_.loop_huber_delta)
            : nullptr;
    problem.AddResidualBlock(cost, loss,
                             it_a->second.data(), it_a->second.data() + 3,
                             it_b->second.data(), it_b->second.data() + 3);
  }

  // Gauge per CONNECTED COMPONENT (Atlas disjoint-islands model): a tracking-loss
  // BREAKS the chain into a new map that floats freely until a feature match welds it
  // back. Pinning a single global anchor would leave such an island gauge-free ->
  // singular solve. So union-find the edges and pin the LOWEST-ID node of EACH connected
  // component. Effects: (1) one connected graph -> identical to before (one gauge);
  // (2) N islands -> each floats at its own gauge (honest dangling); (3) a weld that
  // joins two islands merges their component -> ONE gauge -> one bends onto the other
  // (geometric fusion comes for free). Only edges (not priors) connect components.
  std::unordered_map<std::uint64_t, std::uint64_t> parent;
  parent.reserve(nodes_.size());
  for (const auto& kv : nodes_) parent[kv.first] = kv.first;
  auto find = [&parent](std::uint64_t x) {
    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
    return x;
  };
  for (const auto& e : edges_) {
    if (!nodes_.count(e.from) || !nodes_.count(e.to)) continue;
    parent[find(e.from)] = find(e.to);
  }
  // Pin every explicitly-FIXED node (a loaded prior map's anchors): they ARE the
  // gauge for their component, and holding ALL of them constant keeps the prior
  // map rigid while the new session's cross-session BETWEEN edges bend it on.
  std::unordered_map<std::uint64_t, bool> comp_has_fixed;
  for (std::uint64_t id : fixed_) {
    auto it = nodes_.find(id);
    if (it == nodes_.end()) continue;
    problem.SetParameterBlockConstant(it->second.data());
    problem.SetParameterBlockConstant(it->second.data() + 3);
    comp_has_fixed[find(id)] = true;
  }
  // Auto-gauge the LOWEST-ID node of each component that has NO fixed node.
  std::unordered_map<std::uint64_t, std::uint64_t> gauge;  // component root -> node to pin
  for (const auto& kv : nodes_) {
    const std::uint64_t r = find(kv.first);
    if (comp_has_fixed.count(r)) continue;  // already pinned by a fixed node
    auto g = gauge.find(r);
    if (g == gauge.end() || kv.first < g->second) gauge[r] = kv.first;
  }
  // Honour an explicit anchor within its own component (keeps the world frame stable).
  if (has_anchor_ && nodes_.count(anchor_id_) && !comp_has_fixed.count(find(anchor_id_)))
    gauge[find(anchor_id_)] = anchor_id_;
  for (const auto& kv : gauge) {
    auto it = nodes_.find(kv.second);
    if (it != nodes_.end()) {
      problem.SetParameterBlockConstant(it->second.data());
      problem.SetParameterBlockConstant(it->second.data() + 3);
    }
  }

  ceres::Solver::Options opts;
  opts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  opts.max_num_iterations = cfg_.max_iters;
  opts.minimizer_progress_to_stdout = cfg_.verbose;
  ceres::Solver::Summary summary;
  ceres::Solve(opts, &problem, &summary);

  res.converged    = summary.IsSolutionUsable();
  res.initial_cost = summary.initial_cost;
  res.final_cost   = summary.final_cost;
  res.iterations   = static_cast<int>(summary.iterations.size());
  return res;
}

std::unordered_map<std::uint64_t, std::uint64_t> PoseGraph::connectedComponents() const {
  std::unordered_map<std::uint64_t, std::uint64_t> parent;
  parent.reserve(nodes_.size());
  for (const auto& kv : nodes_) parent[kv.first] = kv.first;
  auto find = [&parent](std::uint64_t x) {
    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
    return x;
  };
  for (const auto& e : edges_) {
    if (!parent.count(e.from) || !parent.count(e.to)) continue;
    parent[find(e.from)] = find(e.to);
  }
  std::unordered_map<std::uint64_t, std::uint64_t> root;
  root.reserve(nodes_.size());
  for (const auto& kv : nodes_) root[kv.first] = find(kv.first);
  return root;
}

SE3 PoseGraph::pose(std::uint64_t id) const {
  auto it = nodes_.find(id);
  if (it == nodes_.end()) throw std::out_of_range("PoseGraph::pose: unknown node id");
  return fromBlock(it->second);
}

std::vector<std::pair<std::uint64_t, SE3>> PoseGraph::poses() const {
  std::vector<std::pair<std::uint64_t, SE3>> out;
  out.reserve(nodes_.size());
  for (const auto& [id, blk] : nodes_) out.emplace_back(id, fromBlock(blk));
  return out;  // std::map iterates sorted by id
}

}  // namespace slamko
