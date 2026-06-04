// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// PoseGraph — the thin, DISPOSABLE global pose-graph (MASTER_PLAN §4: "the global
// graph is disposable; the fast odometry never depends on it"). This is the
// "loop closure + optimization, but separate" layer: it consumes the VIO's
// keyframe trajectory as a chain of relative-pose ODOMETRY edges, accepts
// LOOP-closure edges (from the Relocalizer's weld), and re-optimizes the absolute
// keyframe poses so the accumulated drift is redistributed and the loop closes.
//
// It is intentionally a POSE-graph, not a bundle adjustment: nodes are SE(3) body
// poses T_W_body, edges are relative-pose measurements with a 6×6 information
// matrix ([trans; rot] order). The heavy joint VI-BA lives behind the
// GlobalSmoother contract (slamko_fusion/GtsamGlobalSmoother); this thin graph is
// the cheap, always-available corrector — the "anchors-only pose-graph" that
// already gave ~33% ATE improvement, rebuilt clean.
//
// Loop edges carry a robust kernel (Huber on the whitened residual) so a single
// bad weld can't tear the map; odometry edges are trusted (no kernel). The solver
// (Ceres) is fully encapsulated in the .cpp — this header pulls only Eigen +
// slamko_core::SE3, so consumers stay solver-agnostic (Hard Rule #2).

#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "slamko_core/se3.hpp"

namespace slamko {

struct PoseGraphConfig {
  // Robust kernel half-width for LOOP edges, in units of the whitened residual
  // (≈ sigmas). <=0 disables the kernel (treat loops as trusted). Odometry edges
  // are never robustified. Default ≈ sqrt(chi²_0.95, 1dof)·~2 — tolerant of a
  // moderately wrong weld without ignoring good ones. (GNC-TLS is the upgrade.)
  double loop_huber_delta = 2.45;
  int    max_iters        = 50;
  bool   verbose          = false;
};

class PoseGraph {
 public:
  struct Result {
    bool   converged    = false;
    double initial_cost = 0.0;
    double final_cost   = 0.0;
    int    iterations   = 0;
    int    num_nodes    = 0;
    int    num_odom     = 0;
    int    num_loops    = 0;
  };

  explicit PoseGraph(PoseGraphConfig cfg = {}) : cfg_(cfg) {}

  // Insert / overwrite a keyframe's absolute pose estimate (T_W_body).
  void addKeyframe(std::uint64_t id, const SE3& T_W_body);

  // Relative-pose edge: measurement T_from_to (body_from → body_to). `information`
  // is the 6×6 weight in [trans(3); rot(3)] order. `is_loop` toggles the robust
  // kernel + counts toward num_loops.
  void addEdge(std::uint64_t from, std::uint64_t to, const SE3& T_from_to,
               const Eigen::Matrix<double, 6, 6>& information, bool is_loop);

  // Convenience overloads: diagonal information from isotropic sigmas
  // (sigma_t in metres, sigma_r in radians).
  void addOdometryEdge(std::uint64_t from, std::uint64_t to, const SE3& T_from_to,
                       double sigma_t = 0.05, double sigma_r = 0.02);
  void addLoopEdge(std::uint64_t from, std::uint64_t to, const SE3& T_from_to,
                   double sigma_t = 0.10, double sigma_r = 0.05);

  // Gauge fix: this node's pose is held constant. If never set, the smallest id
  // is anchored automatically.
  void setAnchor(std::uint64_t id) { anchor_id_ = id; has_anchor_ = true; }

  // Solve. Returns stats; no-op (converged=false) if < 2 nodes or no edges.
  Result optimize();

  SE3 pose(std::uint64_t id) const;                         // throws if absent
  bool hasNode(std::uint64_t id) const { return nodes_.count(id) > 0; }
  std::vector<std::pair<std::uint64_t, SE3>> poses() const;  // sorted by id
  std::size_t numNodes() const { return nodes_.size(); }
  std::size_t numEdges() const { return edges_.size(); }

 private:
  struct Edge {
    std::uint64_t from = 0, to = 0;
    SE3 meas;
    Eigen::Matrix<double, 6, 6> sqrt_info = Eigen::Matrix<double, 6, 6>::Identity();
    bool is_loop = false;
  };

  static Eigen::Matrix<double, 6, 6> sqrtInfoFromSigmas(double sigma_t, double sigma_r);

  PoseGraphConfig cfg_;
  // pose storage: [tx, ty, tz, qx, qy, qz, qw] (Eigen quaternion coeff order).
  std::map<std::uint64_t, std::array<double, 7>> nodes_;
  std::vector<Edge> edges_;
  std::uint64_t anchor_id_ = 0;
  bool has_anchor_ = false;
};

}  // namespace slamko
