// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// ScanContext — a VIEWPOINT-INVARIANT (yaw) geometric place descriptor, the loop
// channel that complements the appearance one (EigenPlaces/XFeat). WHY it exists:
// every slamko doc converges on the SAME root limiter — relocalization RECALL is a
// VIEWPOINT-coverage problem, not a descriptor-quality one (PIPELINE_STATUS §0).
// Appearance retrieval (EigenPlaces) matches same-heading revisits but dies on a
// different-heading return (the 0% opposite-facing ceiling) because the IMAGES don't
// overlap. GEOMETRY, however, is viewpoint-robust: the same room has the same 3D
// shape regardless of which way you face it. ScanContext (Kim & Kim, IROS 2018) turns
// a point cloud into a polar bird's-eye matrix and matches it yaw-invariantly by a
// column shift — so a revisit rotated in yaw still matches. This is the geometric half
// of the disjunctive loop gate (RESEARCH_LIFELONG_NAV_ARCH §5: "accept if visual OR
// geometric passes").
//
// HONEST scope: ScanContext was built for 360° LiDAR. A D455 depth cam is ~90° FOV, so
// the descriptor only fills the sectors it can see — yaw-invariance holds across the
// OVERLAP. It therefore extends recall to MODERATE viewpoint changes (partial overlap,
// e.g. a 90° turn); a TRUE 180°-opposite revisit has ZERO geometric overlap and is
// unmatchable by ANY method (that's physics, not a bug — the doc's honest point).
//
// Convention: points are expected in a GRAVITY-ALIGNED frame (z = up; x,y = ground).
// slamko's odom/world frame is gravity-aligned (the provider aligns gravity), so submap
// landmarks transformed to world satisfy this. Height-agnostic to absolute z (uses the
// max relative height per cell, like the original). Eigen-only (Hard Rule #2; no OpenCV).

#pragma once

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace slamko {

struct ScanContextConfig {
  int n_ring = 20;          // radial bins (range resolution)
  int n_sector = 60;        // azimuthal bins → yaw resolution = 360/n_sector = 6°
  double max_range = 8.0;   // m — D455 depth is reliable to ~6–8 m
  double min_range = 0.3;   // m — ignore too-close clutter
  // The "up" axis of the input points, in their own frame. Gravity must be the
  // invariance axis (we factor out yaw = rotation about up). Default z-up; pass the
  // body-frame gravity direction for a non-z-up frame (e.g. optical y-down → {0,-1,0}).
  Eigen::Vector3d up = Eigen::Vector3d(0, 0, 1);
};

// A ScanContext descriptor + its rotation-invariant ring key, for one place.
struct ScanContextDesc {
  Eigen::MatrixXf sc;        // n_ring × n_sector, cell = max height of points in that bin
  Eigen::VectorXf ring_key;  // n_ring, row-wise mean (yaw-INVARIANT → fast candidate search)
  bool empty() const { return sc.size() == 0; }
};

class ScanContext {
 public:
  // Build the descriptor from 3D points in a GRAVITY-ALIGNED frame (z up), centered on
  // the place origin (subtract the keyframe/submap position before calling). Empty cells
  // are 0 (no return). Returns an empty desc if no point falls in range.
  static ScanContextDesc compute(const std::vector<Eigen::Vector3d>& pts,
                                 const ScanContextConfig& cfg = {}) {
    ScanContextDesc d;
    d.sc = Eigen::MatrixXf::Zero(cfg.n_ring, cfg.n_sector);
    bool any = false;
    const double gap = (cfg.max_range - cfg.min_range);
    // Rotation that maps the input "up" axis onto +z, so the polar binning below
    // (ground = xy, height = z, yaw = azimuth) is correct for any gravity convention.
    const Eigen::Matrix3d R = upToZ(cfg.up);
    for (const auto& p0 : pts) {
      const Eigen::Vector3d p = R * p0;
      const double rng = std::sqrt(p.x() * p.x() + p.y() * p.y());
      if (rng < cfg.min_range || rng >= cfg.max_range) continue;
      // azimuth in [0,2π) → sector; range → ring
      double az = std::atan2(p.y(), p.x());
      if (az < 0) az += 2.0 * M_PI;
      int s = static_cast<int>(az / (2.0 * M_PI) * cfg.n_sector);
      int r = static_cast<int>((rng - cfg.min_range) / gap * cfg.n_ring);
      s = std::min(std::max(s, 0), cfg.n_sector - 1);
      r = std::min(std::max(r, 0), cfg.n_ring - 1);
      const float h = static_cast<float>(p.z());
      if (!any || h > d.sc(r, s)) d.sc(r, s) = h;  // max height per cell
      any = true;
    }
    if (!any) {
      d.sc.resize(0, 0);
      return d;
    }
    d.ring_key = d.sc.rowwise().mean();  // yaw-invariant
    return d;
  }

  // Ring-key distance (L2) — cheap pre-filter for candidate retrieval (yaw-invariant).
  static float ringKeyDistance(const Eigen::VectorXf& a, const Eigen::VectorXf& b) {
    if (a.size() == 0 || a.size() != b.size()) return 1e9f;
    return (a - b).norm();
  }

  // Yaw-invariant ScanContext distance: min over ALL column (yaw) shifts of the mean
  // per-column cosine distance. Returns {distance ∈ [0,1], best_shift in sectors}.
  // best_shift × (360/n_sector)° estimates the relative yaw of the revisit (a coarse
  // prior the geometric verifier can use). Lower distance = better match.
  static std::pair<float, int> distance(const Eigen::MatrixXf& a, const Eigen::MatrixXf& b) {
    if (a.size() == 0 || b.size() == 0 || a.rows() != b.rows() || a.cols() != b.cols())
      return {1.0f, 0};
    const int cols = static_cast<int>(a.cols());
    float best = 2.0f;
    int best_shift = 0;
    for (int shift = 0; shift < cols; ++shift) {
      float sum = 0.0f;
      int valid = 0;
      for (int c = 0; c < cols; ++c) {
        const Eigen::VectorXf va = a.col(c);
        const Eigen::VectorXf vb = b.col((c + shift) % cols);
        const float na = va.norm(), nb = vb.norm();
        if (na < 1e-6f || nb < 1e-6f) continue;  // skip empty columns (no overlap there)
        sum += 1.0f - va.dot(vb) / (na * nb);
        ++valid;
      }
      if (valid == 0) continue;
      const float dcol = sum / valid;
      if (dcol < best) {
        best = dcol;
        best_shift = shift;
      }
    }
    if (best > 1.5f) return {1.0f, 0};  // no column ever overlapped
    return {best, best_shift};
  }

 private:
  // Rotation mapping the unit "up" vector onto +z (shortest arc). Identity when up≈z.
  static Eigen::Matrix3d upToZ(const Eigen::Vector3d& up_in) {
    const Eigen::Vector3d z(0, 0, 1);
    Eigen::Vector3d u = up_in;
    if (u.norm() < 1e-9) return Eigen::Matrix3d::Identity();
    u.normalize();
    const double c = u.dot(z);
    if (c > 1 - 1e-9) return Eigen::Matrix3d::Identity();
    if (c < -1 + 1e-9) return Eigen::DiagonalMatrix<double, 3>(1, -1, -1);  // 180° flip
    const Eigen::Vector3d v = u.cross(z);
    Eigen::Matrix3d K;
    K << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return Eigen::Matrix3d::Identity() + K + K * K * (1.0 / (1.0 + c));
  }
};

}  // namespace slamko
