// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// OdometryProvider contract (P-A, MASTER_PLAN §2) — the seam that makes slamko a
// LOOSE-fusion layer over an EXTERNAL odometry provider (OKVIS2-X default;
// klt_vo/Basalt/cuVSLAM later). slamko consumes the provider's pose stream and
// turns it into RELATIVE keyframe-to-keyframe edges + covariance for the thin
// global pose-graph — never the provider's re-corrected global pose (the
// double-loop-closure rule: slamko's relocalization is the only source of global
// constraints; run the provider with its own loop closure OFF, e.g. OKVIS config
// rsD455_odom848 with do_loop_closures=false).
//
// ProviderChain is the chain-pose-graph front half (Merfels & Stachniss, IROS
// 2016): it decimates the high-rate provider stream into keyframes (translation /
// rotation / max-dt thresholds) and emits one ProviderEdge per new keyframe.
// Degradation = covariance (Hard Rule #3): a provider that inflates its reported
// covariance (OKVIS scales 10x on Marginal, 100x on Lost tracking) flows straight
// into a weaker edge — no `if(sensor_ok)` branch anywhere.
//
// ROS-agnostic on purpose: the ROS node (slamko_ros/provider_fusion_node) is a
// thin shell around this, so the chain math is unit-testable and reusable by the
// offline driver (slamko_ros/tools/provider_chain_offline).

#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>

#include <Eigen/Core>

#include "slamko_core/se3.hpp"

namespace slamko {

// One pose sample from an external odometry provider, in the provider's own
// odom frame O (drifting, gravity-aligned, never globally corrected).
struct ProviderSample {
  double t = 0.0;  // seconds
  SE3 T_OB;        // body pose in the provider odom frame
  // Pose covariance, 6x6 [trans(3); rot(3)] order (nav_msgs/Odometry layout).
  // All-zero = "provider reports no covariance" -> defaults below apply.
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
};

// A relative keyframe-to-keyframe odometry edge, ready for PoseGraph::addEdge().
struct ProviderEdge {
  std::uint64_t from = 0, to = 0;  // keyframe ids (chain-local, monotonic)
  double t_from = 0.0, t_to = 0.0;
  SE3 T_from_to;  // body_from -> body_to, measured in the provider odom frame
  Eigen::Matrix<double, 6, 6> information =
      Eigen::Matrix<double, 6, 6>::Identity();  // [trans; rot]
};

struct ProviderChainConfig {
  // Keyframe decimation: a sample becomes a keyframe when it moved/rotated this
  // much from the last keyframe, or when this much time passed (the max_dt arm
  // keeps the chain alive while stationary so a later loop edge has nodes to
  // pull on, and bounds per-edge integration time).
  double kf_min_translation = 0.10;  // m
  double kf_min_rotation    = 0.10;  // rad (~5.7 deg)
  double kf_max_dt          = 1.0;   // s; <=0 disables the time arm

  // Edge covariance = cov_from + cov_to (independence approximation — slightly
  // conservative, which is the safe direction for a provider whose covariance is
  // itself heuristic, like OKVIS's quality-scaled diagonal). Only the diagonal is
  // used. Floored below; when the provider reports all-zero covariance the
  // defaults are used instead (PoseGraph's trusted-odometry sigmas).
  double default_sigma_t = 0.05;  // m
  double default_sigma_r = 0.02;  // rad
  double floor_sigma_t   = 0.005; // m
  double floor_sigma_r   = 0.002; // rad
};

class ProviderChain {
 public:
  explicit ProviderChain(ProviderChainConfig cfg = {}) : cfg_(cfg) {}

  // Feed one provider sample (any rate, monotonic t). Returns an edge exactly
  // when `s` was promoted to a new keyframe; the first sample seeds keyframe 0
  // and returns nothing.
  std::optional<ProviderEdge> feed(const ProviderSample& s) {
    if (!has_kf_) {
      last_kf_ = s;
      has_kf_ = true;
      next_id_ = 1;
      return std::nullopt;
    }
    const SE3 T_rel = last_kf_.T_OB.inverse() * s.T_OB;
    const double dt = s.t - last_kf_.t;
    const double trans = T_rel.translation().norm();
    const double rot = T_rel.so3().log().norm();
    const bool time_kf = cfg_.kf_max_dt > 0.0 && dt >= cfg_.kf_max_dt;
    if (trans < cfg_.kf_min_translation && rot < cfg_.kf_min_rotation && !time_kf)
      return std::nullopt;

    ProviderEdge e;
    e.from = next_id_ - 1;
    e.to = next_id_;
    e.t_from = last_kf_.t;
    e.t_to = s.t;
    e.T_from_to = T_rel;
    e.information = edgeInformation(last_kf_, s);
    last_kf_ = s;
    ++next_id_;
    return e;
  }

  bool hasKeyframe() const { return has_kf_; }
  std::uint64_t lastId() const { return has_kf_ ? next_id_ - 1 : 0; }
  const ProviderSample& lastKeyframe() const { return last_kf_; }
  std::uint64_t numKeyframes() const { return has_kf_ ? next_id_ : 0; }

 private:
  Eigen::Matrix<double, 6, 6> edgeInformation(const ProviderSample& a,
                                              const ProviderSample& b) const {
    Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
    const Eigen::Matrix<double, 6, 1> var_sum =
        (a.cov.diagonal() + b.cov.diagonal()).cwiseMax(0.0);
    const bool has_cov = var_sum.sum() > 1e-15;
    for (int i = 0; i < 6; ++i) {
      const double def = (i < 3) ? cfg_.default_sigma_t : cfg_.default_sigma_r;
      const double flo = (i < 3) ? cfg_.floor_sigma_t : cfg_.floor_sigma_r;
      const double var = has_cov ? std::max(var_sum[i], flo * flo) : def * def;
      info(i, i) = 1.0 / var;
    }
    return info;
  }

  ProviderChainConfig cfg_;
  ProviderSample last_kf_;
  std::uint64_t next_id_ = 0;
  bool has_kf_ = false;
};

}  // namespace slamko
