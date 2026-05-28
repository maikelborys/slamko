// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// SessionGraph — the OKVIS-style single-graph backbone (Phase P2 of the refactor,
// docs/PLAN_OKVIS_REFACTOR.md). Replaces the multi-submap-anchor live trajectory
// layer that was the source of the magistrale1 "trayectoria fatal" sparkles
// (corrected ATE 850 cm with anchor layer, pure VIO 641 cm WITHOUT it — anchors
// were ACTIVELY destroying 7x of precision).
//
// CONTRACT:
//   - VIO pipeline pushes KFs as they are inserted (pose, velocity, bias, KF
//     observations, IMU window since previous KF). The SessionGraph owns these
//     in a GTSAM whole-session graph; no submap partitioning, no anchor algebra.
//   - The relocalizer (front-end is unchanged: per-KF EigenPlaces VPR + LightGlue
//     verify + PnP RANSAC) pushes verified loop closures. The SessionGraph adds
//     them as BetweenFactor / un-marginalize reprojection factors and runs VI-BA
//     in a background thread.
//   - The optimizer publishes the "session correction" — the SE3 delta between
//     what VIO believes the latest KF pose is vs. what the optimized graph
//     believes. VIO (or the publisher) composes this with the live worldPose
//     SMOOTHLY (interpolated over N frames) so no retroactive jumps occur. Past
//     dumped frames stay as VIO's original output; future frames are corrected.
//
// THREADING:
//   - The insert/append API is thread-safe (single producer assumed: the VIO node).
//   - The optimizer runs in a background std::thread, joined on destruction.
//   - latestCorrection() is a snapshot — never blocks the caller.
//
// HARD RULES: slamko_loop continues to depend only on slamko_core types (GTSAM is
// hidden in the cpp via pimpl). The composition root (vio_node) does not change
// its public include surface.

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "slamko_core/imu_sample.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_core/stereo_observation.hpp"
#include "slamko_core/submap.hpp"   // KeyframeObservations

namespace slamko {

// One KF as fed into the SessionGraph. Anchored in the VIO world frame; the graph
// holds these as variables and refines them. observations + imu_since_prev are the
// substrate for the BA — the SessionGraph re-builds factors from them on insert
// (no Schur marginalization until the KF leaves the active relinearization window).
struct SessionKeyframe {
  std::uint64_t        id = 0;
  double               timestamp = 0.0;
  SE3                  T_WB;             // body pose in VIO world frame
  Eigen::Vector3d      velocity_w = Eigen::Vector3d::Zero();
  ImuBias              bias;
  KeyframeObservations obs;              // 2D obs aligned to landmark_ids
  std::vector<ImuSample> imu_since_prev; // empty for the first KF
};

// One verified loop closure: relative pose constraint between two KFs in the graph.
// kf_to is typically the just-inserted live KF; kf_from is the matched historical one.
struct SessionLoopClosure {
  std::uint64_t kf_from = 0;
  std::uint64_t kf_to   = 0;
  SE3           T_from_to;            // T_kf_from^-1 * T_kf_to (relative body pose)
  double        sigma_t = 0.05;       // m
  double        sigma_r = 0.02;       // rad
};

struct SessionGraphConfig {
  // Calibration + extrinsic (constant per session). Used to construct stereo
  // reprojection factors.
  StereoCalib calib;
  SE3         T_BS;                   // cam→body

  // Continuous-time IMU noise + gravity (slamko_core::ImuParams default = EuRoC).
  ImuParams   imu_params;

  // BA tuning.
  double pixel_sigma = 1.0;            // px
  int    max_iters   = 20;
  // Fixed-lag window — KFs older than this many in the graph get marginalized via
  // Schur; their bias/velocity variables are summarized into the marginal prior. A
  // wider window than the local smoother's (which is ~10-20 KFs) — the SessionGraph
  // can afford a longer history because optimization is async.
  int    relin_window_size = 50;
  // Trigger optimization in the worker thread every N inserted KFs (in addition to
  // immediate trigger on a loop closure). 0 = only on loop closure.
  int    optimize_every_n_kfs = 5;
};

// Lightweight pimpl pattern — GTSAM headers stay confined to session_graph.cpp.
class SessionGraphImpl;

class SessionGraph {
 public:
  explicit SessionGraph(const SessionGraphConfig& cfg);
  ~SessionGraph();
  SessionGraph(const SessionGraph&) = delete;
  SessionGraph& operator=(const SessionGraph&) = delete;

  // VIO submits a freshly inserted KF. Returns false if the KF id is a duplicate
  // (idempotent — repeated submission of the same KF is a no-op).
  bool insertKeyframe(SessionKeyframe kf);

  // The relocalizer / orchestrator submits a verified loop closure between two KFs
  // already in the graph. Returns false if either id is unknown.
  bool insertLoopClosure(SessionLoopClosure lc);

  // The SE3 correction that the SessionGraph's latest optimization implies for the
  // newest KF. SmoothPublisher composes this with the live worldPose so the
  // running trajectory converges to the BA-refined pose over a window of frames
  // (no retroactive jumps to past poses dumped to run.tum).
  // Identity when no optimization has run yet, or when the latest KF is unchanged.
  SE3 latestCorrection() const;

  // True once at least one optimization has produced a refined latest pose.
  bool haveCorrection() const { return have_correction_; }

  // Snapshot of how many KFs are currently in the graph (for diagnostics).
  std::size_t keyframeCount() const;

 private:
  std::unique_ptr<SessionGraphImpl> impl_;
  std::atomic<bool>                 have_correction_{false};
};

}  // namespace slamko
