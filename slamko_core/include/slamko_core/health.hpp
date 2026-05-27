// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Health — designed in from day 1, not retrofitted. The *signals* live here
// (interfaces + probe struct emitted by vio/fusion); the *policy* (the Good/
// Marginal/Lost state machine, watchdogs, recovery triggers) is the never-lost
// supervisor in slamko_loop (P2). Keeping them apart is what lets the supervisor
// gate recovery OUTSIDE the estimator graph.
//
// Note (MASTER_PLAN §7): loss detection is the ODOMETRY STALE-GAP, not a
// covariance spike — a blackout pauses odom, it does not inflate covariance
// (the user's OKVIS2-X finding). odom_stale_gap_s is therefore the load-bearing
// loss signal; the covariance/eigenvalue fields are degeneracy/observability
// monitors, not the primary loss trigger.

#pragma once

namespace slamko {

enum class HealthState { Good, Marginal, Lost };

// A snapshot of estimator health, emitted by vio/fusion and consumed by the
// supervisor. All fields optional in spirit — a producer fills what it has.
struct HealthSignal {
  // Wall-clock / monotonic seconds since the odometry stream last advanced.
  // THE loss trigger (sim-time-proof). 0 while healthy.
  double odom_stale_gap_s = 0.0;

  // Smallest eigenvalue of the local information matrix — degeneracy monitor
  // (near zero ⇒ an unobservable direction). <0 = not reported.
  double min_information_eigenvalue = -1.0;

  // Trace of the latest pose marginal covariance — uncertainty growth monitor.
  double pose_marginal_cov_trace = -1.0;

  // Live tracking quality (e.g. inlier ratio of the last PnP). [0,1], <0 = n/a.
  double tracking_inlier_ratio = -1.0;
};

// Interface for anything that can report its health (vio frontend, fusion).
class HealthReporter {
 public:
  virtual ~HealthReporter() = default;
  virtual HealthSignal health() const = 0;
};

}  // namespace slamko
