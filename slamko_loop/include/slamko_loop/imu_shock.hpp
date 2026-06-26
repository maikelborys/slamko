// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// ImuShockDetector — the KIDNAP / HARD-KNOCK / DROP trigger slamko lacked (P0.3). WHY:
// the quality-break gate fires on KINEMATIC incoherence of the PROVIDER pose (a speed-jump);
// but a clean lift / bump / drone-flip produces NO pose speed-jump — the provider (OKVIS,
// cuVSLAM) COASTS on IMU through the disturbance, so its output looks smooth-but-wrong. The
// disturbance IS visible in the RAW IMU as a jerk/accel/gyro spike. This detector reads the
// raw IMU directly (the canonical kidnap trigger the literature names) and flags the event so
// the never-lost supervisor can SEAL + BREAK (declare the interval unobservable, dangle
// honestly, re-anchor on the next recognized revisit) — never trust a pose dead-reckoned
// through a knock. Complements quality-break; neither alone covers both failure shapes.
//
// Three signatures: IMPACT (jerk spike + high |accel| — a bump/collision), FREEFALL (|accel|
// near 0 — a drop / drone-flip / lift off a surface), YANK (gyro-rate spike — a fast snatch/
// flip). Eigen-only (Hard Rule #2). Stateful (keeps the previous accel for the jerk); a short
// refractory avoids re-flagging the same event every sample of a multi-sample spike.

#pragma once

#include <cmath>

#include <Eigen/Core>

namespace slamko {

struct ImuShockConfig {
  double jerk_thresh = 200.0;     // m/s^3 — |d accel|/dt that flags an IMPACT (a bump/hit)
  double accel_thresh = 35.0;     // m/s^2 — |accel| above this with the jerk = a real impact
  double gyro_thresh = 9.0;       // rad/s — angular-rate spike = a YANK / flip (~515 deg/s)
  double freefall_thresh = 2.5;   // m/s^2 — |accel| below this = FREEFALL (drop/flip/lift)
  double gravity = 9.81;          // for reference; detector is gravity-agnostic (uses |accel|)
  double refractory_s = 0.3;      // suppress re-triggering within this window of a flagged event
};

enum class ShockKind { None, Impact, Freefall, Yank };

struct ShockEvent {
  ShockKind kind = ShockKind::None;
  double jerk = 0.0;       // m/s^3
  double accel_mag = 0.0;  // m/s^2
  double gyro_mag = 0.0;   // rad/s
  bool detected() const { return kind != ShockKind::None; }
  const char* name() const {
    switch (kind) {
      case ShockKind::Impact: return "IMPACT";
      case ShockKind::Freefall: return "FREEFALL";
      case ShockKind::Yank: return "YANK";
      default: return "none";
    }
  }
};

class ImuShockDetector {
 public:
  explicit ImuShockDetector(ImuShockConfig cfg = {}) : cfg_(cfg) {}

  // Feed one IMU sample (accel [m/s^2] incl. gravity, gyro [rad/s], dt [s] since the last).
  // Returns the detected event (kind=None when calm or inside the refractory window).
  ShockEvent feed(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt,
                  double t = 0.0) {
    ShockEvent ev;
    ev.accel_mag = accel.norm();
    ev.gyro_mag = gyro.norm();
    if (have_prev_ && dt > 1e-4) ev.jerk = (accel - prev_accel_).norm() / dt;
    prev_accel_ = accel;
    have_prev_ = true;

    // Classify (gyro yank first — a flip is the most dangerous; then freefall; then impact).
    ShockKind kind = ShockKind::None;
    if (ev.gyro_mag > cfg_.gyro_thresh)
      kind = ShockKind::Yank;
    else if (ev.accel_mag < cfg_.freefall_thresh)
      kind = ShockKind::Freefall;
    else if (ev.jerk > cfg_.jerk_thresh && ev.accel_mag > cfg_.accel_thresh)
      kind = ShockKind::Impact;

    if (kind == ShockKind::None) return ev;            // calm
    if (t > 0.0 && (t - last_event_t_) < cfg_.refractory_s && last_event_t_ > 0.0)
      return ShockEvent{};                              // inside refractory → suppress
    last_event_t_ = t;
    ev.kind = kind;
    return ev;
  }

  void reset() { have_prev_ = false; last_event_t_ = 0.0; }

 private:
  ImuShockConfig cfg_;
  Eigen::Vector3d prev_accel_ = Eigen::Vector3d::Zero();
  bool have_prev_ = false;
  double last_event_t_ = 0.0;
};

}  // namespace slamko
