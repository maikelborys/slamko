// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// GtsamLocalSmoother — the Tier-2 fusion backend: a GTSAM IncrementalFixedLagSmoother
// (iSAM2 + timestamp marginalization) over T_WB / velocity / bias / landmark nodes,
// fusing CombinedImuFactor (IMU preintegration + bias random walk) and
// GenericStereoFactor (metric stereo reprojection). Marginalization (Schur + FEJ)
// replaces klt_vo's lossy gauge-by-constant — the VILENS heart.
//
// GTSAM is fully hidden behind a PIMPL so consumers (slamko_vio / slamko_ros) link
// this without pulling GTSAM headers. Implements slamko_core::LocalSmoother.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "slamko_core/local_smoother.hpp"

namespace slamko_fusion {

struct GtsamSmootherConfig {
  double lag                  = 2.0;    // fixed-lag window (seconds) → marginalize older
  double pixel_sigma          = 1.0;    // stereo reprojection noise (px)
  double prior_pose_sigma     = 0.1;    // first-KF gauge anchor (m / rad)
  double prior_vel_sigma      = 0.1;    // m/s
  double prior_bias_sigma     = 1.0e-2;
  // Weak prior on each landmark at first sight (m). Regularizes single-
  // observation / low-parallax points so the Schur marginalization stays
  // non-singular (LocalBA instead prunes <2-obs landmarks). Soft enough that
  // multi-view stereo factors dominate — negligible bias on tracked landmarks.
  double landmark_prior_sigma  = 10.0;
  // Landmark management — the batch solve cost is dominated by landmark-variable
  // count. Add a landmark only on its `min_landmark_obs`-th sighting (drops the
  // ~half that are single-view births) and cap the live set (real-time VIO keeps
  // ~100-150 features). Keeps the per-KF solve bounded; <=0 disables a knob.
  int    min_landmark_obs      = 2;
  int    max_landmarks         = 150;
  double relinearize_threshold = 0.1;
  int    relinearize_skip      = 1;
  bool   use_imu              = true;   // false = visual-only (synthetic tests)
};

class GtsamLocalSmoother : public slamko::LocalSmoother {
 public:
  explicit GtsamLocalSmoother(const GtsamSmootherConfig& cfg = {});
  ~GtsamLocalSmoother() override;

  void setImuParams(const slamko::ImuParams& params) override;
  void setStereoCalib(const slamko::StereoCalib& calib) override;
  void setExtrinsics(const slamko::SE3& body_T_cam) override;

  void insertKeyframe(double t,
                      const slamko::SE3& T_WB_init,
                      const Eigen::Vector3d& velocity_init,
                      const slamko::ImuBias& bias_init,
                      const std::vector<slamko::ImuSample>& imu_since_prev,
                      const std::vector<slamko::StereoObservation>& observations) override;

  bool optimize() override;
  slamko::SE3      latestPose() const override;
  Eigen::Vector3d  latestVelocity() const override;
  slamko::ImuBias  latestBias() const override;
  bool landmark(std::uint64_t id, Eigen::Vector3d& out) const override;
  slamko::HealthSignal health() const override;

  // Debug introspection (for the bounded-window test): variables held by the
  // smoother after the last update. Marginalization keeps this bounded.
  std::size_t numVariables() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace slamko_fusion
