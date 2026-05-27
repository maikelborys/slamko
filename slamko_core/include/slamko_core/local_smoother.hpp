// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// LocalSmoother — the Tier-2 fusion contract at VI-keyframe granularity (the
// practical swap surface the VIO uses). Implementations: GtsamLocalSmoother
// (slamko_fusion, IncrementalFixedLagSmoother + marginalization) and
// CeresLocalSmoother (slamko_vio, wraps klt_vo's LocalBA). The generic
// FactorGraphBackend (factor_graph_backend.hpp) remains the low-level extension
// seam for arbitrary/custom factors; LocalSmoother is the VI-core fast path.
//
// Convention: poses are T_WB (body-in-world, MASTER_PLAN §8.3). Each backend owns
// its IMU preintegration — the contract passes RAW samples since the previous KF.

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "slamko_core/health.hpp"
#include "slamko_core/imu_sample.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_core/stereo_observation.hpp"

namespace slamko {

class LocalSmoother {
 public:
  virtual ~LocalSmoother() = default;

  // One-time setup (gravity locked from the VI init; rectified-stereo calib;
  // camera-in-body extrinsic so the backend relates the body pose T_WB to the
  // camera observations — GTSAM's body_P_sensor / klt_vo's T_BS).
  virtual void setImuParams(const ImuParams& params) = 0;
  virtual void setStereoCalib(const StereoCalib& calib) = 0;
  virtual void setExtrinsics(const SE3& body_T_cam) = 0;

  // Insert a keyframe at time t: initial body nav state + the raw IMU samples
  // spanning (previous KF, t] + the stereo observations this KF. Landmarks are
  // created lazily from StereoObservation::world_init on first sight.
  virtual void insertKeyframe(double t,
                              const SE3& T_WB_init,
                              const Eigen::Vector3d& velocity_init,
                              const ImuBias& bias_init,
                              const std::vector<ImuSample>& imu_since_prev,
                              const std::vector<StereoObservation>& observations) = 0;

  // (Incrementally) optimize the fixed-lag window. Returns false on numerical
  // failure — the caller decides recovery; the backend must not crash.
  virtual bool optimize() = 0;

  // Refined latest-keyframe estimates (after optimize()).
  virtual SE3             latestPose() const = 0;      // T_WB
  virtual Eigen::Vector3d latestVelocity() const = 0;  // world frame
  virtual ImuBias         latestBias() const = 0;
  // Refined world position of a landmark still in the window; false otherwise.
  virtual bool landmark(std::uint64_t id, Eigen::Vector3d& out) const = 0;

  // Observability / loss probes for the never-lost supervisor (P2).
  virtual HealthSignal health() const = 0;
};

}  // namespace slamko
