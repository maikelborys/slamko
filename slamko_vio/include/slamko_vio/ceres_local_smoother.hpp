// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// CeresLocalSmoother — the ceres-backend implementation of slamko_core::
// LocalSmoother. It is a thin adapter that wraps slamko_vio's validated
// LocalBA (sliding-window Ceres BA + IMU factor chain) behind the abstract
// Tier-2 contract, so VioPipeline can be routed through LocalSmoother and the
// backend swapped to GtsamLocalSmoother (slamko_fusion) at the composition root
// without the pipeline ever naming a concrete backend.
//
// Two boundaries are bridged here (and ONLY here):
//   1. Pose frame. The contract speaks T_WB (body-in-world); LocalBA speaks
//      T_w_c (world→camera: p_cam = R·p_world + t). With E = body_T_cam
//      (= LocalBA's T_BS, cam→body), the pipeline's body pose is the world→body
//      map T_b_w = E·T_w_c (authoritative at vio_pipeline.cpp gravity-calib),
//      so T_WB = (E·T_w_c)⁻¹ and inversely T_w_c = E⁻¹·T_WB⁻¹ = (T_WB·E)⁻¹.
//   2. IMU. The contract passes RAW samples + leaves preintegration to the
//      backend; this adapter preintegrates with klt_vo's ImuPreintegration
//      (the exact call VioPipeline used to make), so the ceres path is
//      numerically the P0 baseline (up to the SE3 quaternion round-trip).
//
// The contract has no reset(); LocalBA is instead rebuilt in-place by the
// config setters (setExtrinsics / setImuParams), reproducing VioPipeline's two
// mid-stream LocalBA rebuilds (on T_BS resolve and on gravity calibration). A
// rebuild clears the window — intended parity (the baseline discards the
// visual-only KFs at those moments).

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "slamko_core/local_smoother.hpp"

#include "slamko_vio/imu_types.hpp"
#include "slamko_vio/local_ba.hpp"
#include "slamko_vio/types.hpp"

namespace slamko_vio {

// Everything the adapter needs to (re)build the wrapped LocalBA + run its own
// preintegration. Mirrors the two structs VioPipeline already maintains, so the
// pipeline ctor just hands over its ba_cfg_ + imu_noise_ verbatim — no value
// can drift between the baseline and the routed path. The contract's ImuParams
// can't carry the LocalBA tuning or the IMU rate, which is why this exists.
struct CeresLocalSmootherConfig {
  LocalBA::Config ba;      // window / huber / inv-depth / enable_imu / T_BS / gravity / bias_rw
  ImuNoise        noise;   // densities + random-walk + rate_hz for preintegration
};

class CeresLocalSmoother : public slamko::LocalSmoother {
 public:
  explicit CeresLocalSmoother(const CeresLocalSmootherConfig& cfg = {});
  ~CeresLocalSmoother() override;

  // Config-phase setters. setExtrinsics / setImuParams rebuild the wrapped
  // LocalBA from the full cached config (eager — so the next insertKeyframe
  // never hits a stale window). setStereoCalib just caches K (LocalBA takes K
  // per-insert, no rebuild needed).
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
  slamko::SE3      latestPose() const override;       // T_WB
  Eigen::Vector3d  latestVelocity() const override;   // world frame
  slamko::ImuBias  latestBias() const override;
  bool landmark(std::uint64_t id, Eigen::Vector3d& out) const override;
  slamko::HealthSignal health() const override;

 private:
  void rebuild();  // local_ba_ = make_unique<LocalBA>(ba_cfg_)

  LocalBA::Config            ba_cfg_;        // full, persistent — every rebuild copies it whole
  ImuNoise                   noise_;         // preintegration noise (rate_hz preserved across setImuParams)
  StereoIntrinsics           K_{};           // from setStereoCalib
  Eigen::Matrix4d            E_     = Eigen::Matrix4d::Identity();  // body_T_cam (= LocalBA T_BS)
  Eigen::Matrix4d            E_inv_ = Eigen::Matrix4d::Identity();  // cam_T_body
  std::unique_ptr<LocalBA>   ba_;
};

}  // namespace slamko_vio
