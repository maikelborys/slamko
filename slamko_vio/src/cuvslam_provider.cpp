// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// CuvslamProvider impl — the ONLY translation unit in slamko that includes
// cuvslam2.h. Links the slamko/trusted-health fork's libcuvslam.so (RPATH-pinned in
// CMake so the APT isaac_ros 4.4 lib on LD_LIBRARY_PATH can never shadow it — that
// lib has a different ABI, the C API was removed upstream in v15).

#include "slamko_vio/cuvslam_provider.hpp"

#include <cmath>
#include <cstdio>

#include <cuvslam2.h>

#include "slamko_vio/cuvslam_conversions.hpp"

namespace slamko {

struct CuvslamProvider::Impl {
  CuvslamProviderConfig cfg;
  std::unique_ptr<cuvslam::Odometry> odom;
  CuvslamHealth health;
  cuvslam::Odometry::State state;  // reused per frame (pnp_health carrier)
};

CuvslamProvider::CuvslamProvider() : impl_(new Impl) {}
CuvslamProvider::~CuvslamProvider() = default;

bool CuvslamProvider::init(const CuvslamProviderConfig& cfg) {
  impl_->cfg = cfg;
  if (cfg.width <= 0 || cfg.height <= 0 || cfg.fx <= 0 || cfg.baseline_m <= 0) return false;

  cuvslam::WarmUpGPU();

  auto cam = [&](float tx) {
    cuvslam::Camera c;
    c.size = {cfg.width, cfg.height};
    c.principal = {static_cast<float>(cfg.cx), static_cast<float>(cfg.cy)};
    c.focal = {static_cast<float>(cfg.fx), static_cast<float>(cfg.fy)};
    c.distortion = {};  // Pinhole, 0 params (rectified input)
    c.rig_from_camera.translation = {tx, 0.f, 0.f};
    return c;
  };
  cuvslam::Rig rig;
  rig.cameras = {cam(0.f), cam(static_cast<float>(cfg.baseline_m))};
  if (cfg.use_imu) {
    cuvslam::ImuCalibration imu;
    imu.rig_from_imu.translation = {static_cast<float>(cfg.imu_tx),
                                    static_cast<float>(cfg.imu_ty),
                                    static_cast<float>(cfg.imu_tz)};
    imu.gyroscope_noise_density = static_cast<float>(cfg.gyro_noise_density);
    imu.gyroscope_random_walk = static_cast<float>(cfg.gyro_random_walk);
    imu.accelerometer_noise_density = static_cast<float>(cfg.accel_noise_density);
    imu.accelerometer_random_walk = static_cast<float>(cfg.accel_random_walk);
    imu.frequency = static_cast<float>(cfg.imu_frequency);
    rig.imus = {imu};
  }

  cuvslam::Odometry::Config ocfg;
  ocfg.odometry_mode = cfg.use_imu ? cuvslam::Odometry::OdometryMode::Inertial
                                   : cuvslam::Odometry::OdometryMode::Multicamera;
  ocfg.async_sba = cfg.async_sba;
  ocfg.rectified_stereo_camera = true;
  ocfg.enable_observations_export = cfg.health;  // pnp_health rides the stat export
  try {
    impl_->odom = std::make_unique<cuvslam::Odometry>(rig, ocfg);
  } catch (const std::exception&) {
    impl_->odom.reset();
    return false;
  }
  return true;
}

const CuvslamHealth& CuvslamProvider::lastHealth() const { return impl_->health; }

void CuvslamProvider::registerImu(double t_s, double ax, double ay, double az,
                                  double gx, double gy, double gz) {
  if (!impl_->odom || !impl_->cfg.use_imu) return;
  cuvslam::ImuMeasurement m;
  m.timestamp_ns = static_cast<int64_t>(t_s * 1e9);
  m.linear_accelerations = {static_cast<float>(ax), static_cast<float>(ay),
                            static_cast<float>(az)};
  m.angular_velocities = {static_cast<float>(gx), static_cast<float>(gy),
                          static_cast<float>(gz)};
  try {
    impl_->odom->RegisterImuMeasurement(0, m);
  } catch (const std::exception&) {
  }
}

std::optional<ProviderSample> CuvslamProvider::track(double t_s, const std::uint8_t* left,
                                                     const std::uint8_t* right, int pitch) {
  if (!impl_->odom) return std::nullopt;
  const auto& cfg = impl_->cfg;

  auto img = [&](const std::uint8_t* px, uint32_t cam_idx) {
    cuvslam::Image im{};
    im.pixels = px;
    im.width = cfg.width;
    im.height = cfg.height;
    im.pitch = pitch;
    im.encoding = cuvslam::ImageData::Encoding::MONO;
    im.data_type = cuvslam::ImageData::DataType::UINT8;
    im.is_gpu_mem = false;
    im.timestamp_ns = static_cast<int64_t>(t_s * 1e9);
    im.camera_index = cam_idx;
    return im;
  };
  const cuvslam::Odometry::ImageSet images = {img(left, 0), img(right, 1)};

  cuvslam::PoseEstimate est;
  try {
    est = impl_->odom->Track(images);
  } catch (const std::exception& e) {
    static int nlog = 0;
    if (nlog++ < 5) std::fprintf(stderr, "[cuvslam_provider] Track threw: %s\n", e.what());
    impl_->health = {};
    return std::nullopt;
  }

  CuvslamHealth& h = impl_->health;
  h = {};
  if (cfg.health) {
    try {
      impl_->odom->GetState(impl_->state);
      const auto& p = impl_->state.pnp_health;
      h.observations = p.observations;
      h.inliers = p.inliers;
      h.mean_residual = p.mean_residual;
      h.final_cost = p.final_cost;
      h.info_condition = p.info_condition;
      h.suspect = (p.inliers < cfg.min_inliers) ||
                  (p.info_condition > cfg.max_info_condition);
    } catch (const std::exception&) {
      // stock lib without the fork: no health channel, never suspect by this path
    }
  }

  if (!est.world_from_rig.has_value()) return std::nullopt;  // hard loss
  h.valid = true;

  const auto& wr = *est.world_from_rig;
  ProviderSample s;
  s.t = t_s;
  s.T_OB = cuvslam_conv::poseToRos(wr.pose.rotation, wr.pose.translation);
  Eigen::Matrix<double, 6, 6> cov = cuvslam_conv::covToRos(wr.covariance.data());
  cov *= cfg.cov_scale;                       // NEES calibration
  if (h.suspect) cov *= cfg.suspect_cov_mult; // bounded degradation (Hard Rule #3)
  s.cov = cov;
  return s;
}

}  // namespace slamko
