// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// CeresLocalSmoother adapter tests. The adapter's whole job is to be an EXACT
// pass-through to LocalBA across the slamko_core::LocalSmoother contract: same
// optimisation, just T_WB↔T_w_c and StereoObservation↔parallel-array repacking.
// These deterministic tests pin that — the P0 end-to-end gate is the EuRoC
// bench (front-end CUDA nondeterminism lives upstream of this seam).

#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slamko_core/se3.hpp"
#include "slamko_core/stereo_observation.hpp"

#include "slamko_vio/ceres_local_smoother.hpp"
#include "slamko_vio/local_ba.hpp"
#include "slamko_vio/types.hpp"

namespace {

using slamko_vio::CeresLocalSmoother;
using slamko_vio::CeresLocalSmootherConfig;
using slamko_vio::LocalBA;
using slamko_vio::StereoIntrinsics;

constexpr double kTol = 1e-9;  // SE3-quaternion-seam tolerance (≫ ~1e-15 actual)

StereoIntrinsics testK() { return StereoIntrinsics{458.6f, 457.3f, 367.2f, 248.4f, 0.11f}; }

// A non-trivial cam→body extrinsic (rotation + translation), the kind T_BS is.
Eigen::Matrix4d testExtrinsic() {
  Eigen::Matrix4d E = Eigen::Matrix4d::Identity();
  E.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(0.37, Eigen::Vector3d(0.2, -0.5, 0.84).normalized())
          .toRotationMatrix();
  E.block<3, 1>(0, 3) = Eigen::Vector3d(0.021, -0.064, 0.009);
  return E;
}

// A small synthetic scene: N landmarks visible in two keyframes, with left-only
// pinhole observations. The geometry need not be perfect — the point is that
// BOTH paths receive byte-identical inputs, so they must return identical output.
struct Scene {
  std::vector<Eigen::Matrix4d> T_w_c;                          // world→cam per KF
  std::vector<Eigen::Vector3d> lm_world;                       // truth landmark
  std::vector<std::vector<Eigen::Vector2d>> uv_left;           // [kf][lm]
  std::vector<std::uint32_t> ids;
};

Scene makeScene() {
  Scene s;
  const StereoIntrinsics K = testK();
  // Two looking-forward poses (world→cam = identity rot, cam at -z offsets).
  for (double z : {0.0, -0.25}) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = 0.05 * z;          // small lateral shift between KFs
    T(2, 3) = z;                 // forward translation (world→cam)
    s.T_w_c.push_back(T);
  }
  for (int i = 0; i < 24; ++i) {
    const double x = -1.0 + 0.1 * (i % 8);
    const double y = -0.6 + 0.15 * (i / 8);
    const double z = 4.0 + 0.3 * (i % 5);
    s.lm_world.emplace_back(x, y, z);
    s.ids.push_back(static_cast<std::uint32_t>(100 + i));
  }
  for (const auto& T : s.T_w_c) {
    std::vector<Eigen::Vector2d> row;
    for (const auto& p : s.lm_world) {
      const Eigen::Vector3d pc = T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);
      row.emplace_back(K.fx * pc.x() / pc.z() + K.cx,
                       K.fy * pc.y() / pc.z() + K.cy);
    }
    s.uv_left.push_back(row);
  }
  return s;
}

LocalBA::Config baseConfig() {
  LocalBA::Config c;
  c.window_size = 5;
  c.use_inv_depth = false;   // Euclidean landmarks → directly comparable
  c.enable_imu = false;      // visual-only parity (T_BS irrelevant to the solve)
  c.min_observations_per_landmark = 2;
  return c;
}

// Run the scene through a bare LocalBA (the baseline path).
Eigen::Matrix4d runDirect(const Scene& s, std::vector<Eigen::Vector3d>& lm_out) {
  LocalBA ba(baseConfig());
  const StereoIntrinsics K = testK();
  const Eigen::Vector2d nan2(std::nan(""), std::nan(""));
  for (std::size_t k = 0; k < s.T_w_c.size(); ++k) {
    std::vector<Eigen::Vector2d> uvr(s.ids.size(), nan2);  // left-only
    ba.insert_keyframe(static_cast<double>(k), s.T_w_c[k], K, s.ids,
                       s.uv_left[k], uvr, s.lm_world);
  }
  EXPECT_TRUE(ba.solve());
  Eigen::Matrix4d T;
  EXPECT_TRUE(ba.latest_pose(T));
  lm_out.clear();
  for (auto id : s.ids) {
    Eigen::Vector3d p;
    ba.landmark_world(id, p);
    lm_out.push_back(p);
  }
  return T;
}

// Run the same scene through the adapter with a given extrinsic E. The pipeline
// hands the contract T_WB = (E·T_w_c)⁻¹; the adapter must recover T_w_c exactly.
Eigen::Matrix4d runAdapter(const Scene& s, const Eigen::Matrix4d& E,
                           std::vector<Eigen::Vector3d>& lm_out) {
  CeresLocalSmootherConfig cfg;
  cfg.ba = baseConfig();
  CeresLocalSmoother sm(cfg);
  sm.setExtrinsics(slamko::SE3(E));
  const StereoIntrinsics K = testK();
  sm.setStereoCalib(slamko::StereoCalib{K.fx, K.fy, K.cx, K.cy, K.baseline_m});

  for (std::size_t k = 0; k < s.T_w_c.size(); ++k) {
    std::vector<slamko::StereoObservation> obs;
    for (std::size_t i = 0; i < s.ids.size(); ++i) {
      slamko::StereoObservation o;
      o.landmark_id = s.ids[i];
      o.uv_left = s.uv_left[k][i];
      // uv_right left default (NaN-x) ⇒ left-only, matching runDirect.
      o.world_init = s.lm_world[i];
      obs.push_back(o);
    }
    const slamko::SE3 T_WB(Eigen::Matrix4d((E * s.T_w_c[k]).inverse()));
    sm.insertKeyframe(static_cast<double>(k), T_WB, Eigen::Vector3d::Zero(),
                      slamko::ImuBias{}, /*imu=*/{}, obs);
  }
  EXPECT_TRUE(sm.optimize());
  // latestPose() is T_WB; convert back to T_w_c for comparison with runDirect.
  const Eigen::Matrix4d T_w_c =
      E.inverse() * sm.latestPose().matrix().inverse();
  lm_out.clear();
  for (auto id : s.ids) {
    Eigen::Vector3d p;
    sm.landmark(id, p);
    lm_out.push_back(p);
  }
  return T_w_c;
}

void expectClose(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b, double tol) {
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) EXPECT_NEAR(a(r, c), b(r, c), tol) << "(" << r << "," << c << ")";
}

// ---- IMU-path parity (the path that actually runs end-to-end) --------------
slamko_vio::ImuNoise testNoise() {
  slamko_vio::ImuNoise n;  // EuRoC-ish defaults; identical on both paths anyway
  return n;
}

// IMU samples spanning (KF0, KF1]: a constant specific force + small rotation
// rate. Values need not be physically exact — both paths integrate the SAME
// samples with the SAME noise/bias, so any difference would be the adapter's.
std::vector<slamko_vio::ImuSample> makeImu() {
  std::vector<slamko_vio::ImuSample> v;
  for (int i = 0; i <= 10; ++i) {
    slamko_vio::ImuSample s;
    s.t = 1.0 + 0.005 * i;                       // KF1 timestamp = 1.05
    s.a = Eigen::Vector3d(0.05, -0.02, 9.81);    // specific force (≈ level, +z up)
    s.w = Eigen::Vector3d(0.01, -0.015, 0.004);  // rad/s
    v.push_back(s);
  }
  return v;
}

LocalBA::Config imuConfig(const Eigen::Matrix4d& E) {
  LocalBA::Config c = baseConfig();
  c.enable_imu = true;
  c.T_BS = E;
  c.gravity_w = Eigen::Vector3d(0.0, 0.0, -9.81);
  return c;
}

const Eigen::Vector3d kSeedVel(0.10, 0.0, -0.5);

// Direct LocalBA with IMU: KF0 visual-only (gauge), KF1 via insert_keyframe_with_imu.
Eigen::Matrix4d runDirectImu(const Scene& s, const Eigen::Matrix4d& E,
                             Eigen::Vector3d& v_out, slamko_vio::ImuBias& b_out) {
  LocalBA ba(imuConfig(E));
  const StereoIntrinsics K = testK();
  const Eigen::Vector2d nan2(std::nan(""), std::nan(""));
  std::vector<Eigen::Vector2d> uvr(s.ids.size(), nan2);

  ba.insert_keyframe(0.0, s.T_w_c[0], K, s.ids, s.uv_left[0], uvr, s.lm_world);

  const auto imu = makeImu();
  slamko_vio::ImuPreintegration pi(slamko_vio::ImuBias{}, testNoise());
  pi.integrate_span(imu.data(), imu.size());
  ba.insert_keyframe_with_imu(1.05, s.T_w_c[1], kSeedVel, slamko_vio::ImuBias{},
                              pi, K, s.ids, s.uv_left[1], uvr, s.lm_world);
  EXPECT_TRUE(ba.solve());
  Eigen::Matrix4d T;
  EXPECT_TRUE(ba.latest_pose(T));
  ba.latest_velocity(v_out);
  ba.latest_bias(b_out);
  return T;
}

// Adapter with IMU: same two KFs through the contract (KF0 empty IMU, KF1 raw IMU).
Eigen::Matrix4d runAdapterImu(const Scene& s, const Eigen::Matrix4d& E,
                              Eigen::Vector3d& v_out, slamko_vio::ImuBias& b_out) {
  CeresLocalSmootherConfig cfg;
  cfg.ba = baseConfig();        // enable_imu/T_BS/gravity arrive via the setters
  cfg.ba.enable_imu = true;
  cfg.noise = testNoise();
  CeresLocalSmoother sm(cfg);
  sm.setExtrinsics(slamko::SE3(E));
  slamko::ImuParams p;
  p.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  p.accel_noise_density = testNoise().accel_noise_density;
  p.gyro_noise_density  = testNoise().gyro_noise_density;
  p.accel_bias_rw       = baseConfig().bias_rw_accel;
  p.gyro_bias_rw        = baseConfig().bias_rw_gyro;
  sm.setImuParams(p);
  const StereoIntrinsics K = testK();
  sm.setStereoCalib(slamko::StereoCalib{K.fx, K.fy, K.cx, K.cy, K.baseline_m});

  auto obsOf = [&](std::size_t k) {
    std::vector<slamko::StereoObservation> obs;
    for (std::size_t i = 0; i < s.ids.size(); ++i) {
      slamko::StereoObservation o;
      o.landmark_id = s.ids[i];
      o.uv_left = s.uv_left[k][i];
      o.world_init = s.lm_world[i];
      obs.push_back(o);
    }
    return obs;
  };

  sm.insertKeyframe(0.0, slamko::SE3(Eigen::Matrix4d((E * s.T_w_c[0]).inverse())),
                    Eigen::Vector3d::Zero(), slamko::ImuBias{}, /*imu=*/{}, obsOf(0));

  const auto imu = makeImu();
  std::vector<slamko::ImuSample> cimu;
  for (const auto& m : imu) {
    slamko::ImuSample x; x.timestamp = m.t; x.accel = m.a; x.gyro = m.w;
    cimu.push_back(x);
  }
  sm.insertKeyframe(1.05, slamko::SE3(Eigen::Matrix4d((E * s.T_w_c[1]).inverse())),
                    kSeedVel, slamko::ImuBias{}, cimu, obsOf(1));
  EXPECT_TRUE(sm.optimize());
  const Eigen::Matrix4d T_w_c = E.inverse() * sm.latestPose().matrix().inverse();
  v_out = sm.latestVelocity();
  const slamko::ImuBias b = sm.latestBias();
  b_out.bg = b.gyro;
  b_out.ba = b.accel;
  return T_w_c;
}

}  // namespace

// The adapter feeds LocalBA identical inputs and returns identical outputs as a
// direct LocalBA call — exact up to the SE3 quaternion seam — for an identity
// extrinsic (the cleanest comparison: T_WB = T_w_c⁻¹).
TEST(CeresLocalSmoother, VisualParityIdentityExtrinsic) {
  const Scene s = makeScene();
  std::vector<Eigen::Vector3d> lm_direct, lm_adapter;
  const Eigen::Matrix4d T_direct = runDirect(s, lm_direct);
  const Eigen::Matrix4d T_adapter =
      runAdapter(s, Eigen::Matrix4d::Identity(), lm_adapter);
  expectClose(T_direct, T_adapter, kTol);
  ASSERT_EQ(lm_direct.size(), lm_adapter.size());
  for (std::size_t i = 0; i < lm_direct.size(); ++i)
    EXPECT_LT((lm_direct[i] - lm_adapter[i]).norm(), kTol) << "landmark " << i;
}

// A non-identity cam→body extrinsic must NOT change the visual-only solve (T_BS
// only enters the IMU factor) — so the recovered T_w_c still matches the direct
// path. This is the load-bearing check on the T_WB↔T_w_c pose algebra.
TEST(CeresLocalSmoother, VisualParityNonIdentityExtrinsic) {
  const Scene s = makeScene();
  std::vector<Eigen::Vector3d> lm_direct, lm_adapter;
  const Eigen::Matrix4d T_direct = runDirect(s, lm_direct);
  const Eigen::Matrix4d T_adapter = runAdapter(s, testExtrinsic(), lm_adapter);
  expectClose(T_direct, T_adapter, kTol);
  ASSERT_EQ(lm_direct.size(), lm_adapter.size());
  for (std::size_t i = 0; i < lm_direct.size(); ++i)
    EXPECT_LT((lm_direct[i] - lm_adapter[i]).norm(), kTol) << "landmark " << i;
}

// The IMU path (insert_keyframe_with_imu — what runs end-to-end) is also an
// exact pass-through: same preintegration, same velocity/bias, just the pose
// frame conversion. Pose + refined velocity + bias must match the direct path.
TEST(CeresLocalSmoother, ImuParityNonIdentityExtrinsic) {
  const Scene s = makeScene();
  const Eigen::Matrix4d E = testExtrinsic();
  Eigen::Vector3d v_direct, v_adapter;
  slamko_vio::ImuBias b_direct, b_adapter;
  const Eigen::Matrix4d T_direct  = runDirectImu(s, E, v_direct, b_direct);
  const Eigen::Matrix4d T_adapter = runAdapterImu(s, E, v_adapter, b_adapter);
  expectClose(T_direct, T_adapter, kTol);
  EXPECT_LT((v_direct - v_adapter).norm(), kTol);
  EXPECT_LT((b_direct.bg - b_adapter.bg).norm(), kTol);
  EXPECT_LT((b_direct.ba - b_adapter.ba).norm(), kTol);
}

// Pure pose-algebra sanity: T_w_c → T_WB → T_w_c is identity within the seam,
// for a non-identity extrinsic and a non-trivial pose.
TEST(CeresLocalSmoother, PoseConversionRoundTrip) {
  const Eigen::Matrix4d E = testExtrinsic();
  Eigen::Matrix4d T_w_c = Eigen::Matrix4d::Identity();
  T_w_c.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(-0.9, Eigen::Vector3d(0.6, 0.3, -0.74).normalized())
          .toRotationMatrix();
  T_w_c.block<3, 1>(0, 3) = Eigen::Vector3d(1.3, -2.1, 0.7);

  const slamko::SE3 T_WB(Eigen::Matrix4d((E * T_w_c).inverse()));   // forward (pipeline)
  const Eigen::Matrix4d back = E.inverse() * T_WB.matrix().inverse();  // back (adapter)
  expectClose(T_w_c, back, kTol);
}
