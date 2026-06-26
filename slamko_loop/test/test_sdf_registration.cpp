// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Point-to-SDF ICP = the DENSE geometric verify/align channel. The load-bearing property:
// a depth cloud placed at a DRIFTED pose snaps back onto the already-mapped surfaces, and
// that snap IS the loop/recovery correction — viewpoint-robust where appearance + sparse
// ScanContext fail. Tested against an analytic 3-orthogonal-plane field (constrains all
// 6 DOF, the nvblox-ESDF stand-in): a known transform offset is recovered to mm/mrad.

#include <cmath>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "slamko_core/se3.hpp"
#include "slamko_loop/sdf_registration.hpp"

using slamko::SE3;
using slamko::SO3;

namespace {
// Signed distance to the nearest of 3 orthogonal planes (x=0, y=0, z=0) — a "room corner".
// Returns {signed distance to that plane, its unit normal}. A valid local SDF for points
// near the surfaces; constrains translation (3 normals) + rotation (spatial spread).
struct ThreePlanes {
  std::pair<double, Eigen::Vector3d> operator()(const Eigen::Vector3d& p) const {
    const double dx = p.x(), dy = p.y(), dz = p.z();
    const double ax = std::abs(dx), ay = std::abs(dy), az = std::abs(dz);
    if (ax <= ay && ax <= az) return {dx, Eigen::Vector3d(1, 0, 0)};
    if (ay <= ax && ay <= az) return {dy, Eigen::Vector3d(0, 1, 0)};
    return {dz, Eigen::Vector3d(0, 0, 1)};
  }
};

SE3 makeSE3(double rx, double ry, double rz, const Eigen::Vector3d& t) {
  return SE3(SO3::exp(Eigen::Vector3d(rx, ry, rz)), t);
}

// Surface points on the 3 planes, spread out so the field is well-conditioned.
std::vector<Eigen::Vector3d> cornerSurface() {
  std::vector<Eigen::Vector3d> s;
  for (double a = -1.5; a <= 1.5; a += 0.15)
    for (double b = 0.2; b <= 2.0; b += 0.15) {
      s.emplace_back(a, b, 0.0);  // floor z=0
      s.emplace_back(0.0, a, b);  // wall  x=0
      s.emplace_back(b, 0.0, a);  // wall  y=0
    }
  return s;
}
}  // namespace

TEST(SdfRegistration, RecoversKnownTransform) {
  const ThreePlanes field;
  const std::vector<Eigen::Vector3d> S = cornerSurface();  // surface pts in MAP frame
  const SE3 T_true = makeSE3(0.05, -0.08, 0.12, {0.20, -0.15, 0.10});
  // query cloud in the QUERY frame: T_true·query == S (exactly on the surface).
  std::vector<Eigen::Vector3d> query;
  query.reserve(S.size());
  for (const auto& p : S) query.push_back(T_true.inverse() * p);
  // start OFFSET from the truth (the drift) and ICP back onto the surfaces.
  const SE3 T_init = T_true * SE3::exp((slamko::Vector6d() << 0.1, -0.08, 0.06, 0.07, -0.05, 0.09).finished());

  auto r = slamko::registerToSdf(query, T_init, field);
  ASSERT_TRUE(r.converged);
  EXPECT_GT(r.inliers, 200);
  EXPECT_LT(r.rms, 1e-3) << "cloud should snap onto the surfaces";
  const double err = (r.T_refined.inverse() * T_true).log().norm();
  EXPECT_LT(err, 1e-2) << "recovered transform should match the truth (err=" << err << ")";
}

TEST(SdfRegistration, RejectsTooFewInliers) {
  const ThreePlanes field;
  // points far from any surface → all beyond the correspondence gate → not converged.
  std::vector<Eigen::Vector3d> far;
  for (int i = 0; i < 100; ++i) far.emplace_back(10.0 + i, 10.0, 10.0);
  auto r = slamko::registerToSdf(far, SE3(), field);
  EXPECT_FALSE(r.converged);
  EXPECT_LT(r.inliers, 30);
}

TEST(SdfRegistration, EmptyCloudSafe) {
  const ThreePlanes field;
  auto r = slamko::registerToSdf({}, SE3(), field);
  EXPECT_FALSE(r.converged);
  EXPECT_EQ(r.inliers, 0);
}
