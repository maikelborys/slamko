// SPDX-License-Identifier: Apache-2.0
// MapPointStore — Phase A (drift-tolerant associate) + Phase B (multi-view refine) unit tests.
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "slamko_loop/mappoint_store.hpp"

using slamko::MapPointStore;

namespace {
// A canonical L2-normalised 64-D descriptor pointing mostly along axis k (k in [0,64)).
Eigen::Matrix<float, 1, 64, Eigen::RowMajor> desc(int k, float noise = 0.0f) {
  Eigen::Matrix<float, 1, 64, Eigen::RowMajor> d;
  for (int i = 0; i < 64; ++i) d(0, i) = (i == k) ? 1.0f : noise;
  d /= d.norm();
  return d;
}
}  // namespace

// Phase A: a revisit point within radius + descriptor cosine re-associates (drift-tolerant);
// a far point or a mismatched descriptor does not.
TEST(MapPointStore, DriftTolerantAssociate) {
  MapPointStore s(/*cell=*/0.4, /*radius=*/0.4, /*cos=*/0.82f);
  s.add(1, Eigen::Vector3d(0, 0, 0), desc(3));
  // Same point, 0.3 m of VIO drift (< radius) + same descriptor -> ASSOCIATES.
  EXPECT_EQ(s.associate(Eigen::Vector3d(0.3, 0, 0), desc(3)), 1);
  // Within radius but a DIFFERENT descriptor -> no association (the discriminator).
  EXPECT_EQ(s.associate(Eigen::Vector3d(0.1, 0, 0), desc(10)), -1);
  // Same descriptor but OUTSIDE radius -> no association.
  EXPECT_EQ(s.associate(Eigen::Vector3d(2.0, 0, 0), desc(3)), -1);
}

// Phase B: refine folds re-observations into a running-mean position and bumps n_obs, so the
// stored point converges to the CONSENSUS (averaging out per-visit noise).
TEST(MapPointStore, MultiViewRefineRunningMean) {
  MapPointStore s(0.4, 0.4, 0.82f);
  s.add(7, Eigen::Vector3d(1.0, 0, 0), desc(5));   // first visit, n_obs=1
  s.refine(7, Eigen::Vector3d(1.2, 0, 0), desc(5));  // 2nd: mean -> 1.1
  s.refine(7, Eigen::Vector3d(1.3, 0, 0), desc(5));  // 3rd: mean -> 1.1667
  const Eigen::Vector3d* p = s.position(7);
  ASSERT_NE(p, nullptr);
  EXPECT_NEAR(p->x(), (1.0 + 1.2 + 1.3) / 3.0, 1e-6);
  EXPECT_NEAR(p->y(), 0.0, 1e-6);
  EXPECT_EQ(s.points()[0].n_obs, 3);
  // The descriptor stays L2-normalised after refinement.
  EXPECT_NEAR(s.points()[0].desc.norm(), 1.0f, 1e-5f);
}

// position() returns nullptr for an unknown id; the store never invents geometry.
TEST(MapPointStore, PositionUnknownIsNull) {
  MapPointStore s;
  EXPECT_EQ(s.position(999), nullptr);
}
