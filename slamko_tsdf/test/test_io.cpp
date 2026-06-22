// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// depth_io round-trip + the submap→kf_world_pose glue (anchor ∘ T_WB).

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

#include "slamko_core/submap.hpp"
#include "slamko_tsdf/depth_io.hpp"
#include "slamko_tsdf/submap_poses.hpp"

using namespace slamko;

namespace {
std::string tmpPath(const char* name) {
  const char* dir = std::getenv("TEST_TMPDIR");
  return (dir ? std::string(dir) : std::string("/tmp")) + "/" + name;
}
}  // namespace

TEST(DepthIO, RoundTrip) {
  DepthFrame f;
  f.kf_id = 7;
  f.width = 3;
  f.height = 2;
  f.depth = {0.1f, 0.2f, 0.3f, 0.4f, 0.f, 0.6f};
  f.K = {200.0, 201.0, 1.5, 0.5, 3, 2};
  f.T_body_cam = SE3(SO3::exp(Eigen::Vector3d(0.1, -0.2, 0.3)),
                     Eigen::Vector3d(0.01, 0.02, 0.03));

  const std::string p = tmpPath("rt.skdf");
  ASSERT_TRUE(saveDepthFrame(f, p));

  DepthFrame g;
  ASSERT_TRUE(loadDepthFrame(g, p));
  EXPECT_EQ(g.kf_id, 7u);
  EXPECT_EQ(g.width, 3);
  EXPECT_EQ(g.height, 2);
  EXPECT_DOUBLE_EQ(g.K.fx, 200.0);
  EXPECT_DOUBLE_EQ(g.K.cy, 0.5);
  EXPECT_EQ(g.K.width, 3);
  ASSERT_EQ(g.depth.size(), f.depth.size());
  for (std::size_t i = 0; i < f.depth.size(); ++i)
    EXPECT_FLOAT_EQ(g.depth[i], f.depth[i]);
  // extrinsic survives binary exactly
  EXPECT_TRUE(g.T_body_cam.matrix().isApprox(f.T_body_cam.matrix(), 1e-12));
}

TEST(DepthIO, RejectsBadMagic) {
  const std::string p = tmpPath("bad.skdf");
  std::ofstream o(p, std::ios::binary);
  o << "XXXX....";
  o.close();
  DepthFrame g;
  EXPECT_FALSE(loadDepthFrame(g, p));
}

TEST(SubmapPoses, ComposesAnchorAndLocalPose) {
  SubMap a;
  a.id = 0;
  a.anchor = SE3(SO3(), Eigen::Vector3d(10, 0, 0));
  KeyframePose k0;
  k0.id = 1;
  k0.T_WB = SE3(SO3(), Eigen::Vector3d(1, 0, 0));  // local
  a.keyframes = {k0};

  const auto poses = keyframeWorldPoses({a});
  ASSERT_EQ(poses.count(1u), 1u);
  // world = anchor ∘ local = (10,0,0) + (1,0,0)
  EXPECT_DOUBLE_EQ(poses.at(1).translation().x(), 11.0);
}

TEST(SubmapPoses, LaterSubmapWinsOnDuplicateKeyframeId) {
  SubMap a, b;
  a.anchor = SE3(SO3(), Eigen::Vector3d(0, 0, 0));
  b.anchor = SE3(SO3(), Eigen::Vector3d(5, 0, 0));
  KeyframePose ka, kb;
  ka.id = 9;
  ka.T_WB = SE3();
  kb.id = 9;  // same id, re-sealed under a newer anchor
  kb.T_WB = SE3();
  a.keyframes = {ka};
  b.keyframes = {kb};

  const auto poses = keyframeWorldPoses({a, b});
  EXPECT_DOUBLE_EQ(poses.at(9).translation().x(), 5.0);  // b wins
}
