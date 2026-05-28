// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// SubMap (de)serialization round-trip tests — a synthetic submap (keyframes +
// landmarks + N×64 descriptors + a non-identity anchor) must survive save→load
// bit-for-bit, and a multi-submap archive directory must round-trip via its
// manifest. This is the persistence foundation for cross-session recovery (P4).

#include <filesystem>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "slamko_core/submap.hpp"
#include "slamko_core/submap_io.hpp"

using slamko::loadSubMap;
using slamko::loadSubMaps;
using slamko::saveSubMap;
using slamko::saveSubMaps;
using slamko::SE3;
using slamko::SO3;
using slamko::SubMap;

namespace {

SE3 makeSE3(double tx, double ty, double tz, double angle, Eigen::Vector3d axis) {
  const Eigen::Matrix3d R =
      Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  return SE3(R, Eigen::Vector3d(tx, ty, tz));
}

SubMap synthetic(std::uint64_t id, int n_lm, int n_kf) {
  SubMap sm;
  sm.id = id;
  sm.anchor = makeSE3(1.0 + id, -2.0, 0.5, 0.3 + 0.1 * id, {0.2, 1.0, 0.3});
  for (int k = 0; k < n_kf; ++k)
    sm.keyframes.push_back({static_cast<std::uint64_t>(k), 100.0 + 0.05 * k,
                            makeSE3(0.1 * k, 0.2 * k, 0.0, 0.01 * k, {0, 0, 1})});
  sm.descriptors.resize(n_lm, 64);
  for (int i = 0; i < n_lm; ++i) {
    slamko::MapLandmark lm;
    lm.id = static_cast<std::uint64_t>(1000 + i);
    lm.position = Eigen::Vector3d(0.01 * i, -0.02 * i, 0.5 + 0.001 * i);
    lm.descriptor_row = i;
    sm.landmarks.push_back(lm);
    for (int d = 0; d < 64; ++d) sm.descriptors(i, d) = 0.001f * (i * 64 + d);
  }
  return sm;
}

void expectEqual(const SubMap& a, const SubMap& b) {
  EXPECT_EQ(a.id, b.id);
  EXPECT_LT((a.anchor.matrix() - b.anchor.matrix()).cwiseAbs().maxCoeff(), 1e-12);
  ASSERT_EQ(a.keyframes.size(), b.keyframes.size());
  for (size_t k = 0; k < a.keyframes.size(); ++k) {
    EXPECT_EQ(a.keyframes[k].id, b.keyframes[k].id);
    EXPECT_DOUBLE_EQ(a.keyframes[k].timestamp, b.keyframes[k].timestamp);
    EXPECT_LT((a.keyframes[k].T_WB.matrix() - b.keyframes[k].T_WB.matrix())
                  .cwiseAbs().maxCoeff(), 1e-12);
  }
  ASSERT_EQ(a.landmarks.size(), b.landmarks.size());
  for (size_t i = 0; i < a.landmarks.size(); ++i) {
    EXPECT_EQ(a.landmarks[i].id, b.landmarks[i].id);
    EXPECT_EQ(a.landmarks[i].descriptor_row, b.landmarks[i].descriptor_row);
    EXPECT_LT((a.landmarks[i].position - b.landmarks[i].position).norm(), 1e-12);
  }
  ASSERT_EQ(a.descriptors.rows(), b.descriptors.rows());
  ASSERT_EQ(a.descriptors.cols(), b.descriptors.cols());
  if (a.descriptors.size())
    EXPECT_EQ((a.descriptors - b.descriptors).cwiseAbs().maxCoeff(), 0.0f);  // binary-exact
  ASSERT_EQ(a.global_descriptor.size(), b.global_descriptor.size());
  if (a.global_descriptor.size())
    EXPECT_EQ((a.global_descriptor - b.global_descriptor).cwiseAbs().maxCoeff(), 0.0f);
  ASSERT_EQ(a.kf_obs.size(), b.kf_obs.size());
  for (size_t k = 0; k < a.kf_obs.size(); ++k) {
    ASSERT_EQ(a.kf_obs[k].landmark_ids, b.kf_obs[k].landmark_ids);
    ASSERT_EQ(a.kf_obs[k].uv.rows(), b.kf_obs[k].uv.rows());
    if (a.kf_obs[k].uv.size())
      EXPECT_EQ((a.kf_obs[k].uv - b.kf_obs[k].uv).cwiseAbs().maxCoeff(), 0.0f);
    ASSERT_EQ(a.kf_obs[k].uv_right.rows(), b.kf_obs[k].uv_right.rows());
    if (a.kf_obs[k].uv_right.size())
      EXPECT_EQ((a.kf_obs[k].uv_right - b.kf_obs[k].uv_right).cwiseAbs().maxCoeff(), 0.0f);
    ASSERT_EQ(a.kf_obs[k].global_descriptor.size(), b.kf_obs[k].global_descriptor.size());
    if (a.kf_obs[k].global_descriptor.size())
      EXPECT_EQ(
          (a.kf_obs[k].global_descriptor - b.kf_obs[k].global_descriptor)
              .cwiseAbs()
              .maxCoeff(),
          0.0f);
  }
}

std::string tmpdir() {
  return (std::filesystem::temp_directory_path() / "slamko_submap_io_test").string();
}

}  // namespace

TEST(SubMapIO, SingleRoundTrip) {
  const SubMap sm = synthetic(7, /*n_lm=*/40, /*n_kf=*/5);
  const std::string path = tmpdir() + "_single.smap";
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  ASSERT_TRUE(saveSubMap(sm, path));
  SubMap got;
  ASSERT_TRUE(loadSubMap(got, path));
  expectEqual(sm, got);
}

TEST(SubMapIO, EmptyDescriptorsRoundTrip) {
  SubMap sm = synthetic(3, /*n_lm=*/6, /*n_kf=*/2);
  sm.descriptors.resize(0, 0);                 // descriptor-less (Shi-Tomasi) submap
  for (auto& l : sm.landmarks) l.descriptor_row = -1;
  const std::string path = tmpdir() + "_empty.smap";
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  ASSERT_TRUE(saveSubMap(sm, path));
  SubMap got;
  ASSERT_TRUE(loadSubMap(got, path));
  expectEqual(sm, got);
  EXPECT_EQ(got.descriptors.size(), 0);
}

TEST(SubMapIO, ArchiveDirectoryRoundTrip) {
  std::vector<SubMap> maps = {synthetic(0, 30, 3), synthetic(1, 50, 4),
                              synthetic(2, 20, 2)};
  const std::string dir = tmpdir() + "_archive";
  ASSERT_TRUE(saveSubMaps(maps, dir));
  std::vector<SubMap> got;
  ASSERT_TRUE(loadSubMaps(got, dir));
  ASSERT_EQ(got.size(), maps.size());
  for (size_t i = 0; i < maps.size(); ++i) expectEqual(maps[i], got[i]);
}

TEST(SubMapIO, MissingFileFails) {
  SubMap sm;
  EXPECT_FALSE(loadSubMap(sm, tmpdir() + "_does_not_exist.smap"));
  std::vector<SubMap> v;
  EXPECT_FALSE(loadSubMaps(v, tmpdir() + "_no_such_dir"));
}

// SMP3 per-keyframe observations: stereo + mono entries round-trip bit-exact, the
// have_right flag survives, and empty entries (KF with no obs) load back empty. This
// is the BA substrate (per docs/PLAN_BA_GLOBAL.md) — a wrong codec here corrupts every
// reprojection factor downstream.
TEST(SubMapIO, KeyframeObservationsRoundTrip) {
  SubMap sm = synthetic(11, /*n_lm=*/8, /*n_kf=*/3);
  sm.global_descriptor.resize(5);
  for (int i = 0; i < 5; ++i) sm.global_descriptor[i] = 0.1f * (i + 1);
  sm.kf_obs.resize(sm.keyframes.size());
  // KF 0: 4 stereo observations.
  sm.kf_obs[0].landmark_ids = {1000, 1001, 1003, 1005};
  sm.kf_obs[0].uv.resize(4, 2);
  sm.kf_obs[0].uv_right.resize(4, 2);
  for (int i = 0; i < 4; ++i) {
    sm.kf_obs[0].uv(i, 0) = 100.f + 7.f * i;
    sm.kf_obs[0].uv(i, 1) = 220.f - 3.f * i;
    sm.kf_obs[0].uv_right(i, 0) = sm.kf_obs[0].uv(i, 0) - 12.f;  // disparity
    sm.kf_obs[0].uv_right(i, 1) = sm.kf_obs[0].uv(i, 1);
  }
  // KF 1: 2 monocular observations (have_right=0 path).
  sm.kf_obs[1].landmark_ids = {1002, 1004};
  sm.kf_obs[1].uv.resize(2, 2);
  sm.kf_obs[1].uv << 40.f, 50.f, 60.f, 70.f;
  // KF 2: empty (legal — keyframe inserted but landmark observations cleared).

  const std::string path = tmpdir() + "_kfobs.smap";
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  ASSERT_TRUE(saveSubMap(sm, path));
  SubMap got;
  ASSERT_TRUE(loadSubMap(got, path));
  expectEqual(sm, got);
  EXPECT_TRUE(got.kf_obs[0].hasStereo());
  EXPECT_FALSE(got.kf_obs[1].hasStereo());
  EXPECT_EQ(got.kf_obs[2].size(), 0);
}

// SMP4 per-keyframe VPR descriptor round-trip. Each KF carries its OWN 512-D global
// descriptor (the granularity fix — per-submap aggregation loses the place signal on
// long revisits, per docs/PLAN_BA_GLOBAL.md). Distinct values per KF make a swap or
// truncation visible; an empty KF must load back empty (the back-compat path for
// VPR-less KFs in an otherwise SMP4 archive).
TEST(SubMapIO, PerKeyframeVprRoundTrip) {
  SubMap sm = synthetic(13, /*n_lm=*/8, /*n_kf=*/3);
  sm.kf_obs.resize(sm.keyframes.size());
  // KF 0: a real 8-D per-KF descriptor (the synthetic dim — codec is dim-agnostic).
  sm.kf_obs[0].global_descriptor.resize(8);
  for (int i = 0; i < 8; ++i) sm.kf_obs[0].global_descriptor[i] = 0.01f * (i + 1);
  // KF 1: distinct 8-D descriptor (different values prove no aliasing across KFs).
  sm.kf_obs[1].global_descriptor.resize(8);
  for (int i = 0; i < 8; ++i) sm.kf_obs[1].global_descriptor[i] = -0.02f * (i + 1);
  // KF 2: empty per-KF descriptor (legal — codec writes kf_gdim=0).

  const std::string path = tmpdir() + "_kfvpr.smap";
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  ASSERT_TRUE(saveSubMap(sm, path));
  SubMap got;
  ASSERT_TRUE(loadSubMap(got, path));
  expectEqual(sm, got);
  EXPECT_TRUE(got.kf_obs[0].hasGlobalDescriptor());
  EXPECT_TRUE(got.kf_obs[1].hasGlobalDescriptor());
  EXPECT_FALSE(got.kf_obs[2].hasGlobalDescriptor());
  // Spot-check distinct values survived (catches a copy-paste swap across KFs).
  EXPECT_FLOAT_EQ(got.kf_obs[0].global_descriptor[0], 0.01f);
  EXPECT_FLOAT_EQ(got.kf_obs[1].global_descriptor[0], -0.02f);
}
