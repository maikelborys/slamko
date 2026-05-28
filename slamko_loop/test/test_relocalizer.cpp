// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// XFeatRelocalizer tests — synthetic 2D-3D PnP, no ROS. Project known
// submap-local landmarks into a query camera at a known pose, attach matching
// descriptors, and assert relocalize() recovers the query BODY pose (incl. the
// cam↔body extrinsic), rejects outlier correspondences, and reports no-match
// cleanly. Mirrors slamko_vio/tests/test_p3p.cpp's scene-construction pattern.

#include <random>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "slamko_core/features.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"

#include "slamko_loop/xfeat_relocalizer.hpp"

using slamko::Features;
using slamko::MapLandmark;
using slamko::RelocResult;
using slamko::SE3;
using slamko::SubMap;
using slamko::XFeatRelocalizer;
using slamko::XFeatRelocConfig;

namespace {

constexpr double kFx = 400, kFy = 400, kCx = 320, kCy = 240;

SE3 makeSE3(double tx, double ty, double tz, double angle, Eigen::Vector3d axis) {
  return SE3(Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix(),
             Eigen::Vector3d(tx, ty, tz));
}

void expectSE3Near(const SE3& a, const SE3& b, double tol) {
  const Eigen::Matrix4d A = a.matrix(), B = b.matrix();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) EXPECT_NEAR(A(i, j), B(i, j), tol) << "(" << i << "," << j << ")";
}

XFeatRelocConfig rcfg(const SE3& T_BS, int min_inliers = 12) {
  XFeatRelocConfig c;
  c.fx = kFx; c.fy = kFy; c.cx = kCx; c.cy = kCy;
  c.body_T_cam = T_BS;
  c.match_ratio = 0.8f;
  c.ransac_iters = 400;
  c.ransac_thresh_px = 2.0;
  c.min_inliers = min_inliers;
  c.seed = 1u;
  return c;
}

struct Scene {
  SubMap submap;
  Features query;
  SE3 expected_T_query_match;  // query BODY pose in sealed-local
};

// Build a submap of n landmarks + a query that observes all of them from a known
// camera pose T_sl_cam (camera-in-submaplocal). Descriptors are unit-random and
// shared query↔landmark so matching is exact. expected weld = T_sl_cam·T_BS⁻¹.
Scene buildScene(const SE3& T_sl_cam, const SE3& T_BS, int n, unsigned seed) {
  Scene sc;
  sc.submap.id = 7;
  sc.submap.anchor = SE3();  // submap-local == world
  sc.submap.descriptors.resize(n, 64);
  sc.query.keypoints.resize(n, 3);
  sc.query.descriptors.resize(n, 64);

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> ux(-1.5, 1.5), uy(-1.0, 1.0), uz(3.0, 6.0);
  std::normal_distribution<float> nd(0.f, 1.f);

  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d Xc(ux(rng), uy(rng), uz(rng));  // in front of the camera
    const Eigen::Vector3d Xsl = T_sl_cam * Xc;            // submap-local

    MapLandmark lm;
    lm.id = 100 + i;
    lm.position = Xsl;
    lm.descriptor_row = i;
    sc.submap.landmarks.push_back(lm);

    Eigen::Matrix<float, 1, 64> d;
    for (int j = 0; j < 64; ++j) d(0, j) = nd(rng);
    d.normalize();
    sc.submap.descriptors.row(i) = d;
    sc.query.descriptors.row(i) = d;  // exact match

    sc.query.keypoints(i, 0) = static_cast<float>(kFx * Xc.x() / Xc.z() + kCx);
    sc.query.keypoints(i, 1) = static_cast<float>(kFy * Xc.y() / Xc.z() + kCy);
    sc.query.keypoints(i, 2) = 1.0f;
  }
  sc.expected_T_query_match = T_sl_cam * T_BS.inverse();
  return sc;
}

}  // namespace

TEST(XFeatRelocalizer, RecoversPoseIdentityExtrinsic) {
  const SE3 T_BS;                                  // identity → body == cam
  const SE3 T_sl_cam = makeSE3(0.5, -0.3, 0.2, 0.25, {0.2, 1.0, -0.3});
  const Scene sc = buildScene(T_sl_cam, T_BS, /*n=*/30, /*seed=*/11);

  XFeatRelocalizer reloc(rcfg(T_BS));
  reloc.addSubMap(sc.submap);
  const RelocResult r = reloc.relocalize(sc.query);

  ASSERT_TRUE(r.found);
  EXPECT_EQ(r.submap_id, 7u);
  EXPECT_GE(r.num_inliers, 28);  // ~all of 30 (noise-free)
  expectSE3Near(r.T_query_match, sc.expected_T_query_match, 1e-4);
}

TEST(XFeatRelocalizer, RecoversPoseNonIdentityExtrinsic) {
  const SE3 T_BS = makeSE3(0.02, -0.06, 0.01, 0.37, {0.2, -0.5, 0.84});  // a real T_BS
  const SE3 T_sl_cam = makeSE3(-0.4, 0.6, -0.2, 0.5, {0.1, 0.2, 1.0});
  const Scene sc = buildScene(T_sl_cam, T_BS, 30, 22);

  XFeatRelocalizer reloc(rcfg(T_BS));
  reloc.addSubMap(sc.submap);
  const RelocResult r = reloc.relocalize(sc.query);

  ASSERT_TRUE(r.found);
  // The extrinsic must round-trip: recovered body pose == T_sl_cam · T_BS⁻¹.
  expectSE3Near(r.T_query_match, sc.expected_T_query_match, 1e-4);
}

TEST(XFeatRelocalizer, RansacRejectsOutlierCorrespondences) {
  const SE3 T_BS;
  const SE3 T_sl_cam = makeSE3(0.3, 0.1, -0.1, 0.2, {0.0, 0.0, 1.0});
  Scene sc = buildScene(T_sl_cam, T_BS, 30, 33);
  // Corrupt 10 query pixels → geometric outliers (descriptor still matches, but
  // the 2D-3D pair is wrong). RANSAC must find the 20-inlier consensus.
  for (int i = 0; i < 10; ++i) {
    sc.query.keypoints(i, 0) += 150.0f;
    sc.query.keypoints(i, 1) -= 120.0f;
  }
  XFeatRelocalizer reloc(rcfg(T_BS));
  reloc.addSubMap(sc.submap);
  const RelocResult r = reloc.relocalize(sc.query);

  ASSERT_TRUE(r.found);
  EXPECT_GE(r.num_inliers, 18);          // the ~20 good correspondences
  EXPECT_LT(r.num_inliers, 30);          // the 10 outliers are NOT inliers
  expectSE3Near(r.T_query_match, sc.expected_T_query_match, 1e-4);  // pose still exact
}

TEST(XFeatRelocalizer, NoMatchReturnsNotFound) {
  const SE3 T_BS;
  const SE3 T_sl_cam = makeSE3(0.3, 0.1, -0.1, 0.2, {0, 0, 1});
  const Scene sc = buildScene(T_sl_cam, T_BS, 30, 44);
  XFeatRelocalizer reloc(rcfg(T_BS));
  reloc.addSubMap(sc.submap);

  // A query whose descriptors are unrelated random → the ratio test rejects them
  // (all NN distances similar) → too few correspondences → found = false.
  Features bogus;
  bogus.keypoints.resize(30, 3);
  bogus.descriptors.resize(30, 64);
  std::mt19937 rng(999);
  std::normal_distribution<float> nd(0.f, 1.f);
  for (int i = 0; i < 30; ++i) {
    Eigen::Matrix<float, 1, 64> d;
    for (int j = 0; j < 64; ++j) d(0, j) = nd(rng);
    d.normalize();
    bogus.descriptors.row(i) = d;
    bogus.keypoints(i, 0) = 100.f + i; bogus.keypoints(i, 1) = 100.f; bogus.keypoints(i, 2) = 1.f;
  }
  EXPECT_FALSE(reloc.relocalize(bogus).found);

  // Too few landmarks (below min_inliers) also → not found, even with exact match.
  const Scene tiny = buildScene(T_sl_cam, T_BS, 8, 55);
  XFeatRelocalizer reloc2(rcfg(T_BS, /*min_inliers=*/15));
  reloc2.addSubMap(tiny.submap);
  EXPECT_FALSE(reloc2.relocalize(tiny.query).found);
}

namespace {

// Attach a per-KF VPR descriptor to `submap` (one KF entry that carries `vpr`). The
// relocalizer ranks by max_k cosine(query.global_descriptor, kf_global_desc[k]); a
// single matching KF inside an otherwise-VPR-less submap is enough for top-N.
void attachPerKfVpr(SubMap& submap, const Eigen::VectorXf& vpr) {
  if (submap.keyframes.empty()) {
    slamko::KeyframePose kf;
    kf.id = 0;
    kf.timestamp = 0.0;
    submap.keyframes.push_back(kf);
  }
  submap.kf_obs.resize(submap.keyframes.size());
  submap.kf_obs[0].global_descriptor = vpr;
}

Eigen::VectorXf unitVec(int dim, unsigned seed) {
  Eigen::VectorXf v(dim);
  std::mt19937 rng(seed);
  std::normal_distribution<float> nd(0.f, 1.f);
  for (int i = 0; i < dim; ++i) v[i] = nd(rng);
  v.normalize();
  return v;
}

}  // namespace

// Per-KF VPR ranking is the candidate stage: the right submap must surface in the top-N
// even when other submaps are present in the DB. With `vpr_top_n=1` only the top-ranked
// submap is PnP-verified — if ranking is wrong, geometry doesn't match the distractor's
// landmarks and r.found=false. This is the magistrale-return regression guard: a single
// KF inside a 10-m submap carries the place signal that a per-submap aggregate loses.
TEST(XFeatRelocalizer, VprPerKfTopNRanking) {
  const SE3 T_BS;
  const SE3 T_sl_cam = makeSE3(0.35, -0.1, 0.05, 0.3, {0.0, 1.0, 0.0});

  // Geometrically valid scene → id 7. Two distractors → ids 8, 9 (different scenes,
  // unrelated VPR vectors).
  Scene match    = buildScene(T_sl_cam, T_BS, /*n=*/30, /*seed=*/61);
  Scene distA    = buildScene(T_sl_cam, T_BS, /*n=*/30, /*seed=*/62);
  Scene distB    = buildScene(T_sl_cam, T_BS, /*n=*/30, /*seed=*/63);
  distA.submap.id = 8;
  distB.submap.id = 9;

  // VPR setup: the "match" submap's one KF carries a vector that equals the query's
  // global descriptor (cosine = 1.0). Distractor KFs carry unit-random unrelated
  // vectors (cosine ≈ 0 in expectation). With top_n=1 the right one MUST surface.
  const Eigen::VectorXf place_vec = unitVec(/*dim=*/8, /*seed=*/100);
  attachPerKfVpr(match.submap, place_vec);
  attachPerKfVpr(distA.submap, unitVec(8, 101));
  attachPerKfVpr(distB.submap, unitVec(8, 102));
  match.query.global_descriptor = place_vec;  // exact-match query

  XFeatRelocConfig c = rcfg(T_BS);
  c.use_vpr = true;
  c.vpr_top_n = 1;        // single candidate → top-1 ranking is the only chance
  c.use_bow = false;      // disable BoW so we can isolate the VPR ranker path
  XFeatRelocalizer reloc(c);
  // Register the distractors FIRST so the correct id has to win on score, not order.
  reloc.addSubMap(distA.submap);
  reloc.addSubMap(distB.submap);
  reloc.addSubMap(match.submap);

  const RelocResult r = reloc.relocalize(match.query);
  ASSERT_TRUE(r.found);
  EXPECT_EQ(r.submap_id, match.submap.id) << "VPR per-KF ranking picked wrong submap";
}

// Legacy fallback: when the SubMap-level `global_descriptor` is set but per-KF is
// absent (SMP3 archive), the ranker still works (cosine vs the per-submap aggregate).
// Guards back-compat — older Atlases on disk must keep relocalizing after the SMP4 bump.
TEST(XFeatRelocalizer, VprPerSubmapFallback) {
  const SE3 T_BS;
  const SE3 T_sl_cam = makeSE3(0.2, 0.05, -0.1, 0.15, {0.0, 0.0, 1.0});

  Scene match = buildScene(T_sl_cam, T_BS, 30, 71);
  Scene distA = buildScene(T_sl_cam, T_BS, 30, 72);
  distA.submap.id = 22;

  const Eigen::VectorXf place_vec = unitVec(8, 200);
  // Per-submap descriptors only (SMP3-style). NO kf_obs / kf_global_desc on either.
  match.submap.global_descriptor = place_vec;
  distA.submap.global_descriptor = unitVec(8, 201);
  match.query.global_descriptor = place_vec;

  XFeatRelocConfig c = rcfg(T_BS);
  c.use_vpr = true;
  c.vpr_top_n = 1;
  c.use_bow = false;
  XFeatRelocalizer reloc(c);
  reloc.addSubMap(distA.submap);
  reloc.addSubMap(match.submap);

  const RelocResult r = reloc.relocalize(match.query);
  ASSERT_TRUE(r.found);
  EXPECT_EQ(r.submap_id, match.submap.id) << "VPR per-submap fallback ranked wrong";
}
