// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// GtsamGlobalSmoother sanity gate. Synthesizes a controlled scenario (known
// poses + landmarks → projected stereo observations), perturbs the initial
// estimates, runs BA, and asserts convergence back to the truth. This is what
// guarantees the OKVIS-class alignment build (docs/PLAN_BA_GLOBAL.md) actually
// minimizes reprojection error — not a "graph is built" smoke test.

#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "slamko_core/se3.hpp"
#include "slamko_core/stereo_observation.hpp"
#include "slamko_fusion/gtsam_global_smoother.hpp"

using slamko::GlobalBAInput;
using slamko::GlobalBAObservation;
using slamko::SE3;
using slamko::SO3;
using slamko::StereoCalib;
using slamko_fusion::GtsamGlobalSmoother;

namespace {
// Forward-project a world-frame landmark into a body-pose KF's stereo cam.
// Mirrors GTSAM's GenericStereoFactor convention: T_WC = T_WB · T_BS (cam-in-world),
// X_cam = T_WC⁻¹ · X_world, uL = fx·x/z + cx, uR = uL − fx·b/z, v = fy·y/z + cy.
bool project(const SE3& T_WB, const SE3& T_BS, const StereoCalib& K,
             const Eigen::Vector3d& X_world,
             double& uL, double& uR, double& v) {
  const SE3 T_cam_world = (T_WB * T_BS).inverse();
  const Eigen::Vector3d X_c = T_cam_world * X_world;
  if (X_c.z() < 0.2) return false;
  uL = K.fx * X_c.x() / X_c.z() + K.cx;
  v  = K.fy * X_c.y() / X_c.z() + K.cy;
  uR = uL - K.fx * K.baseline / X_c.z();
  return uL > 0 && uL < 2 * K.cx && v > 0 && v < 2 * K.cy && uR > 0;
}
}  // namespace

// Pure visual BA: 5 KFs along x, 30 random landmarks in front; project to make
// observations, perturb initial estimates, optimize — refined poses+landmarks must
// snap back to the truth (anchor KF locked, others within ~1 cm).
TEST(GtsamGlobalSmoother, SyntheticStereoBAConvergesToTruth) {
  StereoCalib K{200.0, 200.0, 320.0, 240.0, 0.10};  // fx fy cx cy baseline
  // Cam-body identity (simplifies the truth bookkeeping; the factor still composes
  // through body_T_cam, so this exercises the path).
  const SE3 T_BS(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  // Truth keyframes.
  std::vector<SE3> truth_kfs;
  for (int i = 0; i < 5; ++i)
    truth_kfs.emplace_back(Eigen::Matrix3d::Identity(),
                           Eigen::Vector3d(0.5 * i, 0.0, 0.0));

  // Truth landmarks in front of the cameras.
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> ux(-0.5, 2.5), uy(-1.0, 1.0), uz(4.0, 6.0);
  std::vector<Eigen::Vector3d> truth_lms;
  for (int i = 0; i < 30; ++i)
    truth_lms.emplace_back(ux(rng), uy(rng), uz(rng));

  // Project to stereo obs (only visible ones).
  std::vector<GlobalBAObservation> obs;
  for (size_t k = 0; k < truth_kfs.size(); ++k) {
    for (size_t l = 0; l < truth_lms.size(); ++l) {
      double uL, uR, v;
      if (!project(truth_kfs[k], T_BS, K, truth_lms[l], uL, uR, v)) continue;
      GlobalBAObservation o;
      o.kf_id = static_cast<std::uint64_t>(k);
      o.landmark_id = static_cast<std::uint64_t>(l);
      o.uv_left  = Eigen::Vector2f(static_cast<float>(uL), static_cast<float>(v));
      o.uv_right = Eigen::Vector2f(static_cast<float>(uR), static_cast<float>(v));
      obs.push_back(o);
    }
  }
  ASSERT_GT(obs.size(), 100u) << "synthetic scene produced too few visible obs";

  // Perturbed initial estimates (the anchor KF stays at truth via tight prior; the
  // others get translation noise; landmarks get 3D noise).
  std::normal_distribution<double> nt(0.0, 0.05), nl(0.0, 0.10), nr(0.0, 0.01);
  GlobalBAInput in;
  in.calib = K; in.T_BS = T_BS; in.anchor_kf = 0; in.pixel_sigma = 1.0; in.max_iters = 50;
  for (size_t k = 0; k < truth_kfs.size(); ++k) {
    Eigen::Vector3d t = truth_kfs[k].translation();
    if (k != 0) t += Eigen::Vector3d(nt(rng), nt(rng), nt(rng));
    const Eigen::Vector3d w(nr(rng), nr(rng), nr(rng));
    const Eigen::Matrix3d R = (k == 0) ? Eigen::Matrix3d::Identity()
                                       : SO3::exp(w).matrix();
    in.keyframes.emplace_back(static_cast<std::uint64_t>(k), SE3(R, t));
  }
  for (size_t l = 0; l < truth_lms.size(); ++l)
    in.landmarks.emplace_back(static_cast<std::uint64_t>(l),
        truth_lms[l] + Eigen::Vector3d(nl(rng), nl(rng), nl(rng)));
  in.observations = obs;

  GtsamGlobalSmoother smoother;
  const auto out = smoother.optimize(in);

  EXPECT_TRUE(out.converged) << "no cost improvement (initial=" << out.initial_cost
                             << " final=" << out.final_cost << ")";
  EXPECT_LT(out.final_cost, out.initial_cost * 0.01)
      << "BA didn't drive cost ≥100× down — likely a factor wiring bug";
  EXPECT_GT(out.iterations, 0);

  // Refined poses must snap back to within ~1 cm (anchor is pinned to truth).
  for (size_t k = 0; k < truth_kfs.size(); ++k) {
    const double dt = (out.keyframes[k].second.translation() -
                       truth_kfs[k].translation()).norm();
    EXPECT_LT(dt, 0.01) << "kf " << k << " translation err " << dt << " m";
  }
  // Refined landmarks within ~5 cm.
  double max_lm = 0.0;
  for (size_t l = 0; l < truth_lms.size(); ++l)
    max_lm = std::max(max_lm,
        (out.landmarks[l].second - truth_lms[l]).norm());
  EXPECT_LT(max_lm, 0.05) << "max landmark error " << max_lm << " m";
}

// Loop-closure BetweenFactor: two disjoint scenes (no shared landmarks) need the
// BetweenFactor to bridge them; without it the second submap's pose is unconstrained
// relative to the first and ATE blows up. With the factor, both halves align.
TEST(GtsamGlobalSmoother, LoopClosureBetweenFactorBridgesDisjointSets) {
  StereoCalib K{200.0, 200.0, 320.0, 240.0, 0.10};
  const SE3 T_BS(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  // Two "submaps": KFs {0,1,2} see landmarks {0..14}, KFs {10,11,12} see {15..29}.
  // KFs 2 and 10 are the loop pair (a revisit), constrained by the BetweenFactor.
  std::vector<SE3> truth_kfs(13);
  truth_kfs[0]  = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0, 0));
  truth_kfs[1]  = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.5, 0, 0));
  truth_kfs[2]  = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0, 0));
  truth_kfs[10] = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.05, 0, 0));  // revisit
  truth_kfs[11] = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.55, 0, 0));
  truth_kfs[12] = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.05, 0, 0));

  std::mt19937 rng(7);
  std::uniform_real_distribution<double> ux(-0.3, 1.5), uy(-1.0, 1.0), uz(4.0, 6.0);
  std::vector<Eigen::Vector3d> truth_lms(30);
  for (auto& l : truth_lms) l = Eigen::Vector3d(ux(rng), uy(rng), uz(rng));

  std::vector<GlobalBAObservation> obs;
  const std::vector<std::pair<std::vector<int>, std::pair<int, int>>> groups = {
      {{0, 1, 2}, {0, 15}}, {{10, 11, 12}, {15, 30}}};
  for (const auto& g : groups) {
    for (int k : g.first) {
      for (int l = g.second.first; l < g.second.second; ++l) {
        double uL, uR, v;
        if (!project(truth_kfs[k], T_BS, K, truth_lms[l], uL, uR, v)) continue;
        GlobalBAObservation o;
        o.kf_id = static_cast<std::uint64_t>(k);
        o.landmark_id = static_cast<std::uint64_t>(l);
        o.uv_left  = Eigen::Vector2f((float)uL, (float)v);
        o.uv_right = Eigen::Vector2f((float)uR, (float)v);
        obs.push_back(o);
      }
    }
  }

  GlobalBAInput in;
  in.calib = K; in.T_BS = T_BS; in.anchor_kf = 0; in.pixel_sigma = 1.0; in.max_iters = 50;
  for (int k : {0, 1, 2, 10, 11, 12}) {
    SE3 init = truth_kfs[k];
    if (k != 0)  // perturb everything except the anchor
      init = SE3(init.so3(), init.translation() + Eigen::Vector3d(0.05, -0.03, 0.02));
    in.keyframes.emplace_back(static_cast<std::uint64_t>(k), init);
  }
  for (int l = 0; l < 30; ++l)
    in.landmarks.emplace_back(static_cast<std::uint64_t>(l),
        truth_lms[l] + Eigen::Vector3d(0.05, 0.0, -0.05));
  in.observations = obs;

  // Loop closure: KF 2 ≈ KF 10 (revisit). T_2_10 = T_W2⁻¹ · T_W10 (relative body
  // pose, "from 2's frame to 10's").
  in.has_loop     = true;
  in.loop_kf_from = 2;
  in.loop_kf_to   = 10;
  in.T_from_to    = truth_kfs[2].inverse() * truth_kfs[10];
  in.loop_sigma_t = 0.02;
  in.loop_sigma_r = 0.01;

  GtsamGlobalSmoother smoother;
  const auto out = smoother.optimize(in);
  EXPECT_TRUE(out.converged);
  EXPECT_LT(out.final_cost, out.initial_cost * 0.1)
      << "loop closure failed to drive cost down with BetweenFactor";

  // Both halves must align to truth within ~5 cm.
  double max_dt = 0.0;
  for (const auto& [id, T] : out.keyframes)
    max_dt = std::max(max_dt, (T.translation() - truth_kfs[id].translation()).norm());
  EXPECT_LT(max_dt, 0.05) << "max KF trans err " << max_dt << " m";
}
