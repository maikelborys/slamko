// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// XFeatRelocalizer — see header. Pure Eigen + the core P3P; no OpenCV.

#include "slamko_loop/xfeat_relocalizer.hpp"

#include <random>

#include "slamko_core/p3p_solver.hpp"

namespace slamko {

namespace {

// Brute-force NN match query→submap descriptors with a Lowe ratio test. Outputs
// aligned 2D (query pixel) ↔ 3D (submap-local) correspondences.
void matchDescriptors(const Features& q,
                      const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
                                          Eigen::RowMajor>& sdesc,
                      const std::vector<Eigen::Vector3d>& spos, float ratio,
                      std::vector<Eigen::Vector2d>& uv_out,
                      std::vector<Eigen::Vector3d>& X_out) {
  uv_out.clear();
  X_out.clear();
  if (!q.hasDescriptors() || sdesc.rows() < 2 ||
      q.descriptorDim() != static_cast<int>(sdesc.cols()))
    return;
  for (int i = 0; i < q.size(); ++i) {
    const auto qd = q.descriptors.row(i);
    float best = 1e30f, second = 1e30f;
    int best_j = -1;
    for (int j = 0; j < sdesc.rows(); ++j) {
      const float d2 = (qd - sdesc.row(j)).squaredNorm();
      if (d2 < best) { second = best; best = d2; best_j = j; }
      else if (d2 < second) { second = d2; }
    }
    if (best_j >= 0 && best < ratio * ratio * second) {  // ratio on squared dist
      uv_out.emplace_back(q.keypoints(i, 0), q.keypoints(i, 1));
      X_out.push_back(spos[best_j]);
    }
  }
}

// Pixel → unit bearing (camera frame): xn=(u-cx)/fx, yn=(v-cy)/fy, normalize.
void bearing(double u, double v, double fx, double fy, double cx, double cy,
             double f[3]) {
  const double xn = (u - cx) / fx, yn = (v - cy) / fy;
  const double nr = std::sqrt(xn * xn + yn * yn + 1.0);
  f[0] = xn / nr; f[1] = yn / nr; f[2] = 1.0 / nr;
}

// PnP-RANSAC over (X submap-local 3D, uv pixel). On success outputs the query
// CAMERA pose in submap-local (T_sl_cam) + the inlier count. Mirrors pnp_cuda.cu:
// sample 3 → P3P → reproject all → count inliers → argmax.
bool pnpRansac(const std::vector<Eigen::Vector3d>& X,
               const std::vector<Eigen::Vector2d>& uv, const XFeatRelocConfig& cfg,
               SE3& T_sl_cam_out, int& inliers_out) {
  const int N = static_cast<int>(X.size());
  if (N < 4) return false;
  const double thr2 = cfg.ransac_thresh_px * cfg.ransac_thresh_px;

  std::mt19937 rng(cfg.seed);
  std::uniform_int_distribution<int> pick(0, N - 1);

  int best_inliers = 0;
  Eigen::Matrix3d bestR = Eigen::Matrix3d::Identity();
  Eigen::Vector3d bestt = Eigen::Vector3d::Zero();
  bool have = false;

  for (int it = 0; it < cfg.ransac_iters; ++it) {
    int i0 = pick(rng), i1, i2;
    do { i1 = pick(rng); } while (i1 == i0);
    do { i2 = pick(rng); } while (i2 == i0 || i2 == i1);
    const int idx[3] = {i0, i1, i2};

    double Pw[3][3], fb[3][3];
    for (int k = 0; k < 3; ++k) {
      Pw[k][0] = X[idx[k]].x(); Pw[k][1] = X[idx[k]].y(); Pw[k][2] = X[idx[k]].z();
      bearing(uv[idx[k]].x(), uv[idx[k]].y(), cfg.fx, cfg.fy, cfg.cx, cfg.cy, fb[k]);
    }
    double Rs[4][9], ts[4][3];
    const int ns = p3p::solve(Pw, fb, Rs, ts);
    for (int s = 0; s < ns; ++s) {
      Eigen::Matrix3d R;
      for (int r = 0; r < 3; ++r)
        for (int cc = 0; cc < 3; ++cc) R(r, cc) = Rs[s][r * 3 + cc];
      const Eigen::Vector3d t(ts[s][0], ts[s][1], ts[s][2]);
      int inl = 0;
      for (int j = 0; j < N; ++j) {
        const Eigen::Vector3d Xc = R * X[j] + t;  // world(sl)→cam
        if (Xc.z() < 1e-3) continue;
        const double up = cfg.fx * Xc.x() / Xc.z() + cfg.cx;
        const double vp = cfg.fy * Xc.y() / Xc.z() + cfg.cy;
        const double du = up - uv[j].x(), dv = vp - uv[j].y();
        if (du * du + dv * dv <= thr2) ++inl;
      }
      if (inl > best_inliers) { best_inliers = inl; bestR = R; bestt = t; have = true; }
    }
  }

  if (!have || best_inliers < cfg.min_inliers) return false;
  // (R,t) maps sl→cam; the camera pose IN sl is the inverse.
  T_sl_cam_out = SE3(bestR, bestt).inverse();
  inliers_out = best_inliers;
  return true;
}

}  // namespace

void XFeatRelocalizer::addSubMap(const SubMap& submap) {
  Entry e;
  e.id = submap.id;
  e.anchor = submap.anchor;
  int m = 0;
  for (const auto& lm : submap.landmarks)
    if (lm.descriptor_row >= 0 && lm.descriptor_row < submap.descriptors.rows()) ++m;
  e.desc.resize(m, submap.descriptors.cols());
  e.pos.reserve(m);
  int row = 0;
  for (const auto& lm : submap.landmarks) {
    if (lm.descriptor_row < 0 || lm.descriptor_row >= submap.descriptors.rows()) continue;
    e.desc.row(row++) = submap.descriptors.row(lm.descriptor_row);
    e.pos.push_back(lm.position);
  }
  db_.push_back(std::move(e));
}

RelocResult XFeatRelocalizer::relocalize(const Features& query) const {
  RelocResult best;  // found = false
  int best_inliers = 0;
  for (const auto& e : db_) {
    std::vector<Eigen::Vector2d> uv;
    std::vector<Eigen::Vector3d> X;
    matchDescriptors(query, e.desc, e.pos, cfg_.match_ratio, uv, X);
    if (static_cast<int>(X.size()) < cfg_.min_inliers) continue;

    SE3 T_sl_cam;
    int inliers = 0;
    if (!pnpRansac(X, uv, cfg_, T_sl_cam, inliers)) continue;
    if (inliers <= best_inliers) continue;

    best_inliers = inliers;
    best.found = true;
    best.submap_id = e.id;
    // Camera→body: body pose in sealed-local = (cam in sl) · (body in cam).
    // body in cam = body_T_cam⁻¹ (body_T_cam == T_BS, cam→body).
    best.T_query_match = T_sl_cam * cfg_.body_T_cam.inverse();
    best.confidence = static_cast<double>(inliers) /
                      static_cast<double>(std::max<int>(1, (int)X.size()));
    best.num_inliers = inliers;
  }
  return best;
}

}  // namespace slamko
