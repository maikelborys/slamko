// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// XFeatRelocalizer — see header. Pure Eigen + the core P3P; no OpenCV.

#include "slamko_loop/xfeat_relocalizer.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
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
                      bool mutual,
                      std::vector<Eigen::Vector2d>& uv_out,
                      std::vector<Eigen::Vector3d>& X_out) {
  uv_out.clear();
  X_out.clear();
  if (!q.hasDescriptors() || sdesc.rows() < 2 ||
      q.descriptorDim() != static_cast<int>(sdesc.cols()))
    return;
  // Optional cross-check: precompute each submap descriptor's nearest QUERY index, so
  // a match (i→best_j) is kept only if best_j's nearest query is also i (symmetric).
  // Removes the ambiguity penalty that self-similar descriptors impose on the one-sided
  // ratio test, without flooding RANSAC. O(M·Q) extra — fine at the subsampled DB size.
  std::vector<int> sj_best_qi;
  if (mutual) {
    sj_best_qi.assign(sdesc.rows(), -1);
    for (int j = 0; j < sdesc.rows(); ++j) {
      const auto sd = sdesc.row(j);
      float best = 1e30f; int bi = -1;
      for (int i = 0; i < q.size(); ++i) {
        const float d2 = (q.descriptors.row(i) - sd).squaredNorm();
        if (d2 < best) { best = d2; bi = i; }
      }
      sj_best_qi[j] = bi;
    }
  }
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
      if (mutual && sj_best_qi[best_j] != i) continue;   // not reciprocal
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
  e.global_desc = submap.global_descriptor;  // VPR retrieval vector (empty if none)
  e.keyframes = submap.keyframes;             // LighterGlue train-view projection poses
  int m = 0;
  for (const auto& lm : submap.landmarks)
    if (lm.descriptor_row >= 0 && lm.descriptor_row < submap.descriptors.rows()) ++m;
  // Stride-subsample if the map is larger than the brute-force budget.
  const int stride = (cfg_.max_db_landmarks > 0 && m > cfg_.max_db_landmarks)
                         ? (m + cfg_.max_db_landmarks - 1) / cfg_.max_db_landmarks
                         : 1;
  e.desc.resize((m + stride - 1) / stride, submap.descriptors.cols());
  e.pos.reserve(e.desc.rows());
  int valid = 0, row = 0;
  for (const auto& lm : submap.landmarks) {
    if (lm.descriptor_row < 0 || lm.descriptor_row >= submap.descriptors.rows()) continue;
    if ((valid++ % stride) != 0) continue;       // keep every stride-th
    if (row >= e.desc.rows()) break;
    e.desc.row(row++) = submap.descriptors.row(lm.descriptor_row);
    e.pos.push_back(lm.position);
  }
  e.desc.conservativeResize(row, submap.descriptors.cols());
  db_.push_back(std::move(e));

  // P3: BoW index. Train the vocabulary once (on the first submap that has enough
  // descriptors — for the never-lost case that's the prior map / first sealed room,
  // which is representative), then BoW-index every submap behind the inverted index.
  if (cfg_.use_bow && submap.descriptors.rows() > 0) {
    if (vocab_.empty() && submap.descriptors.rows() >= cfg_.bow_vocab_size) {
      const int n = static_cast<int>(submap.descriptors.rows());
      const int stride =
          std::max(1, (n + cfg_.bow_train_sample - 1) / cfg_.bow_train_sample);
      DescriptorBlock sample((n + stride - 1) / stride, submap.descriptors.cols());
      int r = 0;
      for (int i = 0; i < n && r < sample.rows(); i += stride)
        sample.row(r++) = submap.descriptors.row(i);
      sample.conservativeResize(r, submap.descriptors.cols());
      vocab_.build(sample, cfg_.bow_vocab_size, 10, cfg_.seed);
    }
    if (!vocab_.empty()) bow_db_.addSubMap(submap.id, vocab_.transform(submap.descriptors));
  }
}

RelocResult XFeatRelocalizer::relocalize(const Features& query) const {
  RelocResult best;  // found = false
  int best_inliers = 0;

  // Candidate pre-selection: PnP-verify only the most promising submaps. Empty =
  // fall back to ALL submaps, so recall is never reduced, only hopeless submaps skipped.
  std::vector<std::uint64_t> cand;

  // VPR (global-descriptor) retrieval — the primary stage. Rank submaps by cosine of the
  // query's global descriptor against each submap's (both L2-normalized → dot = cosine),
  // keep the top-N. This is what actually recognizes a revisited place (XFeat local
  // descriptors cannot — proven). See docs/PLAN_VPR_RELOC.md.
  if (cfg_.use_vpr && query.hasGlobalDescriptor()) {
    const int qd = static_cast<int>(query.global_descriptor.size());
    std::vector<std::pair<float, std::uint64_t>> scored;
    scored.reserve(db_.size());
    for (const auto& e : db_)
      if (e.global_desc.size() == qd)
        scored.emplace_back(query.global_descriptor.dot(e.global_desc), e.id);
    if (!scored.empty()) {
      const int n = std::min<int>(cfg_.vpr_top_n, static_cast<int>(scored.size()));
      std::partial_sort(scored.begin(), scored.begin() + n, scored.end(),
                        [](const auto& a, const auto& b) { return a.first > b.first; });
      for (int i = 0; i < n; ++i) cand.push_back(scored[i].second);
    }
  }

  // BoW fallback (legacy) only if VPR produced no candidates (no global descriptors).
  if (cand.empty() && cfg_.use_bow && !vocab_.empty() && query.hasDescriptors())
    cand = bow_db_.query(vocab_.transform(query.descriptors), cfg_.bow_top_k);

  for (const auto& e : db_) {
    if (!cand.empty() &&
        std::find(cand.begin(), cand.end(), e.id) == cand.end())
      continue;  // not a BoW candidate this query

    SE3 T_sl_cam;
    int inliers = 0;
    int putative = 0;  // # correspondences fed to PnP (for the confidence ratio)

    // Brute-force NN verify FIRST. When it works (easy revisit, small viewpoint gap) it
    // matches the query against the whole landmark cloud → hundreds of correspondences →
    // a well-constrained PnP, MORE accurate than LighterGlue's sparse synthetic-view
    // matches. So it stays the default; LighterGlue is the RESCUE below.
    {
      std::vector<Eigen::Vector2d> uv;
      std::vector<Eigen::Vector3d> X;
      matchDescriptors(query, e.desc, e.pos, cfg_.match_ratio, cfg_.mutual_check, uv, X);
      if (static_cast<int>(X.size()) >= cfg_.min_inliers &&
          pnpRansac(X, uv, cfg_, T_sl_cam, inliers) &&
          // Precision gate (OKVIS-style): inlier RATIO separates a true place from a
          // coincidental match once the Lowe ratio is permissive.
          !(cfg_.min_inlier_ratio > 0.0 &&
            inliers < cfg_.min_inlier_ratio * static_cast<double>(X.size())))
        putative = static_cast<int>(X.size());
      else
        inliers = 0;  // brute-force could not verify this candidate
    }

    // LighterGlue RESCUE: only when brute-force failed this candidate — the hard revisit
    // (large viewpoint/time gap) where XFeat-NN gives <1% inliers but the learned matcher
    // can still find the geometry. Never overrides a good brute-force weld, so the build
    // is guaranteed ≥ the brute-force baseline (it only ADDS closures NN missed).
    if (inliers == 0 && lg_) {
      int lg_inl = 0, lg_put = 0;
      SE3 lg_T;
      if (lightGlueVerify(query, e, lg_T, lg_inl, lg_put)) {
        T_sl_cam = lg_T;
        inliers = lg_inl;
        putative = lg_put;
      }
    }
    if (inliers == 0) continue;          // neither path verified this candidate
    if (inliers <= best_inliers) continue;

    best_inliers = inliers;
    best.found = true;
    best.submap_id = e.id;
    // Camera→body: body pose in sealed-local = (cam in sl) · (body in cam).
    // body in cam = body_T_cam⁻¹ (body_T_cam == T_BS, cam→body).
    best.T_query_match = T_sl_cam * cfg_.body_T_cam.inverse();
    best.confidence = static_cast<double>(inliers) /
                      static_cast<double>(std::max<int>(1, putative));
    best.num_inliers = inliers;
  }
  return best;
}

bool XFeatRelocalizer::lightGlueVerify(const Features& query, const Entry& e,
                                       SE3& T_sl_cam_out, int& inliers_out,
                                       int& putative_out) const {
  if (!lg_ || e.keyframes.empty() || e.pos.empty() || !query.hasDescriptors())
    return false;
  const int M = static_cast<int>(e.pos.size());
  // The submap stores no 2D keypoints, so a train view is the landmark cloud
  // projected into one keyframe's camera. Image bounds come from the principal
  // point (rectified pinhole: cx≈w/2, cy≈h/2) so we keep only landmarks that
  // land in a real image (the model's positional encoding assumes in-image px).
  const double img_w = 2.0 * cfg_.cx;
  const double img_h = 2.0 * cfg_.cy;
  const int nkf = static_cast<int>(e.keyframes.size());
  const int views = std::min(cfg_.lg_max_views, nkf);

  int best_inl = 0, best_putative = 0;
  SE3 best_T;
  bool have = false;
  for (int v = 0; v < views; ++v) {
    // Evenly-spaced keyframes so the views together cover the whole submap span.
    const int kfi = (views == 1) ? nkf / 2 : (v * (nkf - 1)) / (views - 1);
    const SE3& T_WB = e.keyframes[kfi].T_WB;                  // body-in-world (local)
    const SE3 T_cam_sl = (T_WB * cfg_.body_T_cam).inverse();  // world(local)→cam

    Features train;
    train.keypoints.resize(M, 3);
    train.descriptors.resize(M, e.desc.cols());
    std::vector<Eigen::Vector3d> pos_view;
    pos_view.reserve(M);
    int row = 0;
    for (int k = 0; k < M; ++k) {
      const Eigen::Vector3d Xc = T_cam_sl * e.pos[k];
      if (Xc.z() < 1e-3) continue;                            // behind the camera
      const double u = cfg_.fx * Xc.x() / Xc.z() + cfg_.cx;
      const double w = cfg_.fy * Xc.y() / Xc.z() + cfg_.cy;
      if (u < 0 || u >= img_w || w < 0 || w >= img_h) continue;  // out of FOV
      train.keypoints(row, 0) = static_cast<float>(u);
      train.keypoints(row, 1) = static_cast<float>(w);
      train.keypoints(row, 2) = static_cast<float>(1.0 / (1.0 + Xc.z()));  // near→high
      train.descriptors.row(row) = e.desc.row(k);
      pos_view.push_back(e.pos[k]);
      ++row;
    }
    if (row < cfg_.lg_min_view_landmarks) continue;
    train.keypoints.conservativeResize(row, 3);
    train.descriptors.conservativeResize(row, e.desc.cols());

    const auto matches = lg_->match(query, train);
    if (std::getenv("SLAMKO_LG_DEBUG"))
      std::fprintf(stderr, "[LG] submap %llu view %d/%d: %d train-lm, %zu matches\n",
                   (unsigned long long)e.id, v, views, row, matches.size());
    if (static_cast<int>(matches.size()) < cfg_.min_inliers) continue;
    std::vector<Eigen::Vector2d> uv;
    std::vector<Eigen::Vector3d> X;
    uv.reserve(matches.size());
    X.reserve(matches.size());
    for (const auto& m : matches) {
      uv.emplace_back(query.keypoints(m.query_idx, 0), query.keypoints(m.query_idx, 1));
      X.push_back(pos_view[m.train_idx]);
    }
    SE3 T_sl_cam;
    int inl = 0;
    if (!pnpRansac(X, uv, cfg_, T_sl_cam, inl)) continue;
    if (cfg_.min_inlier_ratio > 0.0 &&
        inl < cfg_.min_inlier_ratio * static_cast<double>(X.size())) continue;
    if (inl > best_inl) {
      best_inl = inl;
      best_putative = static_cast<int>(matches.size());
      best_T = T_sl_cam;
      have = true;
    }
  }
  if (!have) return false;
  if (std::getenv("SLAMKO_LG_DEBUG"))
    std::fprintf(stderr, "[LG] VERIFIED submap %llu: %d inliers / %d matches\n",
                 (unsigned long long)e.id, best_inl, best_putative);
  T_sl_cam_out = best_T;
  inliers_out = best_inl;
  putative_out = best_putative;
  return true;
}

}  // namespace slamko
