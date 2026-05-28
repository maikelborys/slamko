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
  e.keyframes = submap.keyframes;             // submap-local KF poses
  // Real per-KF observations + a lid → (descriptor_row, 3D) index for the LightGlue
  // verifier (lightGlueVerify). The descriptor block is COPIED unsubsampled so the
  // index resolves directly into `e.full_desc.row(drow)`.
  e.kf_obs = submap.kf_obs;
  // Per-KF VPR descriptors (SMP4). Cache aligned 1:1 with `keyframes`; ranker scores
  // by max_k cosine(query, kf_global_desc[k]) (see relocalize() below). Empty rows
  // when SMP4 data is absent are skipped — the legacy SubMap-level `global_desc` then
  // carries the per-submap fallback so older Atlases keep working.
  e.kf_global_desc.reserve(submap.kf_obs.size());
  for (const auto& ko : submap.kf_obs) e.kf_global_desc.push_back(ko.global_descriptor);
  e.full_desc = submap.descriptors;
  for (const auto& lm : submap.landmarks)
    if (lm.descriptor_row >= 0 && lm.descriptor_row < submap.descriptors.rows())
      e.lid_to_desc_pos[lm.id] = {lm.descriptor_row, lm.position};
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
  // query's global descriptor against each submap's per-KF descriptors (both L2-norm →
  // dot = cosine), score = max_k cos(query, kf_global_desc[k]). This is the granularity
  // fix: a per-submap aggregate averages over 10 m of trajectory and loses the start-
  // room signal on a long revisit (magistrale1, offline data showed Recall@5=1.0 only at
  // per-KF granularity); per-KF lets the right KF inside a submap surface. Falls back to
  // the SubMap-level `global_desc` when per-KF is absent (SMP3 / legacy Atlas). See
  // docs/PLAN_BA_GLOBAL.md "VPR retrieval: change granularity first".
  if (cfg_.use_vpr && query.hasGlobalDescriptor()) {
    const int qd = static_cast<int>(query.global_descriptor.size());
    std::vector<std::pair<float, std::uint64_t>> scored;
    scored.reserve(db_.size());
    for (const auto& e : db_) {
      float best = -2.f;  // cosine in [-1,1]; -2 means "no scoring descriptor present"
      // Per-KF max — the discriminative path.
      for (const auto& g : e.kf_global_desc)
        if (g.size() == qd) {
          const float c = query.global_descriptor.dot(g);
          if (c > best) best = c;
        }
      // Per-submap fallback (legacy / SMP3 maps without per-KF VPR).
      if (best == -2.f && e.global_desc.size() == qd)
        best = query.global_descriptor.dot(e.global_desc);
      if (best > -2.f) scored.emplace_back(best, e.id);
    }
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
  if (!lg_ || e.keyframes.empty() || !query.hasDescriptors()) return false;
  // REAL per-keyframe matching (in-distribution for LightGlue, replaces the v1
  // synthetic projected-cloud train view). For each candidate keyframe we build a
  // train Features from its STORED `kf_obs` (the actual 2D pixel locations recorded
  // when the KF was inserted) + descriptors looked up by `landmark_id`. LightGlue
  // matches two real images → dense correspondences even on hard revisits, where
  // the v1 sparse-projected-cloud train view failed (e.g. magistrale1 start-room
  // return). Falls back gracefully when the submap predates Phase A (kf_obs empty).
  if (e.kf_obs.empty() || e.full_desc.rows() == 0) return false;
  const int desc_dim = static_cast<int>(e.full_desc.cols());
  const int nkf = static_cast<int>(e.keyframes.size());
  const int n_obs_kf = static_cast<int>(e.kf_obs.size());
  if (n_obs_kf == 0) return false;
  const int views = std::min(cfg_.lg_max_views, std::min(nkf, n_obs_kf));

  int best_inl = 0, best_putative = 0;
  SE3 best_T;
  bool have = false;
  for (int v = 0; v < views; ++v) {
    // Evenly-spaced KFs across the submap to cover its full span.
    const int kfi = (views == 1) ? n_obs_kf / 2
                                 : (v * (n_obs_kf - 1)) / (views - 1);
    const auto& ko = e.kf_obs[kfi];
    const int N = ko.size();
    if (N < cfg_.lg_min_view_landmarks) continue;

    // Build train Features: real 2D pixel + descriptor (via lid lookup) + parallel
    // 3D pos in submap-local frame for PnP after matching.
    Features train;
    train.keypoints.resize(N, 3);
    train.descriptors.resize(N, desc_dim);
    std::vector<Eigen::Vector3d> pos_view;
    pos_view.reserve(N);
    int row = 0;
    for (int i = 0; i < N; ++i) {
      const auto it = e.lid_to_desc_pos.find(ko.landmark_ids[i]);
      if (it == e.lid_to_desc_pos.end()) continue;  // landmark dropped from descriptor block
      train.keypoints(row, 0) = ko.uv(i, 0);
      train.keypoints(row, 1) = ko.uv(i, 1);
      // Score = 1 (the model uses positional encoding more than score; with all 1.0
      // the top-K selection inside LightGlueMatcher falls back to insertion order).
      train.keypoints(row, 2) = 1.0f;
      train.descriptors.row(row) = e.full_desc.row(it->second.first);
      pos_view.push_back(it->second.second);
      ++row;
    }
    if (row < cfg_.lg_min_view_landmarks) continue;
    train.keypoints.conservativeResize(row, 3);
    train.descriptors.conservativeResize(row, desc_dim);

    const auto matches = lg_->match(query, train);
    if (std::getenv("SLAMKO_LG_DEBUG"))
      std::fprintf(stderr, "[LG] submap %llu view %d/%d (kf %d): %d train-lm, "
                           "%zu matches\n",
                   (unsigned long long)e.id, v, views, kfi, row, matches.size());
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
