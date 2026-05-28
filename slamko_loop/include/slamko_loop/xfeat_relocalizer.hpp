// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// XFeatRelocalizer — the P2b place-recognition + geometric-verification backend
// (implements slamko_core::Relocalizer). It localizes a query frame against
// archived submaps using the XFeat descriptors the VIO already attaches to
// landmarks (SubMap's N×64 index) — NO new model: brute-force NN match (Lowe
// ratio) → 2D-3D correspondences → PnP-RANSAC (core P3P) → the query CAMERA pose
// in the matched submap's local frame, converted to the BODY frame via the
// cam↔body extrinsic.
//
// It returns RelocResult.T_query_match = query BODY pose in sealed-local
// (T_submaplocal_body). The never-lost supervisor composes that with the live
// odom to get the weld constraint (T_active_sealed = T_query_match · T_WB⁻¹ — the
// OKVIS2-X T_AB formula) and feeds it to the lazy-anchor gate. Keeping the
// extrinsic here (the relocalizer is camera-aware) lets the supervisor stay in
// the body frame. Depends on slamko_core only (Hard Rule #2) — no OpenCV.

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include <memory>
#include <unordered_map>

#include "slamko_core/features.hpp"
#include "slamko_core/relocalizer.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_core/submap.hpp"

#include "slamko_loop/bow.hpp"
#include "slamko_loop/lightglue_matcher.hpp"

namespace slamko {

struct XFeatRelocConfig {
  double fx = 0, fy = 0, cx = 0, cy = 0;  // rectified pinhole intrinsics
  SE3    body_T_cam;                      // T_BS (cam→body); identity = cam ≡ body
  // Lowe second-nearest ratio. XFeat-64 descriptors are weakly discriminative on
  // self-similar indoor scenes (measured mean NN cosine ≈0.89 on TUM VI), so a strict
  // 0.8 ratio passes only ~5 matches on a loop return — below min_inliers, PnP never
  // runs, the loop never closes. 0.9 lifts that to ~100 putative matches; PnP-RANSAC +
  // the supervisor's AnchorGate cluster are the precision defense (OKVIS's recipe:
  // permissive appearance match, strict geometric gate). See the 2026-05-27 recall study.
  float  match_ratio = 0.9f;
  bool   mutual_check = false;            // symmetric (cross-check) NN — extra precision
  double min_inlier_ratio = 0.0;          // reject PnP if inliers/matches < this (0 = off)
  int    ransac_iters = 200;
  double ransac_thresh_px = 3.0;          // reprojection inlier threshold
  int    min_inliers = 15;                // accept a relocalization above this
  unsigned seed = 1u;                     // RANSAC RNG seed (deterministic)
  // Brute-force NN match is O(N_query · N_db · D); the cumulative submap can hold
  // tens of thousands of landmarks. Stride-subsample a registered submap to at
  // most this many descriptors so a relocalize() call stays cheap (a real system
  // would use a vocabulary/inverted index — that's the scalable swap).
  int    max_db_landmarks = 3000;

  // P3: BoW candidate pre-selection. Train a vocabulary on the first registered
  // submap's descriptors, BoW-index every submap behind an inverted index, and at
  // relocalize() PnP-verify only the top-k BoW candidates instead of every submap
  // (sublinear in map count). Falls back to all-submaps if the vocab is untrained or
  // returns no candidate — so it never lowers recall, only skips hopeless submaps.
  // Global-descriptor (VPR, e.g. EigenPlaces 512-D) candidate retrieval — the real
  // recall fix. When the query + submaps carry a `global_descriptor`, rank submaps by
  // cosine similarity and PnP-verify only the top-N. XFeat local descriptors carry no
  // place signal (proven), so this REPLACES BoW as the candidate stage; BoW/all-submap
  // remain the fallback when no global descriptor is present. See docs/PLAN_VPR_RELOC.md.
  bool   use_vpr          = true;
  int    vpr_top_n        = 10;    // candidate submaps PnP-verified per query (by VPR cosine)

  bool   use_bow          = true;
  int    bow_vocab_size   = 256;   // K visual words
  // Candidate submaps PnP-verified per query. A whole-submap BoW vector aggregates
  // over a 30–80 m path stretch, so on a loop return the start submap ranks ~8–9 (not
  // top-5): a small top-k silently drops the true loop. 25 recovers it; the all-submap
  // PnP fallback stays cheap at this map scale. (Per-keyframe BoW is the deeper fix.)
  int    bow_top_k        = 25;
  int    bow_train_sample = 6000;  // max descriptors used to train the vocabulary

  // LighterGlue verification — the viewpoint-robust matcher that REPLACES brute-force
  // NN in the verify stage (VPR retrieves the candidate; this confirms the geometry on
  // a hard revisit that XFeat-NN can't, per the recall study). The submap has no stored
  // 2D keypoints, so a candidate's landmarks are projected into each of up to
  // `lg_max_views` keyframe poses to synthesize a train view; LighterGlue matches the
  // live query against each view → matched 3D feeds the UNCHANGED PnP-RANSAC. Falls back
  // to brute-force NN when the matcher can't load (no libtorch / no model). See
  // docs/PLAN_VPR_RELOC.md. Requires the build with -DSLAMKO_LOOP_WITH_TORCH.
  bool   use_lightglue        = false;
  std::string lightglue_model_path;     // TorchScript lighterglue.pt
  int    lg_trace_n           = 512;    // keypoint count baked into the export
  float  lg_score_thresh      = 0.10f;  // mscores0 cut
  int    lg_max_views         = 4;      // keyframe poses projected per candidate submap
  int    lg_min_view_landmarks = 12;    // skip a view with fewer in-FOV landmarks
};

class XFeatRelocalizer : public Relocalizer {
 public:
  explicit XFeatRelocalizer(const XFeatRelocConfig& cfg) : cfg_(cfg) {
    // Build the LighterGlue verifier once (loads + deserializes the .pt). If it
    // can't (no libtorch / missing model), lg_ is left null and the verify stage
    // uses brute-force NN — never a hard failure, just no learned matching.
    if (cfg_.use_lightglue && !cfg_.lightglue_model_path.empty()) {
      LightGlueConfig lc;
      lc.model_path = cfg_.lightglue_model_path;
      lc.trace_n = cfg_.lg_trace_n;
      lc.score_thresh = cfg_.lg_score_thresh;
      auto lg = std::make_unique<LightGlueMatcher>(lc);
      if (lg->build()) lg_ = std::move(lg);
    }
  }

  std::string name() const override { return "xfeat"; }

  // Register an archived submap: cache its descriptor rows + the matching
  // submap-local 3D positions (only landmarks that carry a descriptor).
  void addSubMap(const SubMap& submap) override;

  // Match query → each submap, PnP-RANSAC verify, return the best (most inliers).
  RelocResult relocalize(const Features& query) const override;

  std::size_t numSubMaps() const { return db_.size(); }

 private:
  struct Entry {
    std::uint64_t id = 0;
    SE3 anchor;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> desc;  // M×D
    std::vector<Eigen::Vector3d> pos;  // M submap-local 3D, aligned with desc rows
    Eigen::VectorXf global_desc;       // VPR descriptor for this submap (empty if none)
    std::vector<KeyframePose> keyframes;  // submap-local poses
    // The per-keyframe 2D observation lists (slamko_core SubMap.kf_obs), aligned 1:1
    // with `keyframes`. The REAL-keyframe LighterGlue verifier (lightGlueVerify) uses
    // these as the train Features (real 2D pixels + descriptors looked up by lid) so
    // LightGlue sees a true two-image match (in-distribution). Empty for legacy maps.
    std::vector<KeyframeObservations> kf_obs;
    // Full (un-subsampled) descriptor block + landmark-id index so the LightGlue verify
    // can resolve `kf_obs[k].landmark_ids[i] → descriptor + 3D pos` in O(1). The
    // brute-force NN path keeps the existing subsampled `desc`/`pos` for speed.
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> full_desc;
    std::unordered_map<std::uint64_t, std::pair<int, Eigen::Vector3d>> lid_to_desc_pos;
  };

  // LighterGlue verify of one candidate submap: project its landmarks into up to
  // lg_max_views keyframe poses to synthesize train views, match the query against
  // each, run PnP-RANSAC on the best, return inliers (0 on no usable result).
  bool lightGlueVerify(const Features& query, const Entry& e, SE3& T_sl_cam_out,
                       int& inliers_out, int& putative_out) const;

  XFeatRelocConfig    cfg_;
  std::vector<Entry>  db_;
  BowVocabulary       vocab_;   // P3: trained on the first submap, fixed thereafter
  BowDatabase         bow_db_;  // P3: inverted index for candidate pre-selection
  mutable std::unique_ptr<LightGlueMatcher> lg_;  // null = brute-force fallback
};

}  // namespace slamko
