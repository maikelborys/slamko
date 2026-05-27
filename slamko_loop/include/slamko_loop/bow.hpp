// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Bag-of-Words place recognition (P3) — the scalability layer for relocalization.
// Brute-force NN match of a query against every submap's descriptors is O(maps ×
// descriptors); it doesn't scale past a room. This adds a classic vocabulary +
// INVERTED INDEX so a query first retrieves a handful of CANDIDATE submaps in
// sublinear time, and the expensive geometric verification (PnP-RANSAC) runs only on
// those — the standard ORB-SLAM/DBoW recipe, hand-rolled over XFeat's 64-d float
// descriptors (no DBoW2 dep; Eigen + std only, depends on slamko_core alone).
//
// Two pieces:
//   * BowVocabulary — K visual words = k-means centroids over a descriptor sample.
//     Deterministic (k-means++ seeded init + Lloyd), so results are reproducible.
//   * BowDatabase   — per-submap TF bag-of-words behind an inverted index (word →
//     submaps), scored by TF-IDF cosine. query() returns the top-k candidate submaps.
//
// The vocabulary is trained once (e.g. on the prior map's descriptors); it is NOT
// the verifier — geometry (the XFeat relocalizer's PnP gate) still decides the weld.

#pragma once

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace slamko {

using DescriptorBlock =
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;  // N×D
using BowVector = std::vector<std::pair<int, float>>;  // (word_id, L2-normalized TF)

class BowVocabulary {
 public:
  // Train K words by k-means++ (seeded) + `iters` Lloyd steps over `descriptors`
  // (N×D). Deterministic for a given seed. K is clamped to N.
  void build(const DescriptorBlock& descriptors, int K, int iters = 10,
             unsigned seed = 1);

  int  quantize(const Eigen::Ref<const Eigen::RowVectorXf>& d) const;  // nearest word
  BowVector transform(const DescriptorBlock& descriptors) const;       // L2-norm TF hist

  int  size() const { return static_cast<int>(words_.rows()); }
  bool empty() const { return words_.rows() == 0; }
  const DescriptorBlock& words() const { return words_; }

 private:
  DescriptorBlock words_;  // K×D centroids
};

class BowDatabase {
 public:
  // Register a submap's bag-of-words (keeps the archive submap id).
  void addSubMap(std::uint64_t submap_id, const BowVector& bow);

  // Top-k candidate submap ids for a query BoW, ranked by TF-IDF cosine similarity.
  // Uses the inverted index — only touches submaps that share a word with the query.
  std::vector<std::uint64_t> query(const BowVector& query_bow, int top_k) const;

  std::size_t size() const { return entries_.size(); }
  void clear();

 private:
  struct Entry {
    std::uint64_t id = 0;
    std::unordered_map<int, float> tf;  // word -> normalized TF
  };
  float idf(int word) const;            // log((1+N)/(1+df)) + 1

  std::vector<Entry> entries_;
  std::unordered_map<int, std::vector<int>> inverted_;  // word -> entry indices
};

}  // namespace slamko
