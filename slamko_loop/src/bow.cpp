// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// BoW place recognition — see bow.hpp. Deterministic k-means++ vocabulary + an
// inverted-index TF-IDF database. Eigen + std only.

#include "slamko_loop/bow.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <random>

namespace slamko {

void BowVocabulary::build(const DescriptorBlock& X, int K, int iters, unsigned seed) {
  const int N = static_cast<int>(X.rows()), D = static_cast<int>(X.cols());
  if (N == 0 || K <= 0) { words_.resize(0, 0); return; }
  K = std::min(K, N);

  std::mt19937 rng(seed);
  // k-means++ seeding: first center uniform, each next ∝ squared distance to the
  // nearest chosen center (deterministic given the seed).
  std::vector<int> centers;
  centers.reserve(K);
  centers.push_back(std::uniform_int_distribution<int>(0, N - 1)(rng));
  std::vector<float> d2(N, std::numeric_limits<float>::max());
  for (int c = 1; c < K; ++c) {
    const int last = centers.back();
    for (int i = 0; i < N; ++i)
      d2[i] = std::min(d2[i], static_cast<float>((X.row(i) - X.row(last)).squaredNorm()));
    double sum = 0.0;
    for (float v : d2) sum += v;
    int pick = N - 1;
    if (sum > 0.0) {
      double r = std::uniform_real_distribution<double>(0.0, sum)(rng);
      for (int i = 0; i < N; ++i) { r -= d2[i]; if (r <= 0.0) { pick = i; break; } }
    } else {
      pick = std::uniform_int_distribution<int>(0, N - 1)(rng);
    }
    centers.push_back(pick);
  }
  words_.resize(K, D);
  for (int c = 0; c < K; ++c) words_.row(c) = X.row(centers[c]);

  // Lloyd iterations.
  std::vector<int> assign(N, 0);
  for (int it = 0; it < iters; ++it) {
    for (int i = 0; i < N; ++i) assign[i] = quantize(X.row(i));
    DescriptorBlock sum = DescriptorBlock::Zero(K, D);
    std::vector<int> cnt(K, 0);
    for (int i = 0; i < N; ++i) { sum.row(assign[i]) += X.row(i); ++cnt[assign[i]]; }
    for (int c = 0; c < K; ++c)
      if (cnt[c] > 0) words_.row(c) = sum.row(c) / static_cast<float>(cnt[c]);
  }
}

int BowVocabulary::quantize(const Eigen::Ref<const Eigen::RowVectorXf>& d) const {
  int best = 0;
  float best_d2 = std::numeric_limits<float>::max();
  for (int c = 0; c < words_.rows(); ++c) {
    const float dist = (words_.row(c) - d).squaredNorm();
    if (dist < best_d2) { best_d2 = dist; best = c; }
  }
  return best;
}

BowVector BowVocabulary::transform(const DescriptorBlock& X) const {
  BowVector out;
  if (empty() || X.rows() == 0) return out;
  std::map<int, float> hist;  // ordered → deterministic output
  for (int i = 0; i < X.rows(); ++i) hist[quantize(X.row(i))] += 1.0f;
  float norm = 0.0f;
  for (const auto& kv : hist) norm += kv.second * kv.second;
  norm = std::sqrt(std::max(norm, 1e-12f));
  out.reserve(hist.size());
  for (const auto& kv : hist) out.emplace_back(kv.first, kv.second / norm);
  return out;
}

// ---------------------------------------------------------------- BowDatabase

void BowDatabase::clear() { entries_.clear(); inverted_.clear(); }

float BowDatabase::idf(int word) const {
  auto it = inverted_.find(word);
  const int df = (it == inverted_.end()) ? 0 : static_cast<int>(it->second.size());
  const int N = static_cast<int>(entries_.size());
  return std::log((1.0f + N) / (1.0f + df)) + 1.0f;  // smoothed
}

void BowDatabase::addSubMap(std::uint64_t submap_id, const BowVector& bow) {
  const int e = static_cast<int>(entries_.size());
  Entry entry;
  entry.id = submap_id;
  for (const auto& [w, tf] : bow) {
    entry.tf[w] = tf;
    inverted_[w].push_back(e);
  }
  entries_.push_back(std::move(entry));
}

std::vector<std::uint64_t> BowDatabase::query(const BowVector& q, int top_k) const {
  if (entries_.empty() || q.empty()) return {};

  // IDF-weighted query vector + its norm.
  std::unordered_map<int, float> qw;
  float nq = 0.0f;
  for (const auto& [w, tf] : q) {
    const float v = tf * idf(w);
    qw[w] = v;
    nq += v * v;
  }
  nq = std::sqrt(std::max(nq, 1e-12f));

  // Accumulate the IDF-weighted dot product per candidate via the inverted index —
  // only entries sharing a word with the query are visited.
  std::unordered_map<int, float> dot;
  for (const auto& [w, qv] : qw) {
    auto it = inverted_.find(w);
    if (it == inverted_.end()) continue;
    const float iw = idf(w);
    for (int e : it->second) {
      auto f = entries_[e].tf.find(w);
      if (f != entries_[e].tf.end()) dot[e] += qv * (f->second * iw);
    }
  }

  // Cosine similarity (compute each candidate's IDF-weighted norm on the fly).
  std::vector<std::pair<float, std::uint64_t>> scored;
  scored.reserve(dot.size());
  for (const auto& [e, d] : dot) {
    float ne = 0.0f;
    for (const auto& [w, tf] : entries_[e].tf) {
      const float v = tf * idf(w);
      ne += v * v;
    }
    ne = std::sqrt(std::max(ne, 1e-12f));
    scored.emplace_back(d / (nq * ne), entries_[e].id);
  }
  std::sort(scored.begin(), scored.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });

  std::vector<std::uint64_t> out;
  const int k = std::min<int>(top_k, static_cast<int>(scored.size()));
  out.reserve(k);
  for (int i = 0; i < k; ++i) out.push_back(scored[i].second);
  return out;
}

}  // namespace slamko
