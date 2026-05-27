// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// BoW vocabulary + inverted-index database tests — synthetic, deterministic. Three
// well-separated descriptor clusters in 64-d stand in for three places; the
// vocabulary must recover them, the database must retrieve the right submap for a
// query drawn from a known cluster, and the build must be reproducible.

#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "slamko_loop/bow.hpp"

using slamko::BowDatabase;
using slamko::BowVector;
using slamko::BowVocabulary;
using slamko::DescriptorBlock;

namespace {

// n descriptors clustered around `center` (one-hot-ish blob) with small noise.
DescriptorBlock blob(int n, int center_dim, float spread, std::mt19937& rng) {
  std::normal_distribution<float> noise(0.0f, spread);
  DescriptorBlock X(n, 64);
  for (int i = 0; i < n; ++i) {
    for (int d = 0; d < 64; ++d) X(i, d) = noise(rng);
    X(i, center_dim) += 5.0f;  // dominant axis = the "place"
  }
  return X;
}

DescriptorBlock stack(const std::vector<DescriptorBlock>& parts) {
  int n = 0;
  for (const auto& p : parts) n += p.rows();
  DescriptorBlock X(n, 64);
  int r = 0;
  for (const auto& p : parts) { X.middleRows(r, p.rows()) = p; r += p.rows(); }
  return X;
}

}  // namespace

TEST(Bow, VocabularyRecoversClusters) {
  std::mt19937 rng(7);
  const DescriptorBlock X =
      stack({blob(60, 3, 0.2f, rng), blob(60, 20, 0.2f, rng), blob(60, 50, 0.2f, rng)});
  BowVocabulary v;
  v.build(X, /*K=*/3, /*iters=*/15, /*seed=*/1);
  ASSERT_EQ(v.size(), 3);
  // The three blob prototypes must quantize to three DISTINCT words.
  std::mt19937 r2(99);
  const int w0 = v.quantize(blob(1, 3, 0.0f, r2).row(0));
  const int w1 = v.quantize(blob(1, 20, 0.0f, r2).row(0));
  const int w2 = v.quantize(blob(1, 50, 0.0f, r2).row(0));
  EXPECT_NE(w0, w1);
  EXPECT_NE(w1, w2);
  EXPECT_NE(w0, w2);
}

TEST(Bow, Deterministic) {
  std::mt19937 a(1), b(1);
  const DescriptorBlock Xa =
      stack({blob(40, 1, 0.3f, a), blob(40, 30, 0.3f, a), blob(40, 60, 0.3f, a)});
  const DescriptorBlock Xb =
      stack({blob(40, 1, 0.3f, b), blob(40, 30, 0.3f, b), blob(40, 60, 0.3f, b)});
  BowVocabulary va, vb;
  va.build(Xa, 5, 10, 42);
  vb.build(Xb, 5, 10, 42);
  ASSERT_EQ(va.size(), vb.size());
  EXPECT_EQ((va.words() - vb.words()).cwiseAbs().maxCoeff(), 0.0f);  // bit-identical
}

TEST(Bow, DatabaseRetrievesRightSubmap) {
  std::mt19937 rng(3);
  std::vector<DescriptorBlock> places = {
      blob(80, 5, 0.2f, rng), blob(80, 25, 0.2f, rng), blob(80, 45, 0.2f, rng)};
  BowVocabulary v;
  v.build(stack(places), /*K=*/8, 15, 1);

  BowDatabase db;
  for (int s = 0; s < 3; ++s)
    db.addSubMap(/*submap_id=*/100 + s, v.transform(places[s]));
  ASSERT_EQ(db.size(), 3u);

  // A fresh query from place 1 → top candidate must be submap 101.
  std::mt19937 q(123);
  const BowVector qbow = v.transform(blob(40, 25, 0.2f, q));
  const auto cand = db.query(qbow, /*top_k=*/2);
  ASSERT_FALSE(cand.empty());
  EXPECT_EQ(cand.front(), 101u);
}

TEST(Bow, EmptyAndEdgeCases) {
  BowVocabulary v;
  EXPECT_TRUE(v.empty());
  EXPECT_TRUE(v.transform(DescriptorBlock(0, 64)).empty());
  BowDatabase db;
  EXPECT_TRUE(db.query({{1, 1.0f}}, 5).empty());   // empty db
  db.addSubMap(7, {{2, 1.0f}});
  EXPECT_TRUE(db.query({}, 5).empty());            // empty query
  EXPECT_EQ(db.query({{2, 1.0f}}, 5).front(), 7u); // shares a word
}
