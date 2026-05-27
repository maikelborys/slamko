// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// NodeKey identity: equality, the (type, id) namespace disjointness, ordering,
// and that it can key an unordered_map (the NodeValues container).

#include <gtest/gtest.h>

#include <map>
#include <unordered_map>

#include "slamko_core/node_key.hpp"

using slamko::NodeKey;
using slamko::NodeType;

TEST(NodeKey, Equality) {
  EXPECT_EQ(NodeKey(NodeType::Pose, 3), NodeKey(NodeType::Pose, 3));
  EXPECT_NE(NodeKey(NodeType::Pose, 3), NodeKey(NodeType::Pose, 4));
}

// pose #3 and landmark #3 must be distinct (disjoint namespaces).
TEST(NodeKey, TypeNamespacesAreDisjoint) {
  EXPECT_NE(NodeKey(NodeType::Pose, 3), NodeKey(NodeType::Landmark, 3));
}

TEST(NodeKey, OrderingByTypeThenId) {
  EXPECT_LT(NodeKey(NodeType::Pose, 5), NodeKey(NodeType::Velocity, 0));
  EXPECT_LT(NodeKey(NodeType::Pose, 1), NodeKey(NodeType::Pose, 2));
  EXPECT_FALSE(NodeKey(NodeType::Pose, 2) < NodeKey(NodeType::Pose, 2));
}

TEST(NodeKey, WorksInStdMap) {
  std::map<NodeKey, int> m;
  m[NodeKey(NodeType::Pose, 1)] = 10;
  m[NodeKey(NodeType::Landmark, 1)] = 20;
  EXPECT_EQ(m.size(), 2u);
  EXPECT_EQ(m[NodeKey(NodeType::Pose, 1)], 10);
}

TEST(NodeKey, WorksInUnorderedMap) {
  std::unordered_map<NodeKey, int> m;
  m[NodeKey(NodeType::Pose, 7)] = 1;
  m[NodeKey(NodeType::Bias, 7)] = 2;
  m[NodeKey(NodeType::Pose, 8)] = 3;
  EXPECT_EQ(m.size(), 3u);
  EXPECT_EQ(m.at(NodeKey(NodeType::Bias, 7)), 2);
}

TEST(NodeKey, HashDistinguishesTypeAndId) {
  std::hash<NodeKey> h;
  EXPECT_NE(h(NodeKey(NodeType::Pose, 3)), h(NodeKey(NodeType::Landmark, 3)));
  EXPECT_NE(h(NodeKey(NodeType::Pose, 3)), h(NodeKey(NodeType::Pose, 4)));
}
