// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// NodeKey — a typed handle to a variable in the factor graph. A Factor names
// the nodes it touches via NodeKey; the FactorGraphBackend owns the storage.
// The (NodeType, id) pair keeps namespaces disjoint, so pose #3 and landmark #3
// never collide. This is the only identity the contracts exchange.

#pragma once

#include <cstdint>
#include <functional>

namespace slamko {

enum class NodeType : std::uint8_t {
  Pose = 0,           // T_WB ∈ SE(3), body-in-world (MASTER_PLAN §8.3)
  Velocity,           // v_W ∈ R^3
  Bias,               // (b_g, b_a) ∈ R^6
  Landmark,           // 3D point, Euclidean
  InvDepthLandmark,   // anchored inverse-depth (1 DOF in host KF)
  Plane,              // (n, d), 3 DOF
  Line,               // Plücker, 4 DOF
  Object,             // quadric / cuboid
  Extrinsics,         // T_BS sensor-in-body
  GpsDatum,           // T_WL local-world -> ENU (yaw + xyz)
};

struct NodeKey {
  NodeType type = NodeType::Pose;
  std::uint64_t id = 0;

  constexpr NodeKey() = default;
  constexpr NodeKey(NodeType t, std::uint64_t i) : type(t), id(i) {}

  bool operator==(const NodeKey& o) const { return type == o.type && id == o.id; }
  bool operator!=(const NodeKey& o) const { return !(*this == o); }
  // Total order: by type first, then id. Lets NodeKey live in std::map / sort.
  bool operator<(const NodeKey& o) const {
    if (type != o.type) return type < o.type;
    return id < o.id;
  }
};

}  // namespace slamko

// Hash so NodeKey can key std::unordered_map (e.g. NodeValues).
template <>
struct std::hash<slamko::NodeKey> {
  std::size_t operator()(const slamko::NodeKey& k) const noexcept {
    // Pack the 8-bit type into the high bits of the 64-bit id; collisions only
    // if an id exceeds 2^56, far beyond any realistic graph.
    const std::uint64_t packed =
        (static_cast<std::uint64_t>(k.type) << 56) ^ k.id;
    return std::hash<std::uint64_t>{}(packed);
  }
};
