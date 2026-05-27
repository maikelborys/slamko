// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Factor — the unit of fusion and of pluggability (MASTER_PLAN §2, "register-
// not-rewrite"). A sensor frontend turns a measurement into Factors; the
// backend (GTSAM/Ceres adapter) owns nodes + solve + marginalization. The
// backend never names a sensor; the frontend never touches the solver.
//
// The ONLY uncertainty knob is sqrt_information() (√Ω); robustness rides on
// robust_kernel(). Degradation is covariance inflation, never an if-branch
// (MASTER_PLAN §0, principle 2).

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "slamko_core/node_key.hpp"
#include "slamko_core/robust_kernel.hpp"

namespace slamko {

// The linearization point passed to evaluate(): each touched node's current
// value, in its minimal/ambient parameterization (the backend defines the
// layout per NodeType; Pose may be a 6-vector tangent or a 7-vector, etc.).
using NodeValues = std::unordered_map<NodeKey, Eigen::VectorXd>;

class Factor {
 public:
  virtual ~Factor() = default;

  // Typed node handles this factor connects, in a fixed order. The Jacobian
  // block order in evaluate() matches this order.
  virtual std::vector<NodeKey> keys() const = 0;

  // Residual dimension k.
  virtual int dim() const = 0;

  // Evaluate the (already-whitened or raw — see sqrt_information) residual r
  // (size k) at `values`. If J != nullptr, also fill the tangent-space
  // Jacobians: one block per key, J[i] is k × dim(tangent of keys()[i]).
  virtual bool evaluate(const NodeValues& values, Eigen::VectorXd& r,
                        std::vector<Eigen::MatrixXd>* J) const = 0;

  // √Ω — the square-root information matrix (k × k), the sole uncertainty knob.
  virtual Eigen::MatrixXd sqrt_information() const = 0;

  // M-estimator applied to the whitened residual. Default: none.
  virtual RobustKernel robust_kernel() const { return {}; }
};

using FactorPtr = std::shared_ptr<Factor>;

}  // namespace slamko
