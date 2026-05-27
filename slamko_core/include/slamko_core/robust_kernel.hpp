// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// RobustKernel — the per-factor M-estimator. A Factor exposes one of these so
// the backend can down-weight outliers (IRLS): the effective squared cost is
// rho(e), and weight(e) = rho'(e)/e is the IRLS multiplier applied to the
// whitened residual e = ||sqrt_information * raw_residual||.
//
// MASTER_PLAN §4: data-association-heavy factors (loop/object/GPS) use DCS or
// switchable constraints so a bad association self-deweights toward zero
// instead of corrupting the pose.

#pragma once

#include <algorithm>
#include <cstdint>

namespace slamko {

struct RobustKernel {
  enum class Type : std::uint8_t { None, Huber, Cauchy, DCS, Tukey };

  Type type = Type::None;
  double param = 0.0;  // δ (Huber) / c (Cauchy,Tukey) / Φ (DCS); ignored for None

  constexpr RobustKernel() = default;
  constexpr RobustKernel(Type t, double p) : type(t), param(p) {}

  // IRLS weight w(e) ∈ [0,1] applied to the whitened residual (multiply e by
  // sqrt(w) to reweight). e is the (non-negative) whitened residual norm.
  double weight(double e) const {
    const double e2 = e * e;
    switch (type) {
      case Type::None:
        return 1.0;
      case Type::Huber: {
        const double d = param;
        return (e <= d || e == 0.0) ? 1.0 : d / e;
      }
      case Type::Cauchy: {
        const double c2 = param * param;
        return 1.0 / (1.0 + e2 / c2);
      }
      case Type::Tukey: {
        const double c = param;
        if (e >= c) return 0.0;
        const double r = 1.0 - (e2) / (c * c);
        return r * r;
      }
      case Type::DCS: {
        // Dynamic Covariance Scaling (Agarwal ICRA'13): s = (2Φ/(Φ+e²))²,
        // clamped to 1 for inliers (e² ≤ Φ).
        const double phi = param;
        if (e2 <= phi) return 1.0;
        const double s = 2.0 * phi / (phi + e2);
        return s * s;
      }
    }
    return 1.0;
  }
};

}  // namespace slamko
