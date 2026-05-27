# slamko — Decoupling & Contracts

<!-- validated: b36ea43 2026-05-27 · tests: slamko_core 25/25 gtest -->

> **Status (2026-05-27):** the contracts below are implemented header-only in
> `slamko_core` and exercised by 25 passing gtests. The feature seam
> (`FeatureSource` / `FeatureTracker` / `Matcher`, + `ImageView` / `Features`)
> was added to core alongside the original sketch — see `slamko_core/docs/ARCHITECTURE.md`.

How the modules stay independent and pluggable. **The one rule: a module depends
only on `slamko_core`; it never includes another module's headers.** Modules meet
only at the `slamko_core` contracts below — the DigiForest/VILENS/GLIM pattern.

## The contracts (all live in `slamko_core`)

### 1. `Factor` — the unit of fusion (register-not-rewrite)
```cpp
class Factor {
  virtual std::vector<NodeKey> keys()  const = 0;   // typed node handles, ordered
  virtual int                  dim()   const = 0;
  virtual bool evaluate(const NodeValues&, VectorXd& r,
                        std::vector<MatrixXd>* J) const = 0;  // r + tangent Jacobians
  virtual MatrixXd     sqrt_information() const = 0;          // √Ω — only uncertainty knob
  virtual RobustKernel robust_kernel()   const { return {None}; }
};
```

### 2. `SensorFrontend` — turns a measurement into factors (Tier 1)
```cpp
class SensorFrontend {
  virtual std::string name() const = 0;
  virtual void process(const Measurement&, const KeyframeTimeline&,
                       FactorGraphBackend&) = 0;   // create nodes + emit factors
};
```

### 3. `FactorGraphBackend` — owns nodes + solve + marginalization (Tier 2)
```cpp
class FactorGraphBackend {
  virtual NodeKey  addNode(NodeType, const VectorXd& init, bool constant=false) = 0;
  virtual void     addFactor(std::shared_ptr<Factor>) = 0;
  virtual bool     optimize() = 0;
  virtual VectorXd value(NodeKey) const = 0;
  virtual MatrixXd marginalCovariance(NodeKey) const = 0;     // observability monitor
  virtual void     marginalizeOlderThan(double stamp) = 0;    // Schur prior + FEJ
};
// adapters: GtsamBackend (default, iSAM2), CeresBackend (S1 inner loop)
```

**`FactorGraphBackend` (generic) vs `LocalSmoother` (VI façade).** The Tier-2 swap
the VIO actually uses is at **VI-keyframe granularity**, and each backend owns its
IMU preintegration — so `slamko_core::LocalSmoother` (`local_smoother.hpp`) is the
practical contract: `setImuParams`/`setStereoCalib`/`insertKeyframe(T_WB, raw-IMU
span, stereo obs)`/`optimize`/readback/`health`. Impls: `GtsamLocalSmoother`
(`slamko_fusion`, IncrementalFixedLagSmoother + marginalization) and
`CeresLocalSmoother` (`slamko_vio`, wraps klt_vo's LocalBA). `FactorGraphBackend`
above stays the **low-level extension seam** for arbitrary/custom factors (→
`gtsam::CustomFactor`); LocalSmoother is the optimized VI-core fast path.

### 4. `SubMap` / `EstimationFrame` — the only types crossing tiers
Serializable, with a `custom_data: map<string, shared_ptr<void>>` escape hatch
(GLIM) so the pipeline extends without changing the interface. `SubMap` = {KF
poses, landmarks, small descriptor index, optional dense payload, anchor pose}.

### 5. Map-server contract (DigiForest) — `slamko_msgs` + `slamko_mapping`
The map is a **published data format + API**, not an internal class → multi-robot,
multi-session, swappable backends. Defines: submap persistence format, the
`map→odom` correction stream, `localization_status`, cross-session correspondences.

## Pluggability in practice

`slamko_vio`, `slamko_sensors`, `slamko_semantic` each implement `SensorFrontend`
(+ concrete `Factor`s). `slamko_reloc` implements a `Relocalizer` interface.
`slamko_fusion` provides `FactorGraphBackend` adapters. `slamko_ros` is the only
place that wires concrete modules together (composition root). Swap any one by
registering a different implementation — no other module changes.

## Recovery decoupling (never-lost)
The multi-map supervisor (`slamko_loop`) runs **external** to the estimator graph:
it consumes `localization_status` + odometry stale-gap, drives seal/branch, and
consumes `T_AB` correspondences to drive merge — gating OUTSIDE the Ceres/GTSAM
graph. (Tight in-graph coupling "pendulates" — the user's OKVIS2-X finding.)
