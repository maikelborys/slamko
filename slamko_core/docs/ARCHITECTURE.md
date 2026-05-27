# slamko_core — Architecture

<!-- validated: 3995b5c 2026-05-27 · tests: 25/25 gtest -->

The spine. Header-only, **Eigen-only** (no ROS / CUDA / GTSAM / Ceres), so every
other package links it for free. Defines the contracts modules meet at and the
shared types crossing tiers — nothing more. See [`../../docs/DECOUPLING.md`](../../docs/DECOUPLING.md)
for the system-wide rationale and [`../../CLAUDE.md`](../../CLAUDE.md) for the rules.

## Frame conventions (locked — MASTER_PLAN §8.3)

- Pose node = **`T_WB`** (body-in-world). Each sensor carries its own `T_BS`
  (sensor-in-body); this removes klt_vo's world-to-cam asymmetry.
- SE(3) twist ordering = **`[rho; omega]`** (translation-first, Sophus-compatible):
  `exp([rho; omega]) = (SO3::exp(omega), V(omega)·rho)`, with `V` the SO(3) left
  Jacobian. `SE3 * point` transforms; `SE3 * SE3` composes.

## The manifold (`se3.hpp`)

`SO3` (unit-quaternion storage) + `SE3` (SO3 + translation). exp/log/inverse/
adjoint/compose, each with a near-zero Taylor branch (cutoff `kEps2 = 1e-20`).
SO(3) exp uses Rodrigues via `AngleAxisd`; SE(3) exp/log use the closed-form left
Jacobian `V` and its inverse `V⁻¹`. The templated `so3_t::Exp/Log` in klt_vo
`imu_factor.hpp` is the validated reference for the small-angle handling. Adjoint
(for `[rho; omega]`): `Adj = [[R, [t]ₓR], [0, R]]`. Verified by the
`T·exp(ξ)·T⁻¹ == exp(Adj_T·ξ)` identity in `test_se3.cpp`.

## The contracts (abstract, register-not-rewrite)

| Header | Interface | Role |
|---|---|---|
| `factor.hpp` | `Factor` | unit of fusion: `keys / dim / evaluate(values,r,J) / sqrt_information / robust_kernel`. `√Ω` is the only uncertainty knob; robustness via `RobustKernel`. |
| `sensor_frontend.hpp` | `SensorFrontend`, `Measurement`, `KeyframeTimeline` | Tier-1 plugin: measurement → nodes + factors. Adding a sensor = register one. |
| `factor_graph_backend.hpp` | `FactorGraphBackend` | owns nodes + solve + **marginalization** (`marginalizeOlderThan` = Schur prior + FEJ) + `marginalCovariance` (observability probe). GTSAM/Ceres adapters. |
| `feature_source.hpp` | `FeatureSource` | **swappable detector/descriptor seam**: `ImageView → Features`. Shi-Tomasi / XFeat / LiftFeat-m1 implement it. |
| `feature_tracker.hpp` | `FeatureTracker` | frame-to-frame association (KLT flow ↔ descriptor-match). detect ⊥ track decoupling → "XFeat detect + KLT track". |
| `matcher.hpp` | `Matcher` | two-view descriptor matching for reloc/loop (LighterGlue / MNN), NOT the hot path. |
| `relocalizer.hpp` | `Relocalizer`, `RelocResult` | long-loss/kidnap recovery; returns ONE relative-pose weld constraint, gated outside the graph. |

Concrete impls (CUDA/TRT/torch) live in `slamko_vio`/`slamko_loop` as optional
build targets, behind these Eigen-only contracts. `image_view.hpp` (an
OpenCV-free non-owning grayscale view) is what keeps the feature contracts
dependency-free — vio wraps its `cv::Mat`/`GpuMat` at the call boundary.

## Shared types (cross-tier)

- `node_key.hpp` — `NodeKey{NodeType,id}`: typed graph-variable handle, disjoint
  namespaces (pose #3 ≠ landmark #3), hashable/orderable. `NodeValues =
  unordered_map<NodeKey, VectorXd>`.
- `features.hpp` — `Features` (N×3 keypoints + optional N×D descriptors) + `Track`.
  The output contract all three detectors converge on.
- `estimation_frame.hpp` / `submap.hpp` — the two types crossing Tier-2 → Tier-3,
  each with the GLIM `custom_data` escape hatch (`custom_data.hpp`).
- `robust_kernel.hpp` — `RobustKernel` (None/Huber/Cauchy/DCS/Tukey) + IRLS `weight(e)`.

## Health (signals here, policy in loop)

`health.hpp`: `HealthState{Good,Marginal,Lost}` + `HealthSignal` probe +
`HealthReporter`. **Loss detection = `odom_stale_gap_s`** (odometry stale-gap),
NOT a covariance spike — a blackout pauses odom, it doesn't inflate covariance
(OKVIS2-X finding). Eigenvalue / marginal-cov fields are degeneracy monitors. The
*decision policy* (state machine, watchdogs, recovery triggers) is the never-lost
supervisor in `slamko_loop` (P2).

## Key files
`include/slamko_core/*.hpp` (all header-only) · `test/{test_se3,test_node_key,test_contracts}.cpp`.

## Deferred (added when a consumer first needs it — not pre-built)
TimeKeeper / interpolatable trajectory buffer / thread-safe queues · config +
platform presets · structured logging · map serialization schema. The core README
lists these as core's eventual responsibility; they land with their first user
(vio time-sync, loop serialization), not speculatively.
