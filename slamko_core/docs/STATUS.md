# slamko_core — Status log

Living, dated progress log + current test results. Append on every validated
change (see [`../../docs/DOC_PROCESS.md`](../../docs/DOC_PROCESS.md)).

## 2026-05-27 — Milestone A: foundation shipped (header-only spine)

**What:** slamko is now a buildable colcon workspace with `slamko_core` as its
first real, tested package. All contracts + shared types + the SE(3) manifold +
the health-signal interfaces are in place, header-only and Eigen-only.

**Delivered:**
- Manifold: `se3.hpp` (`SO3`/`SE3`, exp/log/inverse/adjoint/compose, Taylor branch).
- Factor-graph contracts: `factor.hpp`, `sensor_frontend.hpp`, `factor_graph_backend.hpp`.
- **Swappable feature seam** (the P0 heart, first-class from day 1):
  `feature_source.hpp`, `feature_tracker.hpp`, `matcher.hpp`, `features.hpp`,
  `image_view.hpp` (OpenCV-free).
- Recovery contracts: `relocalizer.hpp`, `health.hpp` (signals; policy is P2/loop).
- Cross-tier types: `node_key.hpp`, `estimation_frame.hpp`, `submap.hpp`,
  `custom_data.hpp`, `robust_kernel.hpp`.
- Build: `package.xml` + `CMakeLists.txt` (ament_cmake, INTERFACE lib, Eigen-only).

**Tests:** `colcon test --packages-select slamko_core` → **25/25 gtest pass, 0
failures** (12 SE3 incl. adjoint identity + Taylor branch · 6 NodeKey · 7
contracts/robust-kernel via dummy Factor/Frontend/Backend/FeatureSource impls).

**Build:** `colcon build --packages-select slamko_core` → green (~1.6 s), no warnings.

**Notes / deferred (intentional, not forgotten):** cross-cutting infra the core
README assigns to core — TimeKeeper / trajectory buffer / thread-safe queues,
config + platform presets, structured logging, serialization schema — is NOT
built yet. It lands with its first consumer (vio time-sync, loop serialization),
per "don't freeze contracts before data flows."

**Next:** Milestone B — `slamko_vio`: refactor klt_vo behind
`FeatureSource`/`FeatureTracker`, reproduce MH_01 EuRoC baseline (≈0.054 m @ ~240
fps) as the regression guard, then add the XFeat-TRT `FeatureSource` (primary:
XFeat-detect + KLT-track) + the feature compare-all benchmark.
