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

## 2026-05-27 — P1 contract additions: LocalSmoother + VI types

Added the Tier-2 VI fusion contract used by slamko_fusion/slamko_vio:
`local_smoother.hpp` (`LocalSmoother`: setImuParams/setStereoCalib/setExtrinsics/
insertKeyframe(T_WB, raw-IMU span, stereo obs)/optimize/readback/health),
`imu_sample.hpp` (`ImuSample`, `ImuBias`, `ImuParams`), `stereo_observation.hpp`
(`StereoObservation`, `StereoCalib`). Still header-only, Eigen-only. The generic
`FactorGraphBackend` stays the low-level extension seam; `LocalSmoother` is the
VI-core swap surface (see `DECOUPLING.md`). Exercised by slamko_fusion's gtests.

**Next:** Milestone B — `slamko_vio`: refactor klt_vo behind
`FeatureSource`/`FeatureTracker`, reproduce MH_01 EuRoC baseline (≈0.054 m @ ~240
fps) as the regression guard, then add the XFeat-TRT `FeatureSource` (primary:
XFeat-detect + KLT-track) + the feature compare-all benchmark.

## 2026-05-27 — P4a: SubMap serialization (map-persistence schema) ✅

`submap_io.hpp` (header-only, Eigen+std): binary save/load of a `SubMap` — id, anchor
(quat+t), keyframes, landmarks (id, xyz, descriptor_row), and the N×64 descriptor block
(row-major) — plus `saveSubMaps`/`loadSubMaps` for a whole archive directory
(`submap_<id>.smap` + `submaps.manifest`). This is the persistence foundation for
cross-session recovery (P4): a sealed submap survives to disk and reloads into a fresh
session's relocalizer. Submaps stay **connected via anchors, never fused** (Hard Rule #4),
so each round-trips independently — and the disjoint-submap partition (slamko_vio) means
no duplicated landmarks on disk. Format: little-endian, same-arch (x86); CustomData
(dense payloads) NOT serialized yet (per-payload concern). **GATE — 4 gtests** (single
round-trip incl. non-identity anchor + N×64 descriptors binary-exact; descriptor-less
submap; multi-submap archive dir via manifest; missing-file → false). Core total: 26
gtests, 0 fail.

**Next — P4b:** wire it into the node — dump the sealed archive at shutdown
(`saveSubMaps`), load a prior map on startup into the `XFeatRelocalizer` + supervisor,
and validate a cross-session weld (session 2 relocalizes into session 1's map).
