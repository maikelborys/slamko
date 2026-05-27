# slamko_fusion — P1 plan (GTSAM fixed-lag smoother + marginalization)

<!-- validated: (P1b) 2026-05-27 · tests: slamko_vio 23 gtest 0 fail (4 adapter parity) -->

Read [`../../docs/SYSTEM.md`](../../docs/SYSTEM.md) + [`../../docs/DECOUPLING.md`](../../docs/DECOUPLING.md)
first. Progress + numbers: [`STATUS.md`](STATUS.md).

## Goal
Replace klt_vo's lossy gauge-by-constant local BA with a GTSAM fixed-lag smoother
that **marginalizes** old nodes (Schur + FEJ) and fuses heterogeneous factors by
covariance — the VILENS heart. Default backend; Ceres LocalBA stays as fallback.
Both behind `slamko_core::LocalSmoother`, swappable via `backend:=ceres|gtsam`.

## Design (locked)
- **`BatchFixedLagSmoother`** (LM + Schur marginalization; no leaf constraint, so
  explicit stereo landmarks work — `IncrementalFixedLagSmoother`/iSAM2 cannot
  marginalize landmark-bridging poses).
- Nodes: `Pose3` (T_WB) · `Vector3` (vel) · `imuBias::ConstantBias` · `Point3`
  (landmarks). Factors: `CombinedImuFactor` (own preint from raw IMU) +
  `GenericStereoFactor` (`Cal3_S2Stereo`, `body_P_sensor` = T_BS).
- GTSAM behind a PIMPL. GTSAM 4.3 at `/usr/local` forced + RPATH/`--disable-new-dtags`
  (4.2 at `/usr` collides on SONAME).

## Phases
- **P1a ✅** — `GtsamLocalSmoother` + core `LocalSmoother`/`ImuSample`/
  `StereoObservation` + synthetic gtests (stereo SfM recovery + bounded window).
  Marginalization-under-load + precise accuracy deferred to P1c (need real IMU; see STATUS).
- **P1b ✅** — `CeresLocalSmoother` (wraps LocalBA) + `VioPipeline` routed through
  the contract + `backend:=ceres|gtsam` param + injection ctor (gtsam wired at the
  node in P1c, no vio→fusion dep). Gate met via **unit-exact pass-through** (≤1e-9,
  visual + IMU paths) — the planned "==P0 ATE" gate was unmeasurable (VIO is
  nondeterministic run-to-run; front-end CUDA, upstream of this seam). Pre/post ATE
  bands overlap (MH_01 Shi-Tomasi median ~0.074). Detail: `slamko_vio/docs/STATUS.md`.
- **P1c** — `backend:=gtsam` end-to-end on EuRoC (ShiTomasi + XFeat). Gate: GTSAM ≤
  baseline ATE; window bounded; real-time. Emit health probes (degeneracy eigenvalue,
  marginal cov). Validates marginalization + IMU + accuracy for real.

## Key files
`include/slamko_fusion/gtsam_local_smoother.hpp` (PIMPL) ·
`src/gtsam_local_smoother.cpp` (all GTSAM) · `test/test_gtsam_smoother.cpp` ·
core `local_smoother.hpp`/`imu_sample.hpp`/`stereo_observation.hpp`.

## Verify
```bash
cd ~/coding/slamko && colcon build --packages-select slamko_core slamko_fusion \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select slamko_fusion && colcon test-result --test-result-base build/slamko_fusion
```
