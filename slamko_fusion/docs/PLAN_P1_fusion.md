# slamko_fusion — P1 plan (GTSAM fixed-lag smoother + marginalization)

<!-- validated: (P1c) 2026-05-27 · tests: fusion 4 + vio 24 gtest 0 fail · gtsam tracks MH_01 end-to-end 0 smoother-fails -->

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
- **P1c ✅ (tracks end-to-end; full-seq ATE deferred)** — `backend:=gtsam` injected at
  the node (shared `libslamko_fusion.so`, GTSAM 4.3 RPATH; core stays decoupled).
  **Tracks MH_01 end-to-end with 0 smoother failures, real-time** (`ms_ba` ~13 ms).
  Fixed a **latent P1a bug**: `CombinedImuFactor` arg order `(pose_i,vel_i,pose_j,vel_j,
  bias_i,bias_j)` (was bias/pose_j swapped → "retrieve vN as ConstantBias"; never fired
  in P1a's visual-only test). Added reset-on-setter, IMU-chain-only V/B, landmark mgmt
  (≥2-obs + cap → fixes indeterminant + an 8 s/KF batch solve), and wired
  `setStereoCalib`. Marginalization-under-load (P1a's deferral) now exercised: works.
  **Deferred:** clean full-sequence ATE (gtsam vs ceres band) + flipping default to
  gtsam + health probes — blocked by this box's wedged rosbag2 recorder + orphan-prone
  launches (bypassed via `pose_dump_path`; zombie guard added). Default stays `ceres`.
  Detail + the honest caveat: `slamko_vio/docs/STATUS.md` (2026-05-27 P1c).

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
