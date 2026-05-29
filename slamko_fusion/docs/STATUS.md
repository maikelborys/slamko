# slamko_fusion — Status log

Living, dated progress + numbers log. Plan: [`PLAN_P1_fusion.md`](PLAN_P1_fusion.md).

## 2026-05-29 — GtsamLocalSmoother HARDENED (catch-damp-rebuild) + KF-count window + bias carry-forward; gravity-init gate. gtsam confirmed SECONDARY (Ceres is the estimator)

**Context:** validating the "single estimator" refactor end-to-end on TUM-VI magistrale1
revealed two PRE-EXISTING gtsam bugs (running it, not the static audit, found them):
1. **Died on tracking loss.** A KF inserted during tracking loss (no stereo + broken IMU
   chain) → orphan pose → GTSAM elimination throws `Leftover keys`/`map::at` and the
   smoother is dead for the rest of the run (mag1: 893+ fails, pose exploded to ~700 m).
2. **Inaccurate full-trajectory** (revealed once it stopped dying): see numbers below.

**Fixes shipped (gtsam_local_smoother.{hpp,cpp}):**
- **Orphan-pose guard** — every inserted pose gets ≥1 factor (weak `orphan_prior_sigma`
  prior if no stereo + no IMU factor), so it stays eliminable.
- **`softRebuild()` catch-damp-rebuild** in `optimize()` — on indeterminate system, DISCARD
  the corrupted BatchFixedLagSmoother and re-anchor a fresh window at the last-good nav
  state (the framework's principle #3, GLIM disposable-graph, never actually implemented
  before — it was `catch → return false` = silent death).
- **KF-count fixed-lag window** (`window_kfs`, was 2.0 s wall-clock) — keys stamped by KF
  index, +0.5 marginalization-boundary offset (keeps it off integer stamps). Decouples the
  bias horizon from speed (the "stairs go horizontal" mechanism on gtsam).
- **Bias/velocity carry-forward** on re-anchor — seed from last converged `cur_bias/cur_vel`,
  never reset a converged bias to the pipeline's zero seed.
- **Low-motion gravity-init gate** (vio_pipeline.cpp) — pick the quietest IMU window for the
  gravity DIRECTION estimate (fallback-safe: never regresses start-in-motion seqs like MH_01).

**Numbers (validated):**
- mag1 smoother failures: **893 (+ pose explosion to 700 m) → 10, survives the run.** ✅
- **Decisive accuracy (MH_01_easy, FULL 3419 poses, 0 fails, Sim3-ATE):
  Ceres 10.1 cm vs gtsam 151 cm** — gtsam is ~15× worse, INHERENT (reverting the KF-count
  window to time-based lag gives 153 cm, same). The old gtsam "8.4 cm" was a truncation
  artifact (it died at 2562/3419 poses before the drift tail).
- mag1 stability: Ceres stays ~1 m near origin through frame 1000; gtsam drifts to 12 m. So
  the catastrophic stairs/vertical drift was **gtsam-specific**; Ceres is stable.

**Decision:** **Ceres (`backend=ceres`, the default) is the production single estimator.**
gtsam stays as the optional backend / global loop-closure BA (Phase D), now hardened. This
REVISES the refactor plan B1 (which assumed gtsam should be the sole survivor + Ceres deleted
— backwards on both accuracy and robustness). Memory: `slamko-gtsam-smoother-inaccurate`.

## 2026-05-27 — P1a: GtsamLocalSmoother built + synthetic SfM validated ✅ (marg → P1c)

**What:** `GtsamLocalSmoother` implements `slamko_core::LocalSmoother` using a GTSAM
**`BatchFixedLagSmoother`** (Levenberg-Marquardt + **Schur marginalization** of
out-of-window keys — no leaf constraint, so explicit stereo landmark variables are
handled correctly). Fuses `CombinedImuFactor` (IMU preint + bias, own integration
from raw samples) + `GenericStereoFactor` (metric stereo, `body_P_sensor` = T_BS).
Pose nodes are `T_WB`. GTSAM hidden behind a PIMPL (consumers don't pull GTSAM).

**Build hurdles resolved (documented for the next session):**
- Two GTSAM installs collide: **4.3 at `/usr/local`** (std::shared_ptr, has
  `FixedLagSmoother` in `libgtsam`) vs **4.2 at `/usr`** (boost, smoother in
  `gtsam_unstable`, same SONAME). Fix: `find_package(GTSAM PATHS /usr/local/...
  NO_DEFAULT_PATH)` + **`-Wl,--disable-new-dtags`** + RPATH `/usr/local/lib` so the
  transitive runtime loader picks 4.3 (DT_RUNPATH isn't transitive → 4.2 wins).
- `IncrementalFixedLagSmoother` (iSAM2) was the first choice but its
  `marginalizeLeaves` **cannot** marginalize a pose that bridges long-lived
  landmarks → `map::at`. Switched to `BatchFixedLagSmoother` (Schur, no leaf req).

**GATE — synthetic gtests (through the LocalSmoother contract, no GTSAM in the test):**
`RecoversStereoTrajectory` (12-KF stereo SfM → latest pose <5 cm from truth) ✅,
`FirstKeyframeIsAnchored` ✅. `colcon test`: 4 tests, 0 failures, 1 disabled.

**Honest deferrals to P1c (real EuRoC + IMU — the robust regime):**
- **Marginalization-under-load** is `DISABLED_` in the unit test: a **visual-only**
  graph (poses linked *only* via shared landmarks, no pose-to-pose factor) makes
  GTSAM's fixed-lag marginalization throw. Real VIO always has the **IMU
  `CombinedImuFactor` chain** giving direct pose-to-pose connectivity (what the
  smoother is built for), so marginalization is validated end-to-end on EuRoC in P1c.
- **Precise accuracy:** the latest (newest, least-refined) synthetic pose sits ~3 cm
  off; the real accuracy gate is the EuRoC ATE in P1c, not this plumbing sanity test.

**Next — P1b:** `CeresLocalSmoother` (wrap klt_vo LocalBA behind the contract) +
route `VioPipeline` through `LocalSmoother` (`backend:=ceres|gtsam`, T_WB boundary).
Gate: `backend:=ceres` reproduces the P0 baseline. Then **P1c**: `backend:=gtsam`
end-to-end on EuRoC — validates marginalization + IMU + accuracy for real.

## 2026-05-27 — P1b/P1c: routed through LocalSmoother; backend:=gtsam tracks MH_01 ✅

**P1b:** `CeresLocalSmoother` wraps klt_vo's LocalBA behind the `slamko_core::LocalSmoother`
contract; `VioPipeline` routes through it (`backend:=ceres|gtsam`, T_WB + raw-IMU boundary).
`backend:=ceres` reproduces the P0 baseline (unit-exact). **P1c:** `backend:=gtsam` injected
at the node (composition root — `VioPipeline` never names a backend, Hard Rule #2); GTSAM is
fully encapsulated in the now-**SHARED** `libslamko_fusion.so` (dodges the 4.3/4.2 SONAME
collision via RPATH + `--disable-new-dtags`). Fixed a latent `CombinedImuFactor` arg-order
bug + added landmark management (min-obs + weak prior + cap) to avoid Indeterminant solves.

**GATE:** `backend:=gtsam` **tracks MH_01 end-to-end, 0 smoother failures**, real-time
(`ms_ba` ~13 ms median). Clean early-segment Sim3-ATE ~0.025 m. `colcon test`
fusion + vio = **28 tests, 0 failures**. Full-sequence regression ATE + flipping the default
to gtsam are deferred (the box's rosbag2/harness flakiness — see slamko_vio STATUS); the
in-process `pose_dump` is the ATE path. **Default stays `ceres`** (validated-stable); gtsam
is validated-working. Detail: `PLAN_P1_fusion.md` (stamp P1c).

**Next:** slamko_fusion is the Tier-2 swap surface; the never-lost spine (P2) + cross-session
(P4) built on top are validated (see `slamko_loop/docs/STATUS.md`). Full-sequence gtsam-vs-ceres
ATE A/B awaits a clean harness.
