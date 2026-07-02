# PLAN_IMMORTAL_FRAMEWORK_01 — consolidar el framework SLAM inmortal (plan maestro v3)

<!-- status: AUTHORED 2026-07-02 · the authoritative campaign plan for the
     "framework antimortal completo" push (user vision restated 2026-07-02).
     Supersedes the ordering of remaining work in PLAN_ROBUSTNESS_01 §8 and
     MASTER_PLAN §8 P-C..P-E; those keep the research provenance. -->

## 0. The verdict that shapes this plan

The user's vision (2026-07-02, verbatim intent): *VIO-only immortal SLAM — seal on
bad tracking, new island when tracking is good again, weld islands by features on
loop, hard/soft edges, deformable volumetric map, no fear of tracking loss, no
wheel/leg odometry, drone-grade fast motion; total root refactor if needed.*

**That vision IS the built architecture** (Atlas islands + seal-on-doubt HOLD +
IMU referee + quality-break + soft/hard covariance edges + XFeat weld + nvblox
deformable TSDF — all shipped, validated on brutal/wall/blackout bags). The gap is
**consolidation, not construction**:

1. **The immortality gates are opt-in default-OFF** (audit 2026-07-02): `imu_referee`,
   `hold_on_loss`, `gate_live_pose`, `depth_loop_refine`, `atlas_break_on_quality`,
   `dr_gate_soft_cov`, `quality_soft_bridge`, `mappoint_assoc`, `compass_yaw_prior`
   all default false (only `proximity_three_tier`, `local_dynamic`,
   `xsession_prior_factor` are ON). A framework whose defining features are off by
   default is a research node, not a framework.
2. **No reproducible full-battery regression** — 26 ad-hoc `.sh` + `slamko_eval.py`
   exist, but no one-command run over ALL bags producing one scorecard table.
3. **One god node**: `provider_fusion_node.cpp` = 3,087 lines, ~150 fields, 0 gtests
   (slamko_ros has zero tests). The libraries beneath are clean.
4. **~6,900 LOC dead own-VIO** in slamko_vio (+7 stale gtests) slated for deletion
   since v2; only eigenplaces/xfeat extractors (~600 LOC) are live.
5. **Compass wired but dark**: `compass_yaw_prior` reaches `graph_.addYawPrior`
   (provider_fusion_node.cpp:940) but defaults OFF and is unvalidated; raw-mag path
   is instrument-only (CSV).
6. **Navigation over soft edges** — genuinely open design question.

**Refactor verdict (code audit 2026-07-02): targeted decomposition, NOT a root
rewrite.** Package boundaries, provider contract, covariance-as-degradation and the
offline gate (`provider_chain_offline`) are sound and benchmarked. A ground-up reorg
would discard working, measured infrastructure to fix a problem localized to one
file and one dead subtree.

## 1. Task order (measurement before change — the method that paid off)

| # | Task | Gate to close it |
|---|---|---|
| **T1** | **Regression battery** (`scripts/battery.sh` + report) | one command runs the immortal profile over the full bag set + EuRoC, emits one scorecard table (slamko_eval 7 channels + ATE + islands/welds); 2 consecutive runs agree within noise |
| **T2** | **Immortal profile → default ON** | T1 A/B (gates OFF vs ON) shows no regression on the clean bags and strict wins on brutal/wall/blackout → flip defaults, add `immortal:=false` escape hatch |
| **T3** | **Targeted decomposition** | delete own-VIO (−6.9k LOC); extract from the god node: NeverLost/LossGate state machine, CompassYawSource, VolumetricBridge, MapPersistence — each with gtests; battery output IDENTICAL before/after (differential golden-map method) |
| **T4** | **Compass BNO055 live** | `compass_yaw_prior` ON over the bno_ab bags: maps north-aligned across sessions, no regression on ATE/coherence; mag Kok-Schön calib stays deferred to outdoor |
| **T5** | **Soft-edge navigation design** | design doc: islands = waypoint graph (Spot GraphNav pattern); hard edge = traversable, soft edge = traversable-with-penalty + re-localize-at-crossing, no edge = unreachable-until-welded; validate in sim (Gazebo) after T1–T3 |
| **T6** | **Viewpoint-recall bags** (user records: multidir, imu-blackout-moving, other-place) | strict I2 + opposite-heading weld measured |

## 2. T1 — the regression battery (the "tests de recorridos")

One command, serial runs (concurrent benches reap each other — hard gotcha), each
bag → run dir → `slamko_eval.py` + ATE + island/weld counts → one markdown table.

**Bag set** (config per row is load-bearing — resolution mismatch inflated scale
1.7× a whole day once):

| Bag | Config | Stresses |
|---|---|---|
| CASA1_Suave_…BNO_strim | rsD455_map_odom (640) + bno_ab launch (accel ×2!) | clean reference map |
| CASA1_Escaleras_…_strim | idem | stairs, 3D |
| CASA1_40cmH / 100cmH_848_trim | rsD455_map848 | cross-session weld 40↔100 |
| CASA1_brutal1_…_trim | rsD455_map_odom | jolts, motion blur, revisit |
| CASA1_wall_…_trim | idem | blank wall + jolts + return (the rich one) |
| CASA1_Suave_blackout, blackout4 | idem | kidnap/recovery |
| CASA1_extremeFinal_strim | idem | extreme motion |
| casa_084815_flashbno_trim | rsD455_odom848 + splitter | 45fps HW depth → volumetric + depth-loop |
| EuRoC MH_03 (median-of-1 ok, V1 ±50%) | euroc | ATE vs GT anchor |

**Per-run outputs:** provider.tum, graph.tum, traj_slewed.tum, fusion.log, map/,
eval JSON. **Report:** `scripts/battery_report.py` → `docs/battery/BATTERY_<date>.md`
(+ the tsdf_slice / smap_cloud renders for the volumetric rows — SHOW the map,
don't just claim coherent).

**Known confound:** OKVIS is GPU-nondeterministic run-to-run → battery verdicts are
per-channel PASS/FAIL + coarse metrics, not mm-level ATE diffs; rate ≤0.5 where
history says so; OKVIS-full-SLAM as GT for the flash bag.

## 3. Standing answers (so they stop being re-asked)

- **wavemap vs nvblox**: decision FROZEN 2026-06-23 (RESEARCH_LIFELONG_NAV_ARCH_01)
  — keep nvblox (true ESDF, live, deforms in-loop); wavemap deferred to
  building/campus scale. Don't reopen without new data.
- **Provider**: cuVSLAM is the rumbo (45fps), OKVIS the validated default; slamko
  is provider-agnostic BY DESIGN and the evaluator judges the layer, not the
  provider. Don't pivot providers to dodge a blocker.
- **Sim**: Gazebo (`cerebro_robot_sim`) is the sim that fits the 8GB GPU
  (Isaac saturates → OKVIS starves). Bags remain the primary refinement surface
  (user 2026-07-02) — sim only for Nav2 closed-loop (T5).
