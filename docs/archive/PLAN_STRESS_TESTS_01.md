<!-- validated: campaign started 2026-06-22 -->
# Stress-test campaign — lifelong mapping + cross-session fusion

Goal: break the system on purpose. Does it survive brutal mapping, vision loss, partial-
overlap fusion, multi-session chains, and adversarial wrong-place merges — and stay bounded?

## Engineering baseline (casa40, measured 2026-06-22)

| Metric | Value | Note |
|---|---|---|
| slamko mapping | ~7 kf/s, **rate 0.5** | OKVIS frame-budget is the throttle (rate-1 diverges on aggressive bags) |
| HITNet depth | ~48 fps (GPU TRT, FFS venv) | per-kf, 640/848 |
| nvblox integrate | **171 fps** (483 frames / 2.8 s) | GPU; re-integrate-once is cheap |
| peak RAM (export) | ~1 GB | nvblox GPU hash + depth load |
| **persist .smap** (graph+landmarks) | **5.7 MB** / session | binary SMP6, round-trip gtest ✅ |
| costmap .pgm (nav surface) | **112 KB** | tiny |
| TSDF mesh .ply | 30 MB | viz/export |
| **depth store .skdf** | **751 MB** / 483 kf ⚠️ | 1.6 MB/frame float32 — the re-integration source does NOT scale lifelong (fix: compress / store disparity / re-derive from bag) |
| deforms with the map? | **YES** | TSDF re-integrated from CORRECTED poses; raw-vs-corrected = 18.2° (casa100) |

## The tests (bags on hand)

| # | Test | Bags | Stresses | Pass = |
|---|---|---|---|---|
| **S1** | Brutal mapping | `brutal1`, `extremeFinal`, `wall` | aggressive motion, wall-pointing | map survives (quality-break fires, no garbage, bounded growth), components sane |
| **S2** | Vision blackout | `Suave_blackout`, `Suave_blackout4`, `Escaleras_2blackout` | tracking loss mid-map | Atlas break→dangle or soft-bridge, recovers, no teleport |
| **S3** | Fusion under stress | brutal/blackout session **+ Suave prior** | cross-session when the session is degraded | rotation→~0, salon fuses, OR honest dangle (no false snap) |
| **S4** | Partial-overlap fusion | a 1-room session + full-house prior | few/clustered matches in a small overlap | aligns if ≥2 spread matches, else dangles (NEVER a false 18° snap) |
| **S5** | Multi-session chain (3×) | Suave → Escaleras → Suave_blackout (640) | drift across 3 fusions | bounded — voxel/landmark count plateaus, no doubling |
| **S6** | Adversarial wrong-place | session vs a **DIFFERENT** prior / look-alike | never-false-merge | REFUSES (dangles), no rotation snap onto the wrong map |
| **S7** | Revisit bounding | same place ×3 in one run | unbounded growth | voxels/landmarks plateau (bounded-by-AREA) |

## Priority (most informative first)
1. **S1 brutal1** (running) — does the map hold under the user's own aggressive bag?
2. **S3** — the new cross-session fix under a degraded session (the real lifelong case).
3. **S6** — never-false-merge (the safety property: must NOT snap onto a wrong map).
4. **S5** — chain bounding (does it stay an apartment, not grow forever).

## Metrics logged per run
submaps · graph components · quality-break LOST events · X-SESSION between edges + distinct
kfs · post-fix ICP rotation vs prior · voxel/landmark count · provider.tum y-span (OKVIS
divergence check) · clean teardown (zombie discipline).

## Known limits to probe
- depth-store size (751 MB/session) — lifelong storage bottleneck.
- rate-1 OKVIS divergence on aggressive motion (run ≤0.5).
- escaleras-class overlap fusion 62% within truncation (vs 82% casa) — harder bags fuse less.
- 848 vs 640 cannot cross-match (config must match).
