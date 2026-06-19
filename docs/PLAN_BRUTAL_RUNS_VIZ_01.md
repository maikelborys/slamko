# Plan — brutal-bag runs + behavior visualization (2026-06-19)

Companion to `docs/PIPELINE_STATUS_01.md`. Captures the recording session, the run order,
and the **visualization decision** for showing slamko's live behavior.

## 1. Bags recorded (2026-06-19) — manifest at `/mnt/data/bno_ab/BRUTAL_BAGS.md`
All: D455 640×480 IR stereo @60 + RGB @30 (emitter OFF) + D455 IMU @200 + BNO055 @100, trimmed
(2 s static pad). Config gotchas: **`rsD455_map_odom`** (640, not odom848); accel DOUBLED in bno_ab;
evaluate `graph.tum` with Umeyama scale. Each bag has a `_preview/` (contact_sheet.png,
motion_profile.png, walk.mp4) — regenerate: `python3 ~/coding/BNO055/ab/bag_preview.py <bag>`.

| Bag | Dur | Stresses |
|---|---|---|
| `CASA1_brutal1_…_trim` | 61 s | fast turns + brusque jolts + revisit (branch supervisor, loop) |
| `CASA1_wall_…_trim` | 59 s | wall(degraded, still 18-24s) + jolts 10-13 rad/s (42-56s) + return-to-start |

**TO RECORD another day** (user deferred): `multidir` (opposite-viewpoint revisit — proves
viewpoint-aware cull), `imublackout` (cover both lenses while moving), `<otherplace>` (strict I2),
`roughterrain`. Recipes in `BRUTAL_BAGS.md`.

## 2. Run order (the lifelong story needs the map built first)
Harness = `scripts/bench_pa.sh <bag> <out> <rate>` (zombie-safe; VPR=true → KF→EigenPlaces→sealed
map in `out/map/`; PRIOR_MAP=<dir> → cross-session relocalize). Default config already correct.
**Before any run: tear down the live D455 driver** — it publishes the SAME `/camera/camera/*`
topics the bag replays → double-publisher corrupts input. (`pgrep -af realsense2_camera`; kill by PID.)

1. **suave + VPR** → builds the clean reference map (also sanity-checks the new recording config
   end-to-end through slamko). `VPR=true scripts/bench_pa.sh CASA1_Suave_… results/run/suave`
2. **brutal1 with PRIOR_MAP=suave/map** → lifelong revisit ("cómo sigue el rollo"): must relocalize,
   must NOT false-merge.
3. **brutal1 / wall single-session (VPR)** → within-session robustness: survive jolts + wall degraded.

## 3. Visualization decision — OFFLINE COMPOSITED VIDEO, not rviz
**Finding:** slamko has NO live rviz pipeline today (zero `.rviz`, zero MarkerArray publishers in
`provider_fusion_node.cpp`). Viz is **offline Plotly** from CSV/dumps (`scripts/plot_*.py`:
plot_multimap, plot_neverlost, plot_health, plot_immortal, plot_dr_gate, plot_reloc_funnel).

**Why not rviz** (user's intuition confirmed): rviz has no notion of *map membership*, so a
**dangling submap with no parent that later connects** can only be hacked via namespace/color; the
event narrative (tracking-lost, edge-created, loop-closure) needs ugly ephemeral Text markers;
recolor-on-connect is state logic rviz doesn't do.

**Chosen: an offline render → mp4** (aligned with slamko's offline-viz model, reproducible,
shareable, LLM-readable). Planned panels:
- **camera + features**: color frame with XFeat keypoints (green=tracked, red=tracking lost).
- **top-down 2D map**: submaps colored by map membership; **GRAY/dashed = dangling (no parent)**;
  edges hard=solid, soft=dashed; current pose. Recolor + draw soft edge when a dangling submap connects.
- **event log strip**: `t=… TRACKING LOST · t=… SOFT EDGE→smN · t=… LOOP CLOSURE smA↔smB · #lm=…`.

Build = a new `scripts/render_behavior_video.py` consuming the run dumps (provider/global/graph.tum,
the submap/landmark dump, anchor_edges.csv, dr_gate.csv, + bag color frames synced by timestamp).
Build it AFTER run #1 (needs real dumps to render against).

## 4. Status of this plan
- [x] Bags recorded + trimmed + previewed + manifested
- [x] Live sensors torn down (no replay conflict)
- [x] **Run #1 suave+VPR** → 13 submaps, 5 loop-closes (kf342/346→sm1, 152-162 inl), 19× dedup;
      reference map in `results/run/suave/map/`. (gate fused-vs-provider 0.42m FAIL = expected: VPR
      loops correct the traj by design; the gate is the no-global P-A check.)
- [x] **`scripts/render_behavior_video.py`** — BUILT + validated on suave (`behavior.mp4`): cam+FAST
      corners | top-down Atlas (submaps/anchors/edges/trail/pose) | event ticker. Reusable on any run.
- [x] `scripts/plot_multimap.py` on suave → `multimap.png/html` (13 islands, 0 odom/12 soft/2 hard).
      **OPEN Q:** 12 SOFT chain edges on a CLEAN bag — likely GPU contention (EigenPlaces+XFeat vs
      OKVIS) → odom micro-stalls → branch supervisor seals+soft-edges. Confirm via the behavior video.
- [x] **Run #2 brutal1 single-session (VPR)** → IMMORTAL CORE FIRED ON REAL STRESS:
      16 submaps; **real TRACKING LOSS @t=44.7s** (the 13 rad/s jolts) → DR-gate d_rot=24.3° (>15°)
      → submap 10 barred as reloc target (degraded) + **seal+branch+SOFT edge** (9→10); then **3 LOOP
      CLOSED back to sm0** at return-to-start (107/93 inl, cost 98→3.9). The full never-lost chain on
      a brutal bag, unforced. `results/run/brutal1/{behavior.html,multimap.png}`.
- [x] **VIZ DECISION LOCKED (user):** Plotly-only, interactive + rewindable, until robustness is
      ensured. NO live-viewer infra (Foxglove/Pangolin/rviz markers) — they'd need slamko to publish
      markers/topics first (same cost as a custom viewer); offline draws from dumps directly and is
      better for algorithm debugging (rewind/zoom/compare). `scripts/plot_behavior.py` = the standard
      (time slider + play). mp4 (`render_behavior_video.py`) deprecated to a secondary artifact.
- [ ] **FINDING → new task: loss-edge covariance not inflated.** anchor_edges.csv: the real
      loss-bridged edge 9→10 has type=1 sigma_t=1 sigma_r=0.3 — **identical to every clean chain edge**.
      The DR-gate measured d_rot=24.3° disagreement but that uncertainty is NOT propagated to the edge
      covariance. Per hard-rule #3 (degradation = covariance inflation), a loss-bridged soft edge
      should carry inflated sigma (and ideally a distinct type/render) so the graph down-weights it and
      the viz shows it. (My earlier "GPU contention" guess for the all-soft chain was WRONG — all chain
      edges are type-1 soft BY DESIGN in loose fusion; the issue is the loss edge isn't *extra*-soft.)
- [x] **Run #3 brutal1 with PRIOR_MAP=suave/map → CROSS-SESSION RELOCALIZE WORKS:**
      `RE-ANCHORED in prior map: kf 151 -> prior submap 6` (T_global_map ~13cm correction); kept
      verifying prior submaps 0/1/2/6/9/11 throughout (same house, no false-merge); closed internal
      return loop (sm13, 111 inl, cost 50→1.0). **`overlay.png` = visual proof**: brutal's map (colored)
      lands ON suave's prior footprint (gray) — same frame. Fusion = the `T_global_map` alignment, not
      persisted edges (prior submaps live in the prior dir). `results/run/brutal1_revisit/{behavior.html,overlay.png}`.
      **OBSERVATION:** cross-session dedup didn't shrink brutal much (brut 13.9k lm vs suave 11.2k) —
      the prior occ seeding could cull revisited landmarks harder (ties to map-bounding-across-sessions).
- [ ] Run #4 wall single-session (the degraded-wall t=18-24s case)
