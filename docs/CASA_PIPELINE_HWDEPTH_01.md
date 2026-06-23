<!-- validated: end-to-end run 2026-06-23 on casa_084815_flashbno_trim -->
# slamko on a live D455 flashing bag — full pipeline + HW-depth volumetric

> First end-to-end run of slamko on a **self-recorded** D455 bag (not EuRoC/casa-legacy):
> `casa_084815_flashbno_trim` (848×480 emitter-FLASHING, 77 s, BNO055). Bag recipe:
> [`V2A_LIVE_DEPTH_D455_01.md`](V2A_LIVE_DEPTH_D455_01.md) + `~/coding/d455_setup/record_d455_flashing_bno.sh`.

## The pipeline (one script: `scripts/run_slamko_casa_flashbag.sh`)

```
bag (flashing 90fps) ─► d455_splitter_auto.py  ──clean IR──► OKVIS stereo VIO ──odom──┐
                          (content/Laplacian,    /okvis/cam0,cam1   (rsD455_odom848)   │
                           NOT metadata)          ──dotted depth──► /nvblox/depth      ▼
   cam_info_inject_848.py ──► /camera/camera/infra1,2/camera_info       slamko provider_fusion_node
   (bag has NO camera_info)                                             (XFeat VPR + loop closures)
                                                                          └─► graph.tum + submap_*.smap (landmarks) + anchor_edges
   bag HW depth ─► make_depth_skdf_frombag.py ─► kf_*.skdf ─► slamko_tsdf_export ─► map.ply (TSDF) + map.pgm (costmap)
   rerun_show.py / render_casa.py ─► landmarks + volumetric + trajectory
```

**Run:** `bash scripts/run_slamko_casa_flashbag.sh` (edit BAG/OUT/CFG at top). Then:
```
python3 scripts/make_depth_skdf_frombag.py --archive <out>/map --bag <bag> --out <skdf>
<install>/slamko_tsdf/.../slamko_tsdf_export <out>/map <skdf> <out>/vol/map 0.05 0.10
/tmp/rrviewer/bin/python3 scripts/rerun_show.py <out>/vol/map.ply <out>/graph.tum out.rrd --archive <out>/map
```

## Result (casa_084815, validated)
Trajectory **42 m**, extent 3.9×9.7×0.6 m, **loop CLOSED** (start≈end), **1 component**, **6 loop
closures**. **14 234 landmarks** / 11 submaps. Volumetric **180 176 verts**, **498/498** depth frames
integrated. Split 3393 clean→VIO / 3330 dotted-depth→volumetric. OKVIS did NOT diverge (flat Z,
sane scale) — the per-session config + content-splitter + rate-0.5 (anti GPU-contention) held.

## Two load-bearing facts

**1. OKVIS does NOT consume the HW depth here.** The run uses `okvis2x_stereo_network_node_subscriber`
+ `rsD455_odom848` (stereo VIO, no submapping) → OKVIS triangulates landmarks from the **clean IR
stereo pair**, not the depth. The HW depth feeds ONLY the volumetric layer (slamko_tsdf), decoupled.
OKVIS *can* fuse depth (`okvis2x_depthfusion_*` + `rsD455_map848` se2 submapping) — that's a roadmap
choice (tighter but couples the provider to depth; current split keeps depth in the volumetric layer
only, which is the cleaner decoupling).

**2. HW depth (emitter-ON) vs recomputed SGBM — why we use HW.** `make_depth_skdf.py` (the original)
recomputes stereo SGBM from the **clean IR** (emitter-OFF) → no projector texture → holes on blank
walls + speckle. `make_depth_skdf_frombag.py` (new) takes the bag's **real ASIC depth**, picking per
keyframe the depth frame with the MOST valid pixels (= the emitter-ON frame adjacent to the clean-IR
keyframe). Plausibly better quality + robustness — emitter dots texture blank walls, 498/498 coverage,
no SGBM recompute/speckle — **but not yet A/B'd** (slamko rule #5: measure before claiming). This is
exactly why flashing recording pays off: clean IR for VIO **and** emitter-textured depth for the map.

## Gotchas hit (so the next run doesn't)
- Flashing bag → MUST insert the **python** content-splitter (`d455_splitter_auto.py`); the NVIDIA C++
  `realsense_splitter` routes by `frame_emitter_mode` metadata, which is **offset/inverted on this D455**.
- The recorded bag has **NO camera_info** (forgot to record it) → `cam_info_inject_848.py` publishes the
  848 intrinsics (left + right P encodes baseline 0.0950564) for the slamko relocalizer.
- Config MUST match resolution: `rsD455_odom848` (848, D455 IMU). Run VPR-on at **rate ≤0.5** (OKVIS↔XFeat
  GPU contention diverges OKVIS at rate 1). 20 s GPU warm-up before the bag.
- `make_depth_skdf_frombag.py` uses **system ROS python** (rosbag2_py); rerun_show uses `/tmp/rrviewer`.
