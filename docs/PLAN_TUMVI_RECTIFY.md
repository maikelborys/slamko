# PLAN — TUM VI fisheye→pinhole rectification + first mapping test (then multi-part merge)

<!-- status: STEP 1 DONE (room1 maps green) · magistrale rectified · 2026-05-27 -->

## PROGRESS (2026-05-27)

**Step 1 (room1 gate) — DONE, green.** Built `scripts/rectify_tumvi.py` (fisheye→pinhole,
CLAHE, drop-in `mav0`) + `scripts/plot_slamko.py` (interactive Plotly 3D viz, the new
default). Rectified `room1` → `/mnt/data/datasets/tumvi_rect/room1`; slamko_vio (XFeat)
tracks all 2821 frames into a coherent room map (`active≈1400`, `stereo≈1300`), roughly
metric (pathlen 157 vs GT 147 m). **Sim3-ATE 69 cm / SE3-ATE 92 cm, drift mostly vertical
(z-RMSE 71 cm)** — pure odometry, no loop closure; maps correctly (the gate). Full
process + the new-bag reference: `/mnt/data/datasets/tumvi_rect/README.md`.

**Two lessons that cost time (now fixed in the rectifier + launch):**
1. **XFeat ONNX is static-shape `[1,1,480,752]`** — a 512×512 input resized in-net to
   752×480 (1.56:1 aspect distortion) → the keypoint head returns **0** keypoints. Fix:
   rectify straight to **752×480** (XFeat-native, no in-net resize). Shi-Tomasi tolerates
   any size + the dark frames; XFeat needs native size **and** CLAHE. (Shi-Tomasi was the
   discriminator: it tracked room1 fine while XFeat gave `active=0` → isolated the bug to
   XFeat, not the rectification/geometry/IMU.)
2. **Launch hardcoded `image_width/height=752/480`** and didn't expose them — the node
   silently drops every frame on a size mismatch. Added `image_width/image_height` +
   `xfeat_keypoint_threshold` launch args (with `value_type` coercion).

**Next:** magistrale1/2 rectified → run single-session, then the map-by-parts + cross-
session weld (the user's real goal). Drift seen on room1 is exactly what the never-lost
loop-closure/welding path is for.

---


**Read first:** [`SYSTEM.md`](SYSTEM.md) (status map) + [`../CLAUDE.md`](../CLAUDE.md)
(rules: zombie guard, run benches SERIALLY) + `slamko_loop/docs/STATUS.md` (P4b
cross-session, P3 BoW — already shipped). This plan is self-contained so a fresh
session can execute it without re-deriving.

## Goal (in order)
1. **Rectify ONE TUM VI sequence (`room1`) fisheye→pinhole**, run slamko_vio on it,
   and **validate it maps correctly: visual check + Sim3-ATE vs ground truth.** This
   is the gate — if rectification + VIO is sound on the easy single-room sequence,
   scale up. (User: "primero probar rectificar uno y a ver si se mapea correcto, ver
   visualmente si está bien, cuál ATE, etc.")
2. Then **magistrale1** (multi-floor continuous traversal) single-session map.
3. Then the real target: **map-by-parts + fuse** — Atlas of magistrale1 as prior,
   weld magistrale2 into it (same building, overlapping → cross-session weld via the
   P4b machinery already validated on EuRoC V1_01→V1_02).

## Why this is needed (the blocker)
TUM VI cameras are **equidistant fisheye** (512×512, ~195° FOV). slamko_vio is
**rectified pinhole** — its KLT + stereo + PnP project with `u = fx·X/Z + cx`, no
fisheye model. So raw TUM VI breaks. Fix = an **offline rectification preprocessor**
that undistorts fisheye → a pinhole virtual stereo pair, written as a standard EuRoC
`mav0`, then replayed by the existing `euroc_player` + `vio_euroc.launch.py` (which
already work on pinhole EuRoC). This is contained — it does NOT touch the validated
VIO pipeline (the alternative, native equidistant in VIO, was rejected as too invasive).

## Conceptual answer to "map by parts → fuse?" (already explained to user)
Yes — that's the Atlas / cross-session model (P4b, validated). BUT fusion happens by
**place recognition** (XFeat + geometric verify): two parts only connect **where they
see the same place**. Different floors share no view → cannot auto-fuse as separate
maps; the only automatic floor-to-floor link is a **continuous traversal** (camera
goes up stairs/slide → continuous odometry, submaps chain by the pose graph). TUM VI
`magistrale` sequences ARE continuous multi-floor traversals, and magistrale1–6 are 6
overlapping runs of the SAME building → ideal multi-part test once fisheye is solved.
Disjoint per-floor recordings with no overlap/transition need a manual transform
(map stitching) — not supported, place-rec can't recover it.

## Dataset (local, confirmed present)
- `/mnt/data/datasets/tumvi/dataset-room1_512_16/` ... `room6`, `magistrale1`..`magistrale6`.
- Layout: `mav0/{cam0,cam1,imu0,mocap0}`. Images: PNG 512×512 (room1: 2821 frames @20Hz).
- GT: `mav0/mocap0/data.csv`, header `#ts[ns], p_x,p_y,p_z, q_w,q_x,q_y,q_z` (body/IMU frame).
- IMU: BMI160 200Hz — gyro_noise 0.00016, gyro_rw 2.2e-5, accel_noise 0.0028, accel_rw 0.00086.
- A prior `okvis_tumvi.launch.py` exists in `~/coding/isaac_ros_ws/src/euroc_publisher/`
  (OKVIS rectifies internally) — reuse the image/timestamp reading pattern if helpful.

### Calibration (from `cam0/sensor.yaml`, `cam1/sensor.yaml` — same across sequences)
- **cam0**: equidistant. intrinsics `fx=190.97847715, fy=190.97330705, cx=254.93170606, cy=256.89744290`.
  distortion `k=[0.0034823894, 0.0007150348, -0.0020532361, 0.0002029367]`.
  `T_BS` (4×4 row-major, sensor-in-body i.e. cam0-in-IMU):
  `[-0.99952504, 0.00750192, -0.02989013, 0.04557484; 0.02961534, -0.03439736, -0.99896935, -0.07116180; -0.00852233, -0.99938008, 0.03415885, -0.04468125; 0,0,0,1]`
- **cam1**: intrinsics `fx=190.44236969, fy=190.43443847, cx=252.59949717, cy=254.91723065`.
  distortion `k=[0.0034003171, 0.0017662782, -0.0026631257, 0.0003299517]`.
  `T_BS` (cam1-in-IMU):
  `[-0.99951105, 0.00810408, -0.03019914, -0.05545634; 0.03029912, 0.01251164, -0.99946257, -0.06925002; -0.00772188, -0.99988889, -0.01275107, -0.04745286; 0,0,0,1]`
- imu0 `T_BS` = identity (IMU is the body frame).
- Stereo baseline ≈ |cam0.t − cam1.t| ≈ 0.10 m (tx 0.0456 vs −0.0555). `T_cam0_cam1 = inv(T_BS_cam0)·T_BS_cam1`.

## Approach — `scripts/rectify_tumvi.py` (Python + OpenCV)
1. Args: `--seq <tumvi dir> --out <rect dir> [--balance 0.3] [--fov-scale 1.0]`.
2. Parse cam0/cam1 sensor.yaml (intrinsics K, equidistant D, T_BS). Compute `T_cam0_cam1`.
3. **Stereo-rectify the fisheye pair:** `cv2.fisheye.stereoRectify(K0,D0,K1,D1,size,R,T, flags=cv2.CALIB_ZERO_DISPARITY, balance=..., fov_scale=...)` → R1,R2,P1,P2,Q. Then
   `cv2.fisheye.initUndistortRectifyMap` per cam → remap maps. (Output a **pinhole**
   virtual stereo pair, row-aligned, horizontal epipolar — what slamko's stereo needs.)
   - `balance`/`fov_scale` trade FOV vs edge stretch. Start balance≈0.3; tune by eye
     (straight lines straight, no black corners eating the image, enough texture).
   - Pick rectified size (keep 512×512 or crop). New pinhole intrinsics come from P1
     (fx,fy,cx,cy), distortion = 0.
4. Remap every cam0/cam1 frame → write PNGs to `<out>/mav0/cam0|cam1/data/` (same
   timestamps/filenames). Copy `imu0/` and `mocap0/` verbatim.
5. Write rectified `sensor.yaml` (camera_model pinhole, **radtan/zero distortion**, new
   P1 intrinsics, resolution) so the player/camera_info advertise pinhole. **T_BS for
   the rectified virtual cam = `T_BS_cam0 · inv(R1_se3)`** (the rectification rotation
   R1 rotates the cam frame; get this right or IMU/gravity init is wrong). cam1 similarly
   with R2; baseline from P2[0,3]/-fx.
6. Output a drop-in EuRoC `mav0` at e.g. `/mnt/data/datasets/tumvi_rect/room1/`.

## Run slamko_vio on the rectified sequence
```bash
# (after rectify_tumvi.py produced /mnt/data/datasets/tumvi_rect/room1)
source /opt/ros/jazzy/setup.bash; source ~/coding/isaac_ros_ws/install/setup.bash; source ~/coding/slamko/install/setup.bash
ros2 launch slamko_vio vio_euroc.launch.py \
  seq:=/mnt/data/datasets/tumvi_rect/room1 rate:=1.0 feature_source:=xfeat \
  image_width:=<RECT_W> image_height:=<RECT_H> \
  enable_neverlost:=true neverlost_use_pose_graph:=true dr_enabled:=true \
  pose_dump_path:=/tmp/tumvi_room1.tum landmark_dump_path:=/tmp/tumvi_room1_lm.csv
```
- **Check the player publishes the rectified pinhole `camera_info` + static TF** (it
  reads intrinsics from where? verify — may need a rectified sensor.yaml it consumes,
  or pass intrinsics as args). slamko resolves T_BS from TF (`resolved T_BS` log line).
- If `image_width/height` aren't already node params, they ARE (`vio_config.hpp`).

## Validate (the gate for step 1)
1. **Visual:** save a sample rectified stereo pair + overlay → straight lines straight,
   rows aligned. Reject if warped/black-cornered (re-tune balance/fov-scale).
2. Convert GT: `mocap0/data.csv` → TUM (`t[s] tx ty tz qx qy qz qw`; ts_ns/1e9; reorder
   q_w,q_x,q_y,q_z → qx,qy,qz,qw). Save `/tmp/tumvi_room1_gt.tum`.
3. `scripts/plot_neverlost.py --gt /tmp/tumvi_room1_gt.tum --est /tmp/tumvi_room1.tum
   --landmarks /tmp/tumvi_room1_lm.csv --out /tmp/tumvi_room1.png` → Sim3-ATE + map.
4. **Judge:** does it track the whole sequence? Is the map a coherent room? ATE
   ballpark (TUM VI room VIO is typically a few cm–tens of cm; we care it MAPS, not SOTA
   accuracy — robustness-over-accuracy memory). Send the plot to the user.

## Risks / open questions (resolve while doing it)
- **FOV crop**: 195°→pinhole loses periphery; balance/fov_scale tuning is the main knob.
- **left/right ordering**: confirm cam0=left, cam1=right and disparity is +x after rect
  (slamko stereo assumes horizontal). Swap if needed.
- **T_BS of the rectified cam** (R1 rotation) — the #1 thing to get exactly right; verify
  via the gravity-init log (`calibrated gravity_w` should be ~[0,9.81,*] sane) + that VIO
  doesn't diverge immediately.
- **Zombie/serial discipline** (CLAUDE.md): reap `slamko_vio_node|euroc_player` before/
  after by name, run ONE at a time. `scripts/bench_neverlost.sh` has the trap pattern.
- Rooms have mocap GT only inside the mocap volume (TUM VI GT has gaps); ATE uses
  associated poses only (plot_neverlost already timestamp-associates).

## Definition of done (step 1)
A rectified `room1` that slamko_vio tracks end-to-end, a plot showing the trajectory
tracking GT + a coherent landmark map, an ATE number, and the visual rectification
check — all sent to the user. Then proceed to magistrale single → magistrale multi-part
cross-session merge (the user's real goal). Update SYSTEM.md + a STATUS entry on green.
