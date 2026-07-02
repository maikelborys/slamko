# PLAN_GAZEBO_SIM_01 — slamko on Gazebo Harmonic (the light sim that works)

<!-- authored 2026-07-01. Cold-start for continuing slamko-on-Gazebo. Companion to
PLAN_ISAACSIM_01. Session found Gazebo works where Isaac saturates the 8GB laptop. -->

## Why Gazebo (vs Isaac)

On this **RTX 4070 Laptop (8 GB)**, Isaac Sim + OKVIS + slamko-volumetric **saturate the GPU** → OKVIS
starves (13k frame drops) → slamko fragments the map (**344 tracking-loss, 338 tiny submaps**). Gazebo
Harmonic renders much lighter → OKVIS breathes → slamko maps **clean: 0 tracking-loss**, GPU ~4 GB/86%.
So **Gazebo is the viable closed-loop sim on this hardware**; Isaac needs a bigger GPU (or LOWRES).

The D455 stereo-VIO robot is **`~/coding/cerebro_robot_sim/`** (gz Harmonic 8.11), NOT `diff_bot_sim`
(that's the lidar/Nav2 robot with a single camera).

## Bring-up (60 fps stereo)

```bash
# 1) sim (house world) — sources jazzy + cerebro_robot_sim
source /opt/ros/jazzy/setup.bash && source ~/coding/cerebro_robot_sim/install/setup.bash
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
ros2 launch cerebro_bringup cerebro_spawn.launch.py world:=small_house_world.sdf rviz:=false
gz service -s /world/my_world/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: false'        # UNPAUSE (gz starts paused)

# 2) DEDICATED depth bridge — THE fix (see below)
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/tmp/isaac_drive/depth_bridge.yaml

# 3) depth 32FC1->16UC1 converter (slamko onDepth only takes 16UC1 mm)
python3 /tmp/isaac_drive/depth_convert.py        # /cerebro/depth/image_raw -> /cerebro/depth/image_16uc1

# 4) OKVIS + slamko volumetric
bash ~/coding/slamko/scripts/run_slamko_gazebo.sh   # (OKVIS d455_sim cfg + slamko; needs a /cmd_vel wiggle)
# then slamko volumetric params: depth_topic:=/cerebro/depth/image_16uc1
#   depth_fx:=446.7 depth_fy:=446.7 depth_cx:=423.5 depth_cy:=239.5 volumetric_voxel_m:=0.04 volumetric:=true
```

Topics: `/camera/infra1|2/image_rect_raw` (mono8 480×270 @60Hz RELIABLE), `/camera/imu` @200Hz,
`/cerebro/depth/image_raw` (32FC1 848×480 @15fps), `/odom_wheel`, `/cmd_vel`, `/clock`.

## The two load-bearing fixes this session

1. **DEDICATED depth bridge.** The main `parameter_bridge` runs ONE gz-transport reception thread; under
   the 60 fps stereo flood the heavy 848×480 float depth (1.55 MB/msg) hits the ZMQ high-water-mark and
   is **silently dropped** (gz-side publishes, ROS-side gets 0). A separate bridge process = own reception
   thread → depth flows at 60 fps. Depth entry was commented OUT of the main `ros_gz_bridge.yaml`.
2. **Depth 32FC1→16UC1.** gz `depth_camera` emits R_FLOAT32; slamko `onDepth` (`provider_fusion_node.cpp:1379`)
   only accepts 16UC1 mm. `depth_convert.py` bridges it. (Also: SDF `depth always_on 0→1`, and the
   camera+IMU moved forward+up to (0.20, 0.35) keeping T_SC valid.)

## Validated
Volumetric map: **0 tracking-loss**, house mesh 46789 verts, 1 fused component, GPU 4 GB/86%. Odometry
**27.8 Hz** — OKVIS caps ~28 fps (30 ms Ceres budget), so 60 fps in → ~half dropped = **benign designed
skip** (0 tracking-loss). Also confirmed offline: **slamko runs 1× realtime on bags** (0 tracking-loss;
the old rate-0.5 was NOT needed for coherence, only marginal density).

## Open items
- **LANDMARKS blocked (biggest open):** slamko CROPS images to 752 px wide (`provider_fusion_node.cpp:1708`
  `crop_x=(img_w-752)/2`) for its STATIC 752×480 XFeat ONNX → needs w≥752. Sim is 480 px → silent crash on
  camera_info (relocalizer build). Workaround shipped = **upscale relay 480→848** (`upscale_hd.py`): stops
  the crash, XFeat/reloc runs (cosine 1.00), BUT **landmarks still 0** — triangulation yields nothing.
  Hypotheses to test next: (a) slamko needs the stereo RIGHT image too (only left is fed); (b) bilinear
  upscale blurs out XFeat corners; (c) a stereo-vs-mono-multiview triangulation requirement. Until landmarks
  work, only the VOLUMETRIC (depth) map is available on Gazebo — which is fine for a dense map.
- **Volumetric quality levers** (from investigation): voxel 0.03 (clean sim depth), correct_every 5,
  store_budget 0, keep_recent 80, mesh_path set. Verify `depth_extrinsic_xyz` matches the moved camera.
- **Nav2 closed loop** on slamko's `~/volumetric_costmap` + `~/local_costmap` (PLAN_NAV2_01) — the endgame.
- 60 vs 30 fps: OKVIS caps ~28 fps regardless; 30 fps stereo would halve the (benign) drops.

## Scripts (session, in /tmp/isaac_drive/ — promote the keepers to slamko/scripts)
`depth_bridge.yaml` (dedicated depth bridge cfg) · `depth_convert.py` (32FC1→16UC1) ·
`upscale_hd.py` (480→848 for XFeat) · `cam_info_gz.py` (480 stereo info) · `gz_drive.py` (circle drive) ·
`run_landmarks.sh` (full landmark pipeline orchestrator). In slamko/scripts: `run_slamko_gazebo.sh`.
```
```
Gotcha: the harness emits spurious "exit 1" on foreground python from /tmp — launch via a background
task or one orchestrator script.
```
