#!/usr/bin/env bash
# slamko LIVE on Isaac Sim. Assumes the Isaac bridge (bridge_isaac_okvis.py) is ALREADY
# running and publishing:
#   /camera/infra1/image_rect_raw + /camera/infra2/image_rect_raw  (stereo, 848x480)
#   /camera/imu_raw   (~200-580 Hz)   ->  imu_noise_relay -> /camera/imu  (OKVIS input)
#   /camera/infra1/depth  (32FC1)     ->  slamko volumetric nvblox TSDF
#   /clock, /isaac/ground_truth (GT), subscribes /cmd_vel
#
# Chain:  Isaac stereo+IMU -> OKVIS pure-VIO (okvis2_isaac.yaml, do_loop_closures=0)
#            -> /okvis/okvis_odometry  -> slamko provider_fusion (volumetric ON)
#            -> slamko map + ~/volumetric_costmap (global) + ~/local_costmap (dynamic)
#            -> slamko TF slamko_map->slamko_odom->slamko_base
# This is the PLAN_ISAACSIM step 2 (slamko on the Isaac stream). Nav2 goes on TOP after.
#
# Reaps ONLY the SLAM procs (okvis/slamko/imu_relay), NEVER the Isaac bridge.
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/coding/OKVIS2-X/install/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null          # libnvblox on runtime path
source ~/coding/slamko/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0 OMP_NUM_THREADS=2
R=~/coding/RTABmap/isaac
OUT=${OUT:-/tmp/slamko_isaac}; rm -rf "$OUT"; mkdir -p "$OUT/map"
CFG=${CFG:-$R/okvis2_isaac.yaml}       # already do_loop_closures=0 (odom-only for slamko)
SE=~/coding/cerebro_robot_sim/src/cerebro_robot/cerebro_okvis_config/config/d455_sim/se2.yaml

echo "[0] reap prior SLAM (NOT the bridge)..."
pkill -9 -f 'okvis2x_node_subscriber|provider_fusion_node|imu_noise_relay' 2>/dev/null; sleep 2
pgrep -f bridge_isaac_okvis >/dev/null || { echo "!! Isaac bridge NOT running — start it first"; exit 1; }

echo "[1] imu_noise_relay (/camera/imu_raw -> /camera/imu for OKVIS)..."
python3 "$R/imu_noise_relay.py" --ros-args -p use_sim_time:=true > "$OUT/imu_relay.log" 2>&1 &
sleep 2

echo "[2] OKVIS pure-VIO (okvis2x_node_subscriber, retry loop)..."
CFG="$CFG" SE="$SE" LOGD="$OUT" bash "$R/okvis_launch.sh" | sed 's/^/    /'

echo "[3] slamko provider_fusion + LIVE VOLUMETRIC..."
ros2 run slamko_ros provider_fusion_node --ros-args \
  -p odom_topic:=/okvis/okvis_odometry \
  -p image_topic:=/camera/infra1/image_rect_raw \
  -p imu_topic:=/camera/imu \
  -p map_dir:="$OUT/map" \
  -p traj_fused_path:="$OUT/fused.tum" \
  -p traj_provider_path:="$OUT/provider.tum" \
  -p traj_graph_path:="$OUT/graph.tum" \
  -p traj_global_path:="$OUT/global.tum" \
  -p kf_per_submap:=50 \
  -p volumetric:=true \
  -p depth_topic:=/camera/infra1/depth \
  -p depth_best_effort:=true \
  -p volumetric_voxel_m:=0.05 \
  -p volumetric_max_range_m:=3.5 \
  -p volumetric_correct_every:=10 \
  -p local_dynamic:=true \
  -p gate_live_pose:=true \
  -p volumetric_mesh_path:="$OUT/volumetric_live.ply" \
  > "$OUT/fusion.log" 2>&1 &
echo "    slamko pid=$!  logs -> $OUT/fusion.log"
echo
echo ">>> slamko LIVE on Isaac. Drive the robot (/cmd_vel) to build the map."
echo ">>> costmaps: ~/volumetric_costmap (global) + ~/local_costmap (dynamic)"
echo ">>> stop: pkill -INT -f provider_fusion_node ; pkill -9 -f 'okvis2x_node_subscriber|imu_noise_relay'"
