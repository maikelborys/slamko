#!/usr/bin/env bash
# slamko LIVE on the Gazebo (cerebro_robot_sim) D455 sim. Assumes the sim is ALREADY
# running + UNPAUSED (cerebro_spawn.launch.py world:=small_house_world.sdf), publishing:
#   /camera/infra1/image_rect_raw + /camera/infra2/image_rect_raw  (L8 480x270 ~53Hz, RELIABLE)
#   /camera/imu (~168Hz)  ·  /odom_wheel  ·  /cmd_vel  ·  /clock
# Chain: sim stereo+IMU -> OKVIS (d455_sim/okvis2.yaml, VIO-only) -> /okvis/okvis_odometry
#          -> slamko provider_fusion (SPARSE — sim depth is disabled) -> map + submap costmap
# NOTE: no depth in the sim (camera_depth always_on=0, gz8.11 bug) -> volumetric OFF.
# Reaps only SLAM procs, never the gz sim.
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/coding/cerebro_robot_sim/install/setup.bash 2>/dev/null
source ~/coding/OKVIS2-X/install/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null          # libnvblox_lib.so on runtime path
source ~/coding/slamko/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0 OMP_NUM_THREADS=2
CFG=~/coding/cerebro_robot_sim/src/cerebro_robot/cerebro_okvis_config/config/d455_sim
OUT=${OUT:-/tmp/slamko_gz}; rm -rf "$OUT"; mkdir -p "$OUT/map"

echo "[0] reap prior SLAM (NOT the sim)..."
pkill -9 -f 'okvis2x_stereo_network_node_subscriber|okvis2x_node_subscriber|provider_fusion_node' 2>/dev/null; sleep 2
pgrep -f 'gz sim' >/dev/null || { echo "!! Gazebo sim NOT running — start cerebro_spawn first"; exit 1; }

echo "[1] OKVIS VIO (d455_sim/okvis2.yaml, sim topics)..."
ros2 run okvis okvis2x_stereo_network_node_subscriber --ros-args \
  -p config_filename:=$CFG/okvis2.yaml -p se_config_filename:=$CFG/se2.yaml \
  -p csv_path:=$OUT/okvis/ -p save_submap_meshes:=false \
  -p imu_propagated_state_publishing_rate:=50.0 -p use_sim_time:=true \
  -r __ns:=/okvis -r __node:=okvis \
  -r /okvis/cam0/image_raw:=/camera/infra1/image_rect_raw \
  -r /okvis/cam1/image_raw:=/camera/infra2/image_rect_raw \
  -r /okvis/imu0:=/camera/imu \
  > "$OUT/okvis.log" 2>&1 &
echo "    okvis pid=$!  waiting for init..."
for s in $(seq 1 20); do sleep 1; grep -q 'Initialized!' "$OUT/okvis.log" 2>/dev/null && { echo "    OKVIS init OK (${s}s)"; break; }; done
grep -q 'Initialized!' "$OUT/okvis.log" 2>/dev/null || echo "    !! OKVIS not initialized yet (needs motion — drive the robot)"

echo "[2] slamko provider_fusion (SPARSE)..."
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
  -p volumetric:=false \
  > "$OUT/fusion.log" 2>&1 &
echo "    slamko pid=$!  logs -> $OUT/fusion.log"
echo
echo ">>> slamko LIVE on Gazebo. Drive the robot (/cmd_vel) to build the map."
echo ">>> stop: pkill -INT -f provider_fusion_node ; pkill -9 -f okvis2x_stereo_network_node_subscriber"
