#!/usr/bin/env bash
# LIVE VOLUMETRIC run: the casa flashing bag -> OKVIS VIO -> slamko provider_fusion
# with the slamko_tsdf live volumetric layer ON (volumetric:=true). The D455 HW depth
# topic feeds a live nvblox TSDF that BENDS with the pose-graph; a Nav2 OccupancyGrid
# is published on ~/volumetric_costmap and a final mesh is exported.
#
# REQUIRES the GPU build:
#   colcon build --packages-select slamko_tsdf --cmake-args -DSLAMKO_WITH_NVBLOX=ON \
#       -DCMAKE_PREFIX_PATH=/home/maikel/ros2_ws/install/nvblox_ros
#   colcon build --packages-select slamko_ros --cmake-args \
#       -DCMAKE_PREFIX_PATH=/home/maikel/ros2_ws/install/nvblox_ros
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/coding/OKVIS2-X/install/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null          # libnvblox_lib.so on the runtime path
source ~/coding/slamko/install/setup.bash 2>/dev/null
export OMP_NUM_THREADS=2
BAG=/mnt/data/d455_bags/casa_084815_flashbno_trim
OUT=/tmp/slamko_casa_vol
CFG=/home/maikel/coding/OKVIS2-X/src/OKVIS2-X/config/rsD455_odom848
QOS=/home/maikel/coding/d455_setup/casa2_qos_override.yaml
rm -rf "$OUT"; mkdir -p "$OUT/map"
pkill -9 -f 'okvis2x|provider_fusion_node|d455_splitter_auto|cam_info_inject|bag play' 2>/dev/null
sleep 2

echo "[1] camera_info injector..."
python3 ~/coding/slamko/scripts/cam_info_inject_848.py > "$OUT/caminfo.log" 2>&1 &
echo "[2] content splitter (flashing -> clean /okvis/cam0,cam1)..."
python3 ~/coding/d455_setup/d455_splitter_auto.py > "$OUT/splitter.log" 2>&1 &
sleep 2

echo "[3] OKVIS VIO (rsD455_odom848)..."
ros2 run okvis okvis2x_stereo_network_node_subscriber --ros-args \
  -p config_filename:=$CFG/okvis2.yaml -p se_config_filename:=$CFG/se2.yaml \
  -p csv_path:=$OUT/okvis/ -p save_submap_meshes:=false \
  -p imu_propagated_state_publishing_rate:=80.0 \
  -r /okvis/imu0:=/camera/camera/imu \
  -r __ns:=/okvis -r __node:=okvis \
  > "$OUT/okvis.log" 2>&1 &
sleep 3

echo "[4] slamko provider_fusion + LIVE VOLUMETRIC (D455 HW depth -> nvblox TSDF)..."
ros2 run slamko_ros provider_fusion_node --ros-args \
  -p odom_topic:=/okvis/okvis_odometry \
  -p image_topic:=/okvis/cam0/image_raw \
  -p map_dir:=$OUT/map \
  -p traj_fused_path:=$OUT/fused.tum \
  -p traj_provider_path:=$OUT/provider.tum \
  -p traj_graph_path:=$OUT/graph.tum \
  -p traj_global_path:=$OUT/global.tum \
  -p imu_topic:=/camera/camera/imu \
  -p mag_topic:=/bno055/mag \
  -p kf_per_submap:=50 \
  -p volumetric:=true \
  -p depth_topic:=/camera/camera/depth/image_rect_raw \
  -p volumetric_voxel_m:=0.05 \
  -p volumetric_max_range_m:=5.0 \
  -p volumetric_correct_every:=10 \
  -p volumetric_mesh_path:=$OUT/volumetric_live.ply \
  > "$OUT/fusion.log" 2>&1 &
sleep 3

echo "[5] WAIT for node readiness (TRT engine build can take minutes) before bag..."
for i in $(seq 1 60); do
  grep -qiE 'reloc ready|provider_fusion_node up' "$OUT/fusion.log" 2>/dev/null && break
  sleep 5
done
echo "    node ready after ~$((i*5))s; +8s GPU settle"; sleep 8
echo "[6] playing bag @ rate 0.5..."
ros2 bag play "$BAG" --rate 0.5 \
  --qos-profile-overrides-path "$QOS" \
  --remap /tf_static:=/_unused_tf > "$OUT/play.log" 2>&1
echo "[7] bag done. settle 8s..."
sleep 8
echo "[8] shutdown OKVIS..."
ros2 service call /okvis/shutdown std_srvs/srv/SetBool "{data: true}" > "$OUT/shutdown.log" 2>&1 || true
sleep 5
echo "[8b] INT provider_fusion (triggers final volumetric bend + mesh export)..."
pkill -INT -f provider_fusion_node 2>/dev/null; sleep 6
pkill -9 -f 'okvis2x|provider_fusion_node|d455_splitter_auto|cam_info_inject' 2>/dev/null
echo "[9] DONE -> $OUT"
echo "--- fusion volumetric log ---"; grep -iE 'VOLUMETRIC|volumetric|nvblox|no-depth' "$OUT/fusion.log" | tail -20
echo "--- submaps: $(ls $OUT/map/submap_*.smap 2>/dev/null | wc -l) | graph.tum: $(wc -l < $OUT/graph.tum 2>/dev/null) ---"
echo "--- mesh: $(ls -la $OUT/volumetric_live.ply 2>/dev/null) ---"
