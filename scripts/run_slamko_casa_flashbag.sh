#!/usr/bin/env bash
# Run slamko (OKVIS VIO + provider_fusion VPR/XFeat mapping) on the FLASHING casa bag.
# bag -> python content-splitter -> clean /okvis/cam0,cam1 -> OKVIS -> /okvis/okvis_odometry
#     -> slamko provider_fusion (vpr on) -> graph.tum + .smap submaps + landmarks.
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/coding/OKVIS2-X/install/setup.bash 2>/dev/null
source ~/coding/slamko/install/setup.bash 2>/dev/null
export OMP_NUM_THREADS=2
BAG=/mnt/data/d455_bags/casa_084815_flashbno_trim
OUT=/tmp/slamko_casa
CFG=/home/maikel/coding/OKVIS2-X/src/OKVIS2-X/config/rsD455_odom848
QOS=/home/maikel/coding/d455_setup/casa2_qos_override.yaml
rm -rf "$OUT"; mkdir -p "$OUT/map"
pkill -9 -f 'okvis2x|provider_fusion_node|d455_splitter_auto|cam_info_pub|bag play' 2>/dev/null
sleep 2

echo "[1] camera_info injector..."
python3 ~/coding/slamko/scripts/cam_info_inject_848.py > "$OUT/caminfo.log" 2>&1 &
echo "[2] content splitter (flashing -> clean /okvis/cam0,cam1 + /nvblox/depth)..."
python3 ~/coding/d455_setup/d455_splitter_auto.py > "$OUT/splitter.log" 2>&1 &
sleep 2

echo "[3] OKVIS VIO (rsD455_odom848, reads splitter /okvis/cam0,cam1, imu0<-/camera/camera/imu)..."
ros2 run okvis okvis2x_stereo_network_node_subscriber --ros-args \
  -p config_filename:=$CFG/okvis2.yaml -p se_config_filename:=$CFG/se2.yaml \
  -p csv_path:=$OUT/okvis/ -p save_submap_meshes:=false \
  -p imu_propagated_state_publishing_rate:=80.0 \
  -r /okvis/imu0:=/camera/camera/imu \
  -r __ns:=/okvis -r __node:=okvis \
  > "$OUT/okvis.log" 2>&1 &
sleep 3

echo "[4] slamko provider_fusion (VPR/XFeat mapping)..."
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
  > "$OUT/fusion.log" 2>&1 &
sleep 3

echo "[5] GPU warm-up 20s before bag..."
sleep 20
echo "[6] playing bag @ rate 0.5 (avoid OKVIS<->XFeat GPU contention)..."
ros2 bag play "$BAG" --rate 0.5 \
  --qos-profile-overrides-path "$QOS" \
  --remap /tf_static:=/_unused_tf > "$OUT/play.log" 2>&1
echo "[7] bag done. settle 8s for final KFs..."
sleep 8
echo "[8] shutdown OKVIS (dump final traj+map)..."
ros2 service call /okvis/shutdown std_srvs/srv/SetBool "{data: true}" > "$OUT/shutdown.log" 2>&1 || true
sleep 5
pkill -INT -f provider_fusion_node 2>/dev/null; sleep 3
pkill -9 -f 'okvis2x|provider_fusion_node|d455_splitter_auto|cam_info_pub' 2>/dev/null
echo "[9] DONE -> $OUT"
echo "--- outputs ---"; ls -la "$OUT"/*.tum "$OUT"/map/*.smap 2>/dev/null | tail -20
echo "--- provider.tum lines: $(wc -l < $OUT/provider.tum 2>/dev/null) ---"
echo "--- graph.tum lines: $(wc -l < $OUT/graph.tum 2>/dev/null) ---"
echo "--- submaps: $(ls $OUT/map/submap_*.smap 2>/dev/null | wc -l) ---"
