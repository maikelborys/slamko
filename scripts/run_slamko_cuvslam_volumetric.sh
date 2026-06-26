#!/usr/bin/env bash
# FULL stack on the DEPTH bag with LIVE Rerun: cuVSLAM VIO (provider) -> body-frame adapter ->
# slamko provider_fusion with BOTH maps (sparse XFeat + volumetric nvblox TSDF) + quality-break
# gate + live Rerun viz. Flashing bag -> splitter gives clean IR for cuVSLAM VIO + dotted depth
# for the volumetric reconstruction = full D455 power.
#   bag(flashing) -> cam_info_inject(848) + splitter -> /okvis/cam0,cam1(clean IR) + /nvblox/depth
#   cuVSLAM on clean IR -> /visual_slam/tracking/odometry -> body_tf -> /cuvslam/odometry_body
#   provider_fusion(odom=body, image=cam0, depth=/nvblox/depth, volumetric:=true, viz:=true)
# REQUIRES the Rerun viewer already running (DISPLAY=:0 rerun --port 9876).
BAG=${1:-/mnt/data/d455_bags/casa_084815_flashbno_trim}
RATE=${2:-0.5}
OUT=${3:-/tmp/slamko_cuvslam_vol}
source /opt/ros/jazzy/setup.bash
source ~/coding/isaac_ros_ws/install/setup.bash
source ~/coding/slamko/install/setup.bash 2>/dev/null
export OMP_NUM_THREADS=2
rm -rf "$OUT"; mkdir -p "$OUT/map"

echo "[pre] reap"
pkill -KILL -f 'visual_slam|cuvslam_container|component_container|robot_state_publisher|provider_fusion_node|cuvslam_body_tf|d455_splitter_auto|cam_info_inject|ros2 bag play' 2>/dev/null; sleep 2

echo "[1] camera_info injector (848)"
python3 ~/coding/slamko/scripts/cam_info_inject_848.py > "$OUT/caminfo.log" 2>&1 &
echo "[2] content splitter (flashing -> clean /okvis/cam0,cam1 + /nvblox/depth)"
python3 ~/coding/d455_setup/d455_splitter_auto.py > "$OUT/splitter.log" 2>&1 &
sleep 2

echo "[3] cuVSLAM VIO on the splitter's clean IR (+ D455 URDF TF, OKVIS-matched IMU noise)"
CUVSLAM_MODE=1 \
  CUVSLAM_IMG0=/okvis/cam0/image_raw CUVSLAM_IMG1=/okvis/cam1/image_raw \
  CUVSLAM_CI0=/camera/camera/infra1/camera_info CUVSLAM_CI1=/camera/camera/infra2/camera_info \
  ros2 launch ~/coding/slamko/scripts/cuvslam_casa.launch.py > "$OUT/cuvslam.log" 2>&1 &
sleep 9

echo "[3b] body-frame adapter -> /cuvslam/odometry_body"
python3 ~/coding/slamko/scripts/cuvslam_body_tf.py > "$OUT/bodytf.log" 2>&1 &
sleep 2

echo "[4] slamko provider_fusion: BOTH maps + quality-break + LIVE Rerun"
ros2 run slamko_ros provider_fusion_node --ros-args \
  -p odom_topic:=/cuvslam/odometry_body \
  -p image_topic:=/okvis/cam0/image_raw \
  -p imu_topic:=/camera/camera/imu \
  -p map_dir:="$OUT/map" \
  -p traj_fused_path:="$OUT/fused.tum" -p traj_provider_path:="$OUT/provider.tum" \
  -p traj_graph_path:="$OUT/graph.tum" \
  -p reloc:=true \
  -p atlas_break_on_quality:=true -p quality_soft_bridge:="${QSOFT:-true}" \
  -p quality_break_speed:="${QBREAK_SPEED:-4.0}" -p quality_break_jump:="${QBREAK_JUMP:-1.5}" \
  -p volumetric:=true \
  -p depth_topic:=/nvblox/depth/image_rect_raw \
  -p volumetric_voxel_m:=0.05 -p volumetric_max_range_m:=5.0 -p volumetric_correct_every:=10 \
  -p volumetric_mesh_path:="$OUT/volumetric_live.ply" \
  -p viz:=true \
  -p kf_per_submap:=50 \
  > "$OUT/fusion.log" 2>&1 &
sleep 4

echo "[5] GPU warm-up 12s (TRT/reloc + nvblox) before bag"
sleep 12
echo "[6] play DEPTH bag @rate=$RATE: $BAG"
ros2 bag play "$BAG" --rate "$RATE" --remap /tf_static:=/_unused_tf_static > "$OUT/play.log" 2>&1
echo "[7] bag done; settle 8s for final KFs + volumetric bend"; sleep 8

echo "[8] stop provider_fusion (seals + final volumetric mesh export)"
pkill -INT -f provider_fusion_node 2>/dev/null; sleep 5
pkill -KILL -f 'visual_slam|cuvslam_container|component_container|robot_state_publisher|provider_fusion_node|cuvslam_body_tf|d455_splitter_auto|cam_info_inject' 2>/dev/null

echo "[9] results"
echo "  components: $(tail -n +2 $OUT/map/components.csv 2>/dev/null | cut -d, -f2 | sort -u | wc -l) | submaps: $(ls $OUT/map/submap_*.smap 2>/dev/null | wc -l)"
grep -iE 'QUALITY LOST|ATLAS BREAK|FUSED|VOLUMETRIC|volumetric|nvblox|loop closure' "$OUT/fusion.log" 2>/dev/null | tail -15
echo "  mesh: $(ls -la $OUT/volumetric_live.ply 2>/dev/null | awk '{print $5}') bytes"
echo "[done] $OUT  (Rerun viewer kept live)"
