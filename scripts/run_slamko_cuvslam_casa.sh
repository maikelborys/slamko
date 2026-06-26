#!/usr/bin/env bash
# THE integration: cuVSLAM (defects and all) -> slamko provider_fusion with the QUALITY-BREAK
# gate ON. The provider VIO WILL diverge/jump (that's a given); slamko's job is to be robust:
# detect the incoherent jump -> seal the good map -> break into a disjoint Atlas island ->
# weld back on a feature match (or dangle honestly). This validates the never-lost thesis on
# the REAL cuVSLAM provider. Sparse XFeat map first (40cmH clean 848 bag, no splitter/depth).
BAG=${1:-/mnt/data/bno_ab/CASA1_40cmH_Stereo60_RGB30_BNO_848_trim}
RATE=${2:-0.5}
OUT=${3:-/tmp/slamko_cuvslam_casa}
source /opt/ros/jazzy/setup.bash
source ~/coding/isaac_ros_ws/install/setup.bash
source ~/coding/slamko/install/setup.bash 2>/dev/null
# nvblox runtime lib (provider_fusion links slamko_tsdf → nvblox; the rebuilt binary needs
# the nvblox lib dir on the loader path since its RPATH isn't baked in).
export LD_LIBRARY_PATH=/home/maikel/ros2_ws/install/nvblox_ros/lib:${LD_LIBRARY_PATH}
rm -rf "$OUT"; mkdir -p "$OUT/map"

echo "[pre] reap"
pkill -KILL -f 'visual_slam|cuvslam_container|component_container|robot_state_publisher|provider_fusion_node|cuvslam_body_tf|ros2 bag play' 2>/dev/null; sleep 2

echo "[1] cuVSLAM (VIO, odom-only, corrected camera_link frames) + D455 URDF TF"
CUVSLAM_MODE=1 ros2 launch ~/coding/slamko/scripts/cuvslam_casa.launch.py > "$OUT/cuvslam.log" 2>&1 &
sleep 9

echo "[1b] body-frame adapter: camera_link pose -> camera_imu_optical_frame (slamko extrinsics correct)"
python3 ~/coding/slamko/scripts/cuvslam_body_tf.py > "$OUT/bodytf.log" 2>&1 &
sleep 2

echo "[2] slamko provider_fusion on cuVSLAM odom (body-frame), QUALITY-BREAK gate ON"
ros2 run slamko_ros provider_fusion_node --ros-args \
  -p odom_topic:=/cuvslam/odometry_body \
  -p image_topic:=/camera/camera/infra1/image_rect_raw \
  -p imu_topic:=/camera/camera/imu \
  -p map_dir:="$OUT/map" \
  -p traj_fused_path:="$OUT/fused.tum" \
  -p traj_provider_path:="$OUT/provider.tum" \
  -p traj_graph_path:="$OUT/graph.tum" \
  -p traj_slewed_path:="$OUT/traj_slewed.tum" \
  -p use_imu_shock:=true \
  -p reloc:=true \
  -p use_scan_context:="${SC:-false}" \
  -p sc_max_dist:="${SC_MAXD:-0.4}" \
  -p atlas_break_on_quality:=true \
  -p quality_break_speed:="${QBREAK_SPEED:-4.0}" \
  -p quality_break_jump:="${QBREAK_JUMP:-1.5}" \
  -p quality_soft_bridge:="${QSOFT:-false}" \
  -p viz:=${VIZ:-false} \
  -p kf_per_submap:=50 \
  > "$OUT/fusion.log" 2>&1 &
sleep 4

echo "[3] play bag @rate=$RATE: $BAG"
ros2 bag play "$BAG" --rate "$RATE" --remap /tf_static:=/_unused_tf_static > "$OUT/play.log" 2>&1
echo "[4] bag done; settle 6s"; sleep 6

echo "[5] stop provider_fusion (seals submaps + dumps)"
pkill -INT -f provider_fusion_node 2>/dev/null; sleep 4
pkill -KILL -f 'visual_slam|cuvslam_container|component_container|robot_state_publisher|provider_fusion_node|cuvslam_body_tf' 2>/dev/null

echo "[6] results"
echo "  graph.tum: $(wc -l < $OUT/graph.tum 2>/dev/null) | provider.tum: $(wc -l < $OUT/provider.tum 2>/dev/null) | submaps: $(ls $OUT/map/submap_*.smap 2>/dev/null | wc -l)"
echo "  QUALITY-BREAK / Atlas activity:"
grep -iE 'QUALITY LOST|QUALITY RECOVERED|ATLAS BREAK|component|FUSED|welded' "$OUT/fusion.log" 2>/dev/null | tail -12
echo "[done] $OUT"
