#!/usr/bin/env bash
# Flash-bag full chain on the cuVSLAM-Inertial provider (the run_slamko_casa_flashbag.sh
# recipe with OKVIS swapped out): bag -> content splitter (clean IR 45fps -> /okvis/cam0,
# cam1; dotted -> /nvblox/depth) + 848 caminfo inject -> cuvslam_provider_node (Inertial)
# -> /cuvslam/odometry -> provider_fusion (VPR/XFeat map + volumetric TSDF + costmaps).
#
# Safety: same zombie discipline as battery.sh — preflight + reap by name on teardown.
# First place to look: $OUT/cuvslam.log ("rig ready"), then $OUT/fusion.log ("reloc ready").
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null       # libnvblox_lib.so (S3 fix)
source ~/coding/slamko/install/setup.bash 2>/dev/null
export OMP_NUM_THREADS=2
BAG=${1:-/mnt/data/bags/d455_bags/casa_084815_flashbno_trim}
OUT=${2:-/tmp/slamko_casa_cuvslam}
RATE=${3:-0.5}
QOS=/home/maikel/coding/d455_setup/casa2_qos_override.yaml

PATTERN='[c]uvslam_provider_node|[p]rovider_fusion_node|[d]455_splitter_auto|[c]am_info_inject'
if pgrep -f "$PATTERN" > /dev/null; then
  echo "ABORT: stale processes:"; pgrep -af "$PATTERN"; exit 2
fi
rm -rf "$OUT"; mkdir -p "$OUT/map"

echo "[1] caminfo injector + content splitter"
python3 ~/coding/slamko/scripts/cam_info_inject_848.py > "$OUT/caminfo.log" 2>&1 &
python3 ~/coding/d455_setup/d455_splitter_auto.py > "$OUT/splitter.log" 2>&1 &
sleep 2

echo "[2] cuVSLAM provider (Inertial) on the splitter's clean IR"
ros2 run slamko_vio cuvslam_provider_node --ros-args \
  -p left_topic:=/okvis/cam0/image_raw -p right_topic:=/okvis/cam1/image_raw \
  -p use_imu:=true -p image_best_effort:=true \
  > "$OUT/cuvslam.log" 2>&1 &
sleep 2

echo "[3] provider_fusion: VPR map + volumetric TSDF + costmaps"
ros2 run slamko_ros provider_fusion_node --ros-args \
  -p odom_topic:=/cuvslam/odometry \
  -p image_topic:=/okvis/cam0/image_raw \
  -p map_dir:="$OUT/map" \
  -p traj_fused_path:="$OUT/fused.tum" -p traj_provider_path:="$OUT/provider.tum" \
  -p traj_graph_path:="$OUT/graph.tum" -p traj_global_path:="$OUT/global.tum" \
  -p imu_topic:=/camera/camera/imu -p mag_topic:=/bno055/mag \
  -p kf_per_submap:=50 \
  -p volumetric:=true -p depth_topic:=/nvblox/depth/image_rect_raw \
  -p depth_best_effort:=true -p volumetric_voxel_m:=0.05 -p volumetric_correct_every:=10 \
  -p volumetric_mesh_path:="$OUT/volumetric_live.ply" \
  -p atlas_break_on_loss:=true -p atlas_break_on_quality:=true -p quality_soft_bridge:=true \
  > "$OUT/fusion.log" 2>&1 &

echo "[4] wait for reloc ready (TRT warmup) before bag"
for _ in $(seq 1 60); do
  grep -qiE 'reloc ready|provider_fusion_node up' "$OUT/fusion.log" 2>/dev/null && break
  sleep 5
done
sleep 8

echo "[5] bag @ rate $RATE"
ros2 bag play "$BAG" --rate "$RATE" \
  --qos-profile-overrides-path "$QOS" \
  --remap /tf_static:=/_unused_tf > "$OUT/play.log" 2>&1
echo "[6] settle 10 s (final KFs + costmap capture at END of run)"
sleep 10
pkill -INT -f '[p]rovider_fusion_node' 2>/dev/null; sleep 4
pkill -KILL -f "$PATTERN" 2>/dev/null
echo "[7] DONE -> $OUT"
wc -l "$OUT"/*.tum 2>/dev/null
ls "$OUT"/map/submap_*.smap 2>/dev/null | wc -l
ls -la "$OUT"/volumetric_live.ply 2>/dev/null
