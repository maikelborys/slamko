#!/usr/bin/env bash
# cuVSLAM->slamko on a bag whose IMAGES are 848 but whose CAMERA_INFO is wrong (640) — the
# bno_ab CASA1_wall / CASA1_brutal recordings. cuVSLAM rejects a dim-mismatched camera_info,
# so we REMAP the bag's bad info away and inject the correct 848 info (cam_info_inject_848).
# Direct infra (NOT flashing → no splitter). P0 gates (catastrophic hard-break + IMU-shock) ON.
BAG=${1:?bag dir}
RATE=${2:-0.5}
OUT=${3:-/tmp/slamko_caminfo}
source /opt/ros/jazzy/setup.bash
source ~/coding/isaac_ros_ws/install/setup.bash
source ~/coding/slamko/install/setup.bash 2>/dev/null
rm -rf "$OUT"; mkdir -p "$OUT/map"

echo "[pre] reap"
pkill -KILL -f 'visual_slam|cuvslam_container|component_container|robot_state_publisher|provider_fusion_node|cuvslam_body_tf|cam_info_inject|ros2 bag play' 2>/dev/null; sleep 2

echo "[1] inject correct 848 camera_info (the bag's is 640 = dim-mismatch cuVSLAM rejects)"
python3 ~/coding/slamko/scripts/cam_info_inject_848.py > "$OUT/caminfo.log" 2>&1 &
sleep 1
echo "[2] cuVSLAM VIO (direct infra1/2) + D455 URDF TF + OKVIS-matched IMU noise"
CUVSLAM_MODE=1 ros2 launch ~/coding/slamko/scripts/cuvslam_casa.launch.py > "$OUT/cuvslam.log" 2>&1 &
sleep 9
echo "[2b] body-frame adapter -> /cuvslam/odometry_body"
python3 ~/coding/slamko/scripts/cuvslam_body_tf.py > "$OUT/bodytf.log" 2>&1 &
sleep 2

echo "[3] slamko provider_fusion (sparse; P0 catastrophic-break + IMU-shock ON)"
ros2 run slamko_ros provider_fusion_node --ros-args \
  -p odom_topic:=/cuvslam/odometry_body \
  -p image_topic:=/camera/camera/infra1/image_rect_raw \
  -p imu_topic:=/camera/camera/imu \
  -p map_dir:="$OUT/map" \
  -p traj_provider_path:="$OUT/provider.tum" -p traj_graph_path:="$OUT/graph.tum" \
  -p reloc:=true -p use_scan_context:=true \
  -p atlas_break_on_quality:=true -p quality_soft_bridge:="${QSOFT:-true}" \
  -p use_imu_shock:=true \
  -p kf_per_submap:=50 \
  > "$OUT/fusion.log" 2>&1 &
sleep 4

echo "[4] play bag @rate=$RATE (REMAP the bag's bad 640 camera_info away): $BAG"
ros2 bag play "$BAG" --rate "$RATE" \
  --remap /tf_static:=/_unused_tf_static \
          /camera/camera/infra1/camera_info:=/_unused_ci1 \
          /camera/camera/infra2/camera_info:=/_unused_ci2 \
  > "$OUT/play.log" 2>&1
echo "[5] bag done; settle 5s"; sleep 5
pkill -INT -f provider_fusion_node 2>/dev/null; sleep 4
pkill -KILL -f 'visual_slam|cuvslam_container|component_container|robot_state_publisher|provider_fusion_node|cuvslam_body_tf|cam_info_inject' 2>/dev/null

echo "[6] STABILITY results"
echo "  crash: $(grep -ciE 'Segmentation|what\(\)|loading shared' $OUT/fusion.log 2>/dev/null) | first-kf: $(grep -c 'first provider' $OUT/fusion.log 2>/dev/null)"
echo "  cuVSLAM |disp|max: $(python3 -c "import numpy as np;d=np.loadtxt('$OUT/provider.tum');p=d[:,1:4];print('%.1fm'%np.linalg.norm(p-p[0],axis=1).max())" 2>/dev/null||echo n/a)"
echo "  P0.1 catastrophic-breaks: $(grep -cE 'CATASTROPHIC divergence' $OUT/fusion.log 2>/dev/null) | P0.3 IMU-shocks: $(grep -cE 'IMU SHOCK' $OUT/fusion.log 2>/dev/null)"
echo "  QUALITY LOST/REC: $(grep -cE 'QUALITY LOST' $OUT/fusion.log 2>/dev/null)/$(grep -cE 'QUALITY RECOVERED' $OUT/fusion.log 2>/dev/null) | ATLAS BREAKs: $(grep -cE 'ATLAS BREAK' $OUT/fusion.log 2>/dev/null)"
echo "  components: $(tail -n +2 $OUT/map/components.csv 2>/dev/null|cut -d, -f2|sort -u|wc -l) | submaps: $(ls $OUT/map/submap_*.smap 2>/dev/null|wc -l)"
echo "[done] $OUT"
