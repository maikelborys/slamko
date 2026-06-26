#!/usr/bin/env bash
# STRESS the slamko Atlas/never-jump/P0 machinery on the HARD bno_ab bags (wall, brutal) using
# the OKVIS provider — which runs NATIVE on the 640x480 bno_ab images (ignores camera_info),
# unlike cuVSLAM (dim-mismatch). The ideology lives in the slamko LAYER (never-lose: provider
# coasts on IMU; never-jump: map->odom slewed 0.5 m/s; Atlas: break-on-quality -> submap ->
# weld-on-feature), not in the provider — so we exercise it with the provider that actually runs.
#
# Reuses the PROVEN bno_ab OKVIS launch (~/coding/BNO055/ab/okvis_ab_c1_d455imu.launch.py) which
# owns the IMU relay + rsD455_map_odom config; OKVIS_CFG=rsD455_map_odom = pure-VIO odom (LC OFF,
# slamko owns loop closure). P0 gates ON: catastrophic hard-break + IMU-shock.
#   usage: stress_okvis_wallbrutal.sh <bag_dir> [rate] [out_dir]
set -u
BAG=${1:?bag dir}
RATE=${2:-0.5}
OUT=${3:-/tmp/slamko_stress_okvis}
LAUNCH=/home/maikel/coding/BNO055/ab/okvis_ab_c1_d455imu.launch.py
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/coding/OKVIS2-X/install/setup.bash 2>/dev/null
source ~/coding/slamko/install/setup.bash 2>/dev/null
export OMP_NUM_THREADS=2
rm -rf "$OUT"; mkdir -p "$OUT/map"

echo "[pre] reap any stale okvis/slamko/bag"
pkill -KILL -f 'okvis2x|okvis_ab_c1|provider_fusion_node|imu_relay|ros2 bag play|robot_state_publisher' 2>/dev/null
sleep 2

echo "[1] OKVIS provider (rsD455_map_odom pure-VIO) + IMU relay + bag@$RATE delayed 22s for warmup"
OKVIS_CFG=rsD455_map_odom ros2 launch "$LAUNCH" \
  bag_path:="$BAG" rate:="$RATE" bag_delay:=22.0 rviz:=false \
  csv_path:="$OUT/okvis/" > "$OUT/okvis.log" 2>&1 &
sleep 10

echo "[2] slamko provider_fusion (P0 catastrophic-break + IMU-shock ON; soft-bridge ON; reloc+scan_context)"
ros2 run slamko_ros provider_fusion_node --ros-args \
  -p odom_topic:=/okvis/okvis_odometry \
  -p image_topic:=/camera/camera/infra1/image_rect_raw \
  -p imu_topic:=/camera/camera/imu \
  -p mag_topic:=/bno055/mag \
  -p map_dir:="$OUT/map" \
  -p traj_provider_path:="$OUT/provider.tum" \
  -p traj_graph_path:="$OUT/graph.tum" \
  -p reloc:=true -p use_scan_context:=true \
  -p atlas_break_on_quality:=true -p quality_soft_bridge:="${QSOFT:-true}" \
  -p use_imu_shock:=true \
  -p kf_per_submap:=50 \
  > "$OUT/fusion.log" 2>&1 &

# bag plays at 22s; duration ~59s/rate + headroom
DUR=$(python3 -c "print(int(59.2/$RATE)+30)")
echo "[3] waiting bag (delay 22 + ~${DUR}s play+settle)..."
sleep $((22 + DUR))

echo "[4] shutdown OKVIS (dump final) + reap"
ros2 service call /okvis/shutdown std_srvs/srv/SetBool "{data: true}" > "$OUT/shutdown.log" 2>&1 || true
sleep 4
pkill -INT -f provider_fusion_node 2>/dev/null; sleep 4
pkill -KILL -f 'okvis2x|okvis_ab_c1|provider_fusion_node|imu_relay|robot_state_publisher' 2>/dev/null

echo "============ STABILITY REPORT: $(basename $BAG) @rate=$RATE ============"
echo "  crash(seg/throw/load): $(grep -ciE 'Segmentation|what\(\)|loading shared|core dumped' $OUT/fusion.log 2>/dev/null)"
echo "  first-kf seeded:       $(grep -c 'first provider' $OUT/fusion.log 2>/dev/null)"
echo "  provider |disp|max:    $(python3 -c "import numpy as np;d=np.loadtxt('$OUT/provider.tum');p=d[:,1:4];print('%.2fm'%np.linalg.norm(p-p[0],axis=1).max())" 2>/dev/null||echo n/a)"
echo "  provider.tum / graph.tum lines: $(wc -l < $OUT/provider.tum 2>/dev/null) / $(wc -l < $OUT/graph.tum 2>/dev/null)"
echo "  --- the Atlas / never-lost machinery firing ---"
echo "  QUALITY LOST/RECOVERED: $(grep -cE 'QUALITY LOST' $OUT/fusion.log 2>/dev/null)/$(grep -cE 'QUALITY RECOVERED' $OUT/fusion.log 2>/dev/null)"
echo "  ATLAS BREAKs:           $(grep -cE 'ATLAS BREAK|new island|seal.*branch' $OUT/fusion.log 2>/dev/null)"
echo "  P0.1 catastrophic:      $(grep -cE 'CATASTROPHIC' $OUT/fusion.log 2>/dev/null)"
echo "  P0.3 IMU-shocks:        $(grep -cE 'IMU SHOCK|IMU-SHOCK' $OUT/fusion.log 2>/dev/null)"
echo "  loop closures / welds:  $(grep -cE 'loop closure|WELD|weld' $OUT/fusion.log 2>/dev/null)"
echo "  --- coherence of the result ---"
echo "  components (islands):   $(tail -n +2 $OUT/map/components.csv 2>/dev/null|cut -d, -f2|sort -u|wc -l)"
echo "  submaps sealed:         $(ls $OUT/map/submap_*.smap 2>/dev/null|wc -l)"
echo "[done] $OUT  (logs: fusion.log okvis.log)"
