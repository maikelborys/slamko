#!/usr/bin/env bash
# Experiment 7-A driver: cuVSLAM (Isaac visual_slam) on an EuRoC seq -> record pose+cov
# + input-frame rate, then run NEES vs GT. EuRoC has ground truth -> a VALID covariance
# calibration (casa bags have none). Serial: never run alongside another GPU SLAM.
SEQ=${1:-/mnt/data/datasets/euroc/MH_03_medium}
RATE=${2:-1.0}
OUT=${3:-/tmp/cuvslam_nees}
GT=${4:-/mnt/data/results/euroc_battle/mh_03_gt_tum.txt}
source /opt/ros/jazzy/setup.bash
source ~/coding/isaac_ros_ws/install/setup.bash
source ~/coding/slamko/install/setup.bash 2>/dev/null
rm -rf "$OUT"; mkdir -p "$OUT"

echo "[pre] reaping any stale cuvslam/euroc procs"
pkill -KILL -f 'visual_slam|euroc_player|component_container|cuvslam_recorder' 2>/dev/null; sleep 2

echo "[1] launch cuVSLAM (VIO) + euroc_player @rate=$RATE seq=$SEQ"
ros2 launch euroc_publisher cuvslam_euroc.launch.py seq:="$SEQ" rate:="$RATE" tracking_mode:=1 \
  > "$OUT/cuvslam.log" 2>&1 &
LP=$!
sleep 8   # container + node warmup before frames flow

echo "[2] start recorder"
python3 ~/coding/slamko/scripts/cuvslam_record_odom.py --out "$OUT" \
  --image-topic /euroc/left/image_rect_raw > "$OUT/recorder.log" 2>&1 &
RP=$!

echo "[3] waiting for euroc_player to finish..."
for i in $(seq 1 400); do
  pgrep -f 'euroc_player' >/dev/null || { echo "[3] player done @ ${i}s"; break; }
  sleep 1
done
sleep 3   # flush last odom

echo "[4] stop recorder (SIGINT -> prints summary)"
kill -INT $RP 2>/dev/null; sleep 3
pkill -KILL -f 'visual_slam|component_container|euroc_player' 2>/dev/null
wait $RP 2>/dev/null

echo "[5] NEES vs GT"
python3 ~/coding/slamko/scripts/cuvslam_nees.py --odom "$OUT/odom.csv" --gt "$GT" --lag 10 | tee "$OUT/nees.txt"
echo "[done] artifacts in $OUT"
grep -iE 'summary|fps|dropped|input|odometry poses' "$OUT/recorder.log" 2>/dev/null | tail -5
