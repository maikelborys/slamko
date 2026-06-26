#!/usr/bin/env bash
# Step 2 of the cuVSLAM integration: run cuVSLAM ALONE on a clean casa bag (loop-closure
# OFF), record odometry, validate trajectory coherence (Hard Rule #5) BEFORE slamko trusts it.
# Uses CASA1_40cmH_848 (clean infra1/2 + imu + camera_info CONSISTENT @848, NO splitter).
# NOTE: CASA1_Suave is 640px IMAGES but 848 camera_info (recording mismatch) -> cuVSLAM rejects
# ("Image dimensions 640x480 do not correspond to camera resolution 848x480"); OKVIS doesn't
# notice (it ignores camera_info, uses its own config). Use a dimension-CONSISTENT bag for cuVSLAM.
BAG=${1:-/mnt/data/bno_ab/CASA1_40cmH_Stereo60_RGB30_BNO_848_trim}
RATE=${2:-0.5}
OUT=${3:-/tmp/cuvslam_casa_suave}
source /opt/ros/jazzy/setup.bash
source ~/coding/isaac_ros_ws/install/setup.bash
source ~/coding/slamko/install/setup.bash 2>/dev/null
rm -rf "$OUT"; mkdir -p "$OUT"

echo "[pre] reap stale procs"
pkill -KILL -f 'visual_slam|cuvslam_container|component_container|cuvslam_recorder|ros2 bag play' 2>/dev/null; sleep 2

echo "[1] launch cuVSLAM (odom-only, VIO) + D455 static TF"
ros2 launch ~/coding/slamko/scripts/cuvslam_casa.launch.py > "$OUT/cuvslam.log" 2>&1 &
sleep 9   # container + cuVSLAM init before frames

echo "[2] recorder"
python3 ~/coding/slamko/scripts/cuvslam_record_odom.py --out "$OUT" \
  --image-topic /camera/camera/infra1/image_rect_raw > "$OUT/recorder.log" 2>&1 &
RP=$!
sleep 2

echo "[3] play bag @rate=$RATE: $BAG"
ros2 bag play "$BAG" --rate "$RATE" --remap /tf_static:=/_unused_tf_static > "$OUT/play.log" 2>&1
echo "[4] bag done; settle 3s"; sleep 3

echo "[5] stop"
kill -INT $RP 2>/dev/null; sleep 3
pkill -KILL -f 'visual_slam|cuvslam_container|component_container' 2>/dev/null
wait $RP 2>/dev/null

echo "[6] coherence check (no GT)"
python3 ~/coding/slamko/scripts/cuvslam_traj_check.py --odom "$OUT/odom.csv" --out-prefix "$OUT/cuvslam" | tee "$OUT/coherence.txt"
grep -A4 'run summary' "$OUT/recorder.log" 2>/dev/null | head -5
echo "[done] $OUT"
