#!/usr/bin/env bash
# Two-pass ZERO-CONTENTION mapping (the production recipe + the decisive
# experiment for "does slamko enlarge the map or does the co-run degrade OKVIS"):
#   PASS 1: OKVIS runs ALONE on the bag (reference-grade odometry, no slamko
#           inference stealing the GPU) while `ros2 bag record` captures
#           /okvis/okvis_odometry.
#   PASS 2: OKVIS is NOT running. The image bag + the recorded odometry bag are
#           replayed together; provider_fusion_node builds the full map (VPR +
#           landmarks + loops + anchors) with the whole GPU to itself.
#
#   scripts/map_two_pass.sh <image_bag> <out_dir> [rate1] [rate2]
#
# Sync note: both players replay ORIGINAL header stamps; the fusion node
# associates by header time (2.5 s image ring buffer), so a sub-second start
# offset between the two players is harmless.
cd "$(dirname "$0")/.."

BAG=${1:?image bag}
OUT=${2:?out dir}
RATE1=${3:-1.0}
RATE2=${4:-1.0}
PATTERN='^[^ ]*(okvis2x_stereo_network_node_subscriber|provider_fusion_node)'

if pgrep -af "$PATTERN" > /dev/null; then
  echo "ABORT: stale provider/fusion process running:"; pgrep -af "$PATTERN"; exit 2
fi
mkdir -p "$OUT"
source /opt/ros/jazzy/setup.bash
source ~/coding/OKVIS2-X/install/setup.bash
source install/setup.bash
QOS=~/coding/d455_setup/casa2_qos_override.yaml

echo "== PASS 1: OKVIS alone (rate $RATE1), recording odometry"
rm -rf "$OUT/odom_bag"
setsid ros2 bag record /okvis/okvis_odometry -o "$OUT/odom_bag" > "$OUT/record.log" 2>&1 &
REC_PID=$!
setsid ros2 launch ~/coding/d455_setup/okvis_d455_casa2_odom.launch.py \
  bag_path:="$BAG" rate:="$RATE1" rviz:=false csv_path:="$OUT/okvis/" \
  > "$OUT/pass1.log" 2>&1 &
P1_PID=$!
sleep 8; P1_PIDS=$(pgrep -f "$PATTERN" | tr '\n' ' ')
elapsed=0
while [ $elapsed -lt 30 ] && ! pgrep -f "ros2 bag play $BAG" > /dev/null; do sleep 2; elapsed=$((elapsed+2)); done
while pgrep -f "ros2 bag play $BAG" > /dev/null; do sleep 5; done
sleep 5
kill -INT -- -"$REC_PID" 2>/dev/null   # recorder first: let it write metadata
sleep 4
kill -INT -- -"$P1_PID" 2>/dev/null
for _ in 1 2 3 4 5 6 7 8; do pgrep -f "$PATTERN" > /dev/null || break; sleep 1; done
kill -- -"$P1_PID" 2>/dev/null
for pid in $P1_PIDS; do kill -9 "$pid" 2>/dev/null; done
sleep 2
ros2 bag info "$OUT/odom_bag" > /dev/null 2>&1 || ros2 bag reindex "$OUT/odom_bag" > /dev/null 2>&1
echo "pass 1 done: $(ros2 bag info "$OUT/odom_bag" 2>/dev/null | grep -m1 Messages || echo 'RECORD FAILED')"

echo "== PASS 2: fusion only (rate $RATE2), OKVIS off"
setsid ros2 run slamko_ros provider_fusion_node --ros-args \
  -p odom_topic:=/okvis/okvis_odometry \
  -p image_topic:=/camera/camera/infra1/image_rect_raw \
  -p map_dir:="$PWD/$OUT/map" \
  -p traj_fused_path:="$PWD/$OUT/fused.tum" \
  -p traj_provider_path:="$PWD/$OUT/provider.tum" \
  -p traj_graph_path:="$PWD/$OUT/graph.tum" \
  > "$OUT/pass2.log" 2>&1 &
P2_PID=$!
sleep 12   # engines deserialize
setsid ros2 bag play "$BAG" --rate "$RATE2" --qos-profile-overrides-path "$QOS" \
  --remap /tf_static:=/_bag_tf_static_unused > /dev/null 2>&1 &
IMG_PID=$!
sleep 0.5
setsid ros2 bag play "$OUT/odom_bag" --rate "$RATE2" > /dev/null 2>&1 &
ODO_PID=$!
while pgrep -f "ros2 bag play" > /dev/null; do sleep 5; done
sleep 8
kill -INT -- -"$P2_PID" 2>/dev/null
for _ in 1 2 3 4 5 6 7 8 9 10; do pgrep -f '^[^ ]*provider_fusion_node' > /dev/null || break; sleep 1; done
kill -- -"$P2_PID" -"$IMG_PID" -"$ODO_PID" 2>/dev/null
pgrep -f '^[^ ]*provider_fusion_node' | xargs -r kill -9 2>/dev/null
echo "== done: map=$OUT/map graph=$OUT/graph.tum (loops: $(grep -c 'LOOP CLOSED' "$OUT/pass2.log"))"
