#!/usr/bin/env bash
# Pass 2 of the two-pass mapping (see map_two_pass.sh): OKVIS off, replay the
# image bag + the recorded odometry bag into provider_fusion_node.
#   scripts/map_pass2.sh <image_bag> <out_dir_with_odom_bag> [rate]
cd "$(dirname "$0")/.."
BAG=${1:?image bag}
OUT=${2:?out dir containing odom_bag/}
RATE=${3:-1.0}

pgrep -f '^[^ ]*(okvis2x_stereo|provider_fusion_node)' > /dev/null && {
  echo "ABORT: stale node running"; pgrep -af '^[^ ]*(okvis2x_stereo|provider_fusion_node)'; exit 2; }
source /opt/ros/jazzy/setup.bash
source install/setup.bash
QOS=~/coding/d455_setup/casa2_qos_override.yaml
rm -f "$OUT"/{provider,fused,graph,global}.tum; rm -rf "$OUT/map"

setsid ros2 run slamko_ros provider_fusion_node --ros-args \
  -p odom_topic:=/okvis/okvis_odometry \
  -p image_topic:=/camera/camera/infra1/image_rect_raw \
  -p map_dir:="$PWD/$OUT/map" \
  -p traj_fused_path:="$PWD/$OUT/fused.tum" \
  -p traj_provider_path:="$PWD/$OUT/provider.tum" \
  -p traj_graph_path:="$PWD/$OUT/graph.tum" \
  -p traj_global_path:="$PWD/$OUT/global.tum" \
  ${PRIOR_MAP:+-p prior_map_dir:="$PRIOR_MAP"} \
  > "$OUT/pass2.log" 2>&1 &
NODE_GRP=$!
sleep 14  # TRT engines deserialize

# odometry is replayed IMAGE-DRIVEN (scripts/odom_player.py): independent bag
# players skew timelines by the pass-1 pre-roll and the fusion node's image
# buffer never overlaps. The player publishes each recorded odometry message
# when the image stream reaches its header stamp — synced by construction.
setsid python3 scripts/odom_player.py "$OUT/odom_bag" > "$OUT/play_odo.log" 2>&1 &
ODOP=$!
sleep 5
setsid ros2 bag play "$BAG" --rate "$RATE" --qos-profile-overrides-path "$QOS" \
  --remap /tf_static:=/_bag_tf_static_unused > "$OUT/play_img.log" 2>&1 &
sleep 10
echo "10s: provider=$(wc -l < "$OUT/provider.tum" 2>/dev/null || echo 0)"
while pgrep -f "ros2 bag play $BAG" > /dev/null; do sleep 5; done
kill -- -"$ODOP" 2>/dev/null
sleep 8
kill -INT -- -"$NODE_GRP" 2>/dev/null
for _ in 1 2 3 4 5 6 7 8 9 10; do pgrep -f '^[^ ]*provider_fusion_node' > /dev/null || break; sleep 1; done
pgrep -f '^[^ ]*provider_fusion_node' | xargs -r kill -9 2>/dev/null
echo "pass2 done: provider=$(wc -l < "$OUT/provider.tum" 2>/dev/null) loops=$(grep -c 'LOOP CLOSED' "$OUT/pass2.log") $(grep -m1 -o 'refreshed [0-9]*/[0-9]*' "$OUT/pass2.log")"
