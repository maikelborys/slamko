#!/usr/bin/env bash
# EuRoC Machine Hall ATE + blackout gate (Stage 1). Runs OKVIS2-X (pure VIO) +
# slamko provider_fusion on a EuRoC bag, dumps fused/provider TUM, prints ATE vs
# ground-truth (Umeyama SE3). Optional VIO / VIO+IMU blackout via FORCE_LOSS.
#
#   scripts/bench_euroc.sh [seq] [out_dir] [rate]
#     seq      MH_01_easy | MH_03_medium | MH_05_difficult   (default MH_03_medium)
#   env:
#     FORCE_LOSS="40,46"  drop odom in that bag-relative window (blackout)
#     IMU=off             VIO-only blackout coast (default on = VIO+IMU DR-gate)
#     VPR=true            Stage-2 stereo-landmark path (needs a RECTIFIED bag!)
#     PRIOR_MAP=<dir>     cross-session relocalization prior
#
# EuRoC is OKVIS-stable -> rate 1.0 default (NOT the D455 rate<=0.5 rule).
# Zombie discipline: abort if okvis/fusion already alive; reap only our PIDs.
cd "$(dirname "$0")/.."

SEQ=${1:-MH_03_medium}
RATE=${3:-1.0}
# seq name -> bag dir (mh_03_okvis) + GT seq (MH_03_medium)
declare -A BAGMAP=([MH_01_easy]=mh_01_okvis [MH_03_medium]=mh_03_okvis [MH_05_difficult]=mh_05_okvis)
BAG=/mnt/data/euroc_bags/${BAGMAP[$SEQ]}
OUT=${2:-results/euroc/$SEQ}
MAX_WAIT=${MAX_WAIT:-600}

PATTERN='^[^ ]*(okvis2x_stereo_network_node_subscriber|provider_fusion_node)'
if pgrep -af "$PATTERN" > /dev/null; then
  echo "ABORT: okvis/provider_fusion already running (may be a live session):"; pgrep -af "$PATTERN"
  exit 2
fi
if [ ! -d "$BAG" ]; then echo "no bag: $BAG"; exit 1; fi

mkdir -p "$OUT/okvis"
source /opt/ros/jazzy/setup.bash
source ~/coding/OKVIS2-X/install/setup.bash             # okvis pkg + euroc config
source ~/coding/isaac_ros_ws/install/setup.bash         # euroc_publisher (gt pub + launch)
source ~/ros2_ws/install/setup.bash
source install/setup.bash

EXTRA=()
[ -n "${VPR:-}" ]       && EXTRA+=("vpr:=$VPR")
[ -n "${PRIOR_MAP:-}" ] && EXTRA+=("prior_map_dir:=$PRIOR_MAP")
[ "${IMU:-on}" = off ]  && EXTRA+=("imu_topic:=none")   # 'none' sentinel -> VIO-only coast
[ -n "${VIZ:-}" ]       && { EXTRA+=("viz:=true"); [ "$VIZ" != true ] && EXTRA+=("viz_endpoint:=$VIZ"); }
[ -n "${FORCE_LOSS:-}" ] && EXTRA+=("force_loss_start:=${FORCE_LOSS%,*}" "force_loss_end:=${FORCE_LOSS#*,}")

echo "== EuRoC bench: seq=$SEQ bag=$BAG out=$OUT rate=$RATE ${FORCE_LOSS:+blackout=$FORCE_LOSS imu=${IMU:-on}}"
setsid ros2 launch slamko_ros pa_okvis_euroc.launch.py \
  bag_path:="$BAG" seq:="$SEQ" out_dir:="$PWD/$OUT" rate:="$RATE" rviz:=false \
  "${EXTRA[@]}" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 8
OUR_PIDS=$(pgrep -f "$PATTERN" | tr '\n' ' ')
echo "launch pid=$LAUNCH_PID nodes: $OUR_PIDS"

elapsed=0
while [ $elapsed -lt 40 ] && ! pgrep -f "ros2 bag play $BAG" > /dev/null; do sleep 2; elapsed=$((elapsed+2)); done
pgrep -f "ros2 bag play $BAG" > /dev/null || echo "WARN: bag player never appeared — see $OUT/launch.log"
elapsed=0
while [ $elapsed -lt "$MAX_WAIT" ] && pgrep -f "ros2 bag play $BAG" > /dev/null; do sleep 5; elapsed=$((elapsed+5)); done
echo "bag finished (${elapsed}s); draining 10s then teardown"
sleep 10

kill -INT -- -"$LAUNCH_PID" 2>/dev/null
for _ in $(seq 1 12); do pgrep -f "$PATTERN" > /dev/null || break; sleep 1; done
kill -- -"$LAUNCH_PID" 2>/dev/null; sleep 2
for pid in $OUR_PIDS; do kill -9 "$pid" 2>/dev/null; done; sleep 2
pgrep -af "$PATTERN" > /dev/null && { echo "WARN survivors:"; pgrep -af "$PATTERN"; }

echo "== ATE vs ground-truth ($SEQ) =="
python3 ~/coding/RTABmap/euroc_ate.py "$OUT/provider.tum" "$SEQ" --label "provider(okvis)"
python3 ~/coding/RTABmap/euroc_ate.py "$OUT/fused.tum"    "$SEQ" --label "fused(slamko)  "
echo "lines: provider=$(wc -l < "$OUT/provider.tum" 2>/dev/null||echo 0) fused=$(wc -l < "$OUT/fused.tum" 2>/dev/null||echo 0)"
