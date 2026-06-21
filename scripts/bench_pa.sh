#!/usr/bin/env bash
# P-A live gate (MASTER_PLAN §8): run OKVIS2-X (pure VIO odometry) on a real
# D455 bag with slamko's provider_fusion_node attached, then verify the fused
# trajectory tracks the provider (no global constraints yet -> must match).
#
#   scripts/bench_pa.sh [bag_dir] [out_dir] [rate]
#
# Process safety (slamko CLAUDE.md zombie rule + euroc_player lesson): we ABORT
# if an okvis / provider_fusion_node is already alive (it may be the user's run —
# never blind-pkill), and on teardown we kill ONLY the PIDs that appeared after
# our launch. (No `set -u`: ROS setup.bash trips on unbound variables.)
cd "$(dirname "$0")/.."

BAG=${1:-/mnt/data/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO}
OUT=${2:-results/pa/$(basename "$BAG")}
RATE=${3:-1.0}
MAX_WAIT=${MAX_WAIT:-900}   # hard cap on the whole run [s]
VPR=${VPR:-false}           # true => P-B path (KF images + EigenPlaces + reloc + sealed map)
IMU_RATE=${IMU_RATE:-80.0}  # OKVIS /okvis_odometry publish rate [Hz]. MUST be a double
                            # (80, integer, throws InvalidParameterType); 0.0 = NO odom
                            # published (empty TUM!) on this build; 80.0 is the proven value.

PROVIDER=${PROVIDER:-okvis}   # okvis | kltvo
PATTERN='^[^ ]*(okvis2x_stereo_network_node_subscriber|provider_fusion_node|klt_vo_node)'
LAUNCH_FILE=pa_okvis_bag.launch.py
[ "$PROVIDER" = kltvo ] && LAUNCH_FILE=pa_kltvo_bag.launch.py

if pgrep -af "$PATTERN" > /dev/null; then
  echo "ABORT: an okvis/provider_fusion_node process is already running:"
  pgrep -af "$PATTERN"
  echo "If it is stale, kill it yourself and re-run (never blind-pkill: it may be a live session)."
  exit 2
fi

mkdir -p "$OUT/okvis"   # OKVIS dumps okvis2-vio-final_map.g2o here on shutdown; -p makes $OUT too
source /opt/ros/jazzy/setup.bash
source ~/coding/OKVIS2-X/install/setup.bash   # provider workspace (okvis pkg)
source ~/ros2_ws/install/setup.bash            # klt_vo workspace
source install/setup.bash

echo "== P-A bench: bag=$BAG out=$OUT rate=$RATE"
EXTRA_ARGS=()
[ -n "${PRIOR_MAP:-}" ] && EXTRA_ARGS+=("prior_map_dir:=$PRIOR_MAP")
# VIZ=true -> live Rerun viewer (gRPC); VIZ=/path.rrd -> offline rewindable file capture.
if [ -n "${VIZ:-}" ]; then
  EXTRA_ARGS+=("viz:=true")
  [ "$VIZ" != true ] && EXTRA_ARGS+=("viz_endpoint:=$VIZ")
fi
# COMPASS_YAW=1 -> BNO055 absolute-yaw graph prior (gated). Straightens heading drift.
[ -n "${COMPASS_YAW:-}" ] && EXTRA_ARGS+=("compass_yaw_prior:=true")
# OCC_REFRESH=false -> disable P2 revisit-fusion (baseline for the A/B).
[ -n "${OCC_REFRESH:-}" ] && EXTRA_ARGS+=("occ_refresh:=$OCC_REFRESH")
# MAPPOINT_ASSOC=true -> Phase A drift-tolerant cross-submap descriptor association.
[ -n "${MAPPOINT_ASSOC:-}" ] && EXTRA_ARGS+=("mappoint_assoc:=$MAPPOINT_ASSOC")
# DR_GATE_SOFT_COV=true -> #12 trunk: DR-gate uncertainty on the loss-gap chain edge.
[ -n "${DR_GATE_SOFT_COV:-}" ] && EXTRA_ARGS+=("dr_gate_soft_cov:=$DR_GATE_SOFT_COV")
[ "$PROVIDER" = okvis ] && EXTRA_ARGS+=("imu_rate:=$IMU_RATE")
# FORCE_LOSS="30,33" -> drop odom in that bag-relative window (test seal+branch).
[ -n "${FORCE_LOSS:-}" ] && EXTRA_ARGS+=("force_loss_start:=${FORCE_LOSS%,*}" "force_loss_end:=${FORCE_LOSS#*,}")
setsid ros2 launch slamko_ros $LAUNCH_FILE \
  bag_path:="$BAG" out_dir:="$PWD/$OUT" rate:="$RATE" rviz:=false vpr:="$VPR" \
  "${EXTRA_ARGS[@]}" \
  > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 8
OUR_PIDS=$(pgrep -f "$PATTERN" | tr '\n' ' ')
echo "launch pid=$LAUNCH_PID, nodes: $OUR_PIDS"

# Wait for the bag player to appear (bag_delay=10 s) and then finish.
elapsed=0
while [ $elapsed -lt 30 ] && ! pgrep -f "ros2 bag play $BAG" > /dev/null; do
  sleep 2; elapsed=$((elapsed + 2))
done
if ! pgrep -f "ros2 bag play $BAG" > /dev/null; then
  echo "WARN: bag player did not appear in 30 s — check $OUT/launch.log"
fi
elapsed=0
while [ $elapsed -lt "$MAX_WAIT" ] && pgrep -f "ros2 bag play $BAG" > /dev/null; do
  sleep 5; elapsed=$((elapsed + 5))
done
echo "bag finished (waited ${elapsed}s); draining 10 s then tearing down"
sleep 10

# Teardown: SIGINT first (clean rclcpp shutdown — the fusion node's destructor
# seals the trailing partial submap), wait, then escalate to exactly our PIDs.
kill -INT -- -"$LAUNCH_PID" 2>/dev/null
for _ in 1 2 3 4 5 6 7 8 9 10; do
  pgrep -f "$PATTERN" > /dev/null || break
  sleep 1
done
kill -- -"$LAUNCH_PID" 2>/dev/null
sleep 2
for pid in $OUR_PIDS; do kill -9 "$pid" 2>/dev/null; done
sleep 2
if pgrep -af "$PATTERN" > /dev/null; then
  echo "WARN: survivors after teardown:"; pgrep -af "$PATTERN"
fi

echo "== gate: fused vs provider (same run, un-aligned)"
python3 scripts/compare_tum.py "$OUT/provider.tum" "$OUT/fused.tum" --tol 0.01
GATE=$?
echo "lines: provider=$(wc -l < "$OUT/provider.tum" 2>/dev/null || echo 0)" \
     "fused=$(wc -l < "$OUT/fused.tum" 2>/dev/null || echo 0)"
exit $GATE
