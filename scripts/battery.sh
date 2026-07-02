#!/usr/bin/env bash
# battery.sh — the ONE-COMMAND regression battery over the real-bag set (T1 of
# docs/PLAN_IMMORTAL_FRAMEWORK_01.md). Runs the CURRENT pipeline (OKVIS provider ->
# provider_fusion_node) SERIALLY over every row, then scores each run with the
# provider-agnostic 7-channel scorecard (scripts/slamko_eval.py) and aggregates one
# markdown report (scripts/battery_report.py).
#
#   usage:  scripts/battery.sh [row ...]            # default: all rows
#   env:    GATES=on|off   immortal gate set (default on — the profile under test)
#           RATE=0.5       bag rate (history: OKVIS caps ~31fps; rate>0.5 = bursty drops)
#           BATTERY_ROOT=/mnt/data/slamko_battery   (durable — /tmp gets wiped)
#           TAG=<name>     run tag (default <date>_<gates>)
#
# Row config is LOAD-BEARING (memory slamko-okvis-config-resolution-mismatch):
#   640x480 bno_ab bags -> rsD455_map_odom via the BNO055/ab launch (owns the IMU
#   relay for the DOUBLED camera-IMU accel); 848 bags -> rsD455_map848; the flash
#   bag -> splitter + caminfo-inject + rsD455_odom848 + volumetric.
# Serial only: concurrent benches reap each other (CLAUDE.md zombie rule).
cd "$(dirname "$0")/.."
SLAMKO=$PWD
# NB: source ROS before `set -u` — jazzy's setup.bash trips on unset vars.
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/coding/OKVIS2-X/install/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null     # libnvblox_lib.so on the loader path (S3 fix)
source "$SLAMKO/install/setup.bash" 2>/dev/null
set -u
GATES=${GATES:-on}
RATE=${RATE:-0.5}
BATTERY_ROOT=${BATTERY_ROOT:-/mnt/data/slamko_battery}
TAG=${TAG:-$(date +%Y%m%d_%H%M)_g$GATES}
ROOT="$BATTERY_ROOT/$TAG"
EVALPY=$HOME/.venvs/slamko-eval/bin/python3
AB_LAUNCH=$HOME/coding/BNO055/ab/okvis_ab_c1_d455imu.launch.py
BNOAB=/mnt/data/bno_ab
REAP_PAT='okvis2x|okvis_ab_c1|provider_fusion_node|imu_relay|d455_splitter_auto|cam_info_inject|robot_state_publisher|ros2 bag play'

export OMP_NUM_THREADS=2

# name|type|bag|okvis_cfg  (type: bnoab | flash)
ROWS=(
  "suave|bnoab|$BNOAB/CASA1_Suave_Stereo60_RGB30_BNO_strim|rsD455_map_odom"
  "escaleras|bnoab|$BNOAB/CASA1_Escaleras_Stereo60_RGB30_BNO_strim|rsD455_map_odom"
  "h40|bnoab|$BNOAB/CASA1_40cmH_Stereo60_RGB30_BNO_848_trim|rsD455_map848"
  "h100|bnoab|$BNOAB/CASA1_100cmH_Stereo60_RGB30_BNO_848_trim|rsD455_map848"
  "brutal1|bnoab|$BNOAB/CASA1_brutal1_Stereo60_RGB30_BNO_trim|rsD455_map_odom"
  "wall|bnoab|$BNOAB/CASA1_wall_Stereo60_RGB30_BNO_trim|rsD455_map_odom"
  "blackout|bnoab|$BNOAB/CASA1_Suave_blackout|rsD455_map_odom"
  "blackout4|bnoab|$BNOAB/CASA1_Suave_blackout4|rsD455_map_odom"
  "extreme|bnoab|$BNOAB/CASA1_extremeFinal_Stereo60_RGB30_BNO_strim|rsD455_map_odom"
  "flash|flash|/mnt/data/d455_bags/casa_084815_flashbno_trim|rsD455_odom848"
)

reap() {
  pkill -KILL -f "$REAP_PAT" 2>/dev/null
  sleep 2
}
trap reap EXIT INT TERM

fusion_args() {  # $1=out_dir $2=image_topic
  local out=$1 img=$2
  local a="-p odom_topic:=/okvis/okvis_odometry -p image_topic:=$img"
  a+=" -p imu_topic:=/camera/camera/imu -p mag_topic:=/bno055/mag"
  a+=" -p map_dir:=$out/map -p kf_per_submap:=50"
  a+=" -p traj_provider_path:=$out/provider.tum -p traj_graph_path:=$out/graph.tum"
  a+=" -p traj_fused_path:=$out/fused.tum -p traj_slewed_path:=$out/traj_slewed.tum"
  a+=" -p reloc:=true -p use_scan_context:=true"
  if [ "$GATES" = on ]; then
    a+=" -p imu_referee:=true -p hold_on_loss:=true"
    a+=" -p gate_live_pose:=true -p live_gate_speed:=2.5"
    a+=" -p atlas_break_on_quality:=true -p quality_soft_bridge:=true -p dr_gate_soft_cov:=true"
  fi
  echo "$a"
}

wait_bag() {  # wait for `ros2 bag play` to appear (<=180s) then finish (<=1200s)
  local el=0
  while [ $el -lt 180 ] && ! pgrep -f 'ros2 bag play' >/dev/null; do sleep 3; el=$((el+3)); done
  pgrep -f 'ros2 bag play' >/dev/null || { echo "    WARN: bag never started"; return 1; }
  el=0
  while [ $el -lt 1200 ] && pgrep -f 'ros2 bag play' >/dev/null; do sleep 5; el=$((el+5)); done
  return 0
}

teardown_run() {  # $1=out_dir — SIGINT fusion first (seals + flushes map), then reap
  ros2 service call /okvis/shutdown std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1 || true
  sleep 3
  pkill -INT -f provider_fusion_node 2>/dev/null; sleep 6
  reap
}

run_bnoab() {  # $1=name $2=bag $3=cfg
  local name=$1 bag=$2 cfg=$3 out="$ROOT/$name"
  mkdir -p "$out/map"
  echo "[1] OKVIS ($cfg, ab launch: IMU relay for doubled accel) + bag@$RATE delay 22s"
  OKVIS_CFG=$cfg ros2 launch "$AB_LAUNCH" bag_path:="$bag" rate:="$RATE" \
    bag_delay:=22.0 rviz:=false csv_path:="$out/okvis/" > "$out/okvis.log" 2>&1 &
  sleep 10
  echo "[2] provider_fusion (GATES=$GATES)"
  # shellcheck disable=SC2046
  ros2 run slamko_ros provider_fusion_node --ros-args $(fusion_args "$out" \
    /camera/camera/infra1/image_rect_raw) > "$out/fusion.log" 2>&1 &
  wait_bag; sleep 8
  teardown_run "$out"
}

run_flash() {  # flash bag: splitter + caminfo inject + volumetric (+depth loop when gates on)
  local name=$1 bag=$2 cfg=$3 out="$ROOT/$name"
  local cfgdir=$HOME/coding/OKVIS2-X/src/OKVIS2-X/config/$cfg
  local qos=$HOME/coding/d455_setup/casa2_qos_override.yaml
  mkdir -p "$out/map"
  echo "[1] caminfo injector + content splitter"
  python3 "$SLAMKO/scripts/cam_info_inject_848.py" > "$out/caminfo.log" 2>&1 &
  python3 ~/coding/d455_setup/d455_splitter_auto.py > "$out/splitter.log" 2>&1 &
  sleep 2
  echo "[2] OKVIS ($cfg)"
  ros2 run okvis okvis2x_stereo_network_node_subscriber --ros-args \
    -p config_filename:="$cfgdir/okvis2.yaml" -p se_config_filename:="$cfgdir/se2.yaml" \
    -p csv_path:="$out/okvis/" -p save_submap_meshes:=false \
    -p imu_propagated_state_publishing_rate:=80.0 \
    -r /okvis/imu0:=/camera/camera/imu -r __ns:=/okvis -r __node:=okvis \
    > "$out/okvis.log" 2>&1 &
  sleep 3
  echo "[3] provider_fusion + volumetric (GATES=$GATES)"
  local extra="-p volumetric:=true -p depth_topic:=/nvblox/depth/image_rect_raw"
  extra+=" -p depth_best_effort:=true -p volumetric_voxel_m:=0.05 -p volumetric_correct_every:=10"
  extra+=" -p volumetric_mesh_path:=$out/volumetric_live.ply"
  [ "$GATES" = on ] && extra+=" -p depth_loop_refine:=true"
  # shellcheck disable=SC2046
  ros2 run slamko_ros provider_fusion_node --ros-args \
    $(fusion_args "$out" /okvis/cam0/image_raw) $extra > "$out/fusion.log" 2>&1 &
  echo "[4] wait node ready (TRT engine build) before bag"
  for _ in $(seq 1 60); do
    grep -qiE 'reloc ready|provider_fusion_node up' "$out/fusion.log" 2>/dev/null && break
    sleep 5
  done
  sleep 8
  echo "[5] bag @$RATE"
  ros2 bag play "$bag" --rate "$RATE" --qos-profile-overrides-path "$qos" \
    --remap /tf_static:=/_unused_tf > "$out/play.log" 2>&1
  sleep 8
  teardown_run "$out"
}

score() {  # $1=name $2=bag — 7-channel scorecard + counts
  local name=$1 bag=$2 out="$ROOT/$name"
  # channel 7 (geometric) needs the landmark cloud export
  ros2 run slamko_loop smap_cloud "$out/map" "$out/global_cloud.csv" 1 \
    > "$out/smap_cloud.log" 2>&1 || true
  "$EVALPY" "$SLAMKO/scripts/slamko_eval.py" "$out" --bag "$bag" --label "$name" \
    --json > "$out/eval.json" 2> "$out/eval.err" || echo "    WARN: eval failed ($name)"
  {
    echo "submaps=$(ls "$out"/map/submap_*.smap 2>/dev/null | wc -l)"
    echo "components=$(tail -n +2 "$out/map/components.csv" 2>/dev/null | cut -d, -f2 | sort -u | wc -l)"
    echo "welds=$(grep -ciE 'LOOP CLOSED|weld' "$out/fusion.log" 2>/dev/null)"
    echo "atlas_breaks=$(grep -cE 'ATLAS BREAK|new island' "$out/fusion.log" 2>/dev/null)"
    echo "crashes=$(grep -ciE 'Segmentation|what\(\)|core dumped' "$out/fusion.log" 2>/dev/null)"
    echo "provider_lines=$(wc -l < "$out/provider.tum" 2>/dev/null || echo 0)"
    echo "graph_lines=$(wc -l < "$out/graph.tum" 2>/dev/null || echo 0)"
  } > "$out/counts.txt"
}

SEL=("$@"); [ ${#SEL[@]} -eq 0 ] && SEL=(all)
mkdir -p "$ROOT"
echo "=== slamko battery: TAG=$TAG GATES=$GATES RATE=$RATE -> $ROOT ==="
pgrep -f "$REAP_PAT" >/dev/null 2>&1 && { echo "[pre-flight reap]"; reap; }

for row in "${ROWS[@]}"; do
  IFS='|' read -r name type bag cfg <<< "$row"
  if [ "${SEL[0]}" != all ]; then
    match=0; for s in "${SEL[@]}"; do [ "$s" = "$name" ] && match=1; done
    [ $match -eq 0 ] && continue
  fi
  [ -d "$bag" ] || { echo "--- SKIP $name (no bag: $bag)"; continue; }
  echo ""
  echo "======== ROW $name  ($type, $(basename "$bag"), $cfg) ========"
  t0=$SECONDS
  "run_$type" "$name" "$bag" "$cfg"
  score "$name" "$bag"
  echo "-------- $name done in $((SECONDS-t0))s"
  cat "$ROOT/$name/counts.txt" | tr '\n' ' '; echo ""
done

echo ""
echo "=== aggregate report ==="
"$EVALPY" "$SLAMKO/scripts/battery_report.py" "$ROOT" \
  --out "$SLAMKO/docs/battery/BATTERY_$TAG.md" || true
echo "=== BATTERY DONE: $ROOT ==="
