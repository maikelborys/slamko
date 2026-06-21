#!/usr/bin/env bash
# EuRoC MH STACKING + blackout — the user's Atlas scenario, GT-backed.
# Session 1: MH_01 -> reference map. Session 2: MH_03 + prior=MH_01 + atlas break +
# an INJECTED blackout. Measures each session's ATE vs its EuRoC ground truth and reports
# the final fused/dangling component structure (etapa 2). Isolated: ROS_DOMAIN_ID=42 +
# reap by name (KLT_VO runs no OKVIS, so okvis/provider/rectify names are slamko-only).
cd /home/maikel/coding/slamko
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/coding/OKVIS2-X/install/setup.bash
source ~/coding/isaac_ros_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
source install/setup.bash

PAT='okvis2x_stereo_network_node_subscriber|provider_fusion_node|euroc_rectify_node'

run_session() {  # $1=name $2=bag $3=seq $4=extra-args
  local out="results/euroc/$1"
  mkdir -p "$out/okvis"
  echo "==== SESSION $1 : $(basename "$2") ($3)  extra=[$4] ===="
  setsid ros2 launch slamko_ros pa_okvis_euroc_x.launch.py \
    bag_path:="$2" seq:="$3" out_dir:="$PWD/$out" rate:=1.0 $4 > "$out/launch.log" 2>&1 &
  sleep 15
  local el=0
  while [ $el -lt 60 ] && ! pgrep -f "ros2 bag play $2" >/dev/null; do sleep 3; el=$((el+3)); done
  el=0
  while [ $el -lt 400 ] && pgrep -f "ros2 bag play $2" >/dev/null; do sleep 5; el=$((el+5)); done
  sleep 10
  pkill -INT -f provider_fusion_node 2>/dev/null; sleep 6   # seals + re-tags components.csv
  for p in $(pgrep -f "$PAT"); do kill -9 "$p" 2>/dev/null; done
  sleep 3
}

run_session stack_mh01 /mnt/data/euroc_bags/mh_01_okvis MH_01_easy ""
run_session stack_mh03 /mnt/data/euroc_bags/mh_03_okvis MH_03_medium \
  "prior_map_dir:=$PWD/results/euroc/stack_mh01/map atlas_break_on_loss:=true force_loss_start:=60.0 force_loss_end:=63.0"

echo "===== STACKING SUMMARY (GT-backed) ====="
python3 ~/coding/RTABmap/euroc_ate.py results/euroc/stack_mh01/fused.tum MH_01_easy   --label "S1 MH_01 ref       "
python3 ~/coding/RTABmap/euroc_ate.py results/euroc/stack_mh03/fused.tum MH_03_medium --label "S2 MH_03+blackout   "
echo "-- session 2 atlas events:"
grep -h 'ATLAS BREAK\|ETAPA 2\|LOCALIZED in prior\|LOOP CLOSED' results/euroc/stack_mh03/launch.log \
  | sed 's/\x1b\[[0-9;]*m//g' | sed 's/.*provider_fusion_node]: //' | tail -10
echo "-- session 2 final components:"; cat results/euroc/stack_mh03/map/components.csv 2>/dev/null | tail -n +2 | cut -d, -f2 | sort | uniq -c
echo STACK_DONE
