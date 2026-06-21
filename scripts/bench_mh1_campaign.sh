#!/usr/bin/env bash
# EuRoC Machine Hall 1 — blackout/localization campaign with the full MapPoint stack ON.
#   1. NORMAL      : MH_01 full, no blackout -> the reference map (submaps + loops + ATE).
#   2-4. BLACKOUT  : MH_01 + prior=NORMAL + an injected blackout (early/mid/long) + atlas break
#                    -> must break into a new submap on loss, recover, and re-LOCALIZE to the
#                    prior; ATE vs GT shows the cost of the gap.
#   5. LOCALIZE    : MH_01 + prior=NORMAL, no blackout -> pure cross-session relocalization +
#                    Phase C dedup-into-prior showcase.
# All runs: assoc+refine ON (clean consensus map); prior runs add xsession (Phase C).
# EuRoC is OKVIS-stable at rate 1.0 (NOT the D455 rate<=0.5 rule). Isolated: ROS_DOMAIN_ID=42
# + reap by name (KLT_VO runs no OKVIS, so these names are slamko-only). Robust teardown
# (PID capture + INT provider to seal + reap) — the manual euroc_x teardown is fragile.
cd /home/maikel/coding/slamko
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/coding/OKVIS2-X/install/setup.bash
source ~/coding/isaac_ros_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
source install/setup.bash

BAG=/mnt/data/euroc_bags/mh_01_okvis
SEQ=MH_01_easy
PAT='okvis2x_stereo_network_node_subscriber|provider_fusion_node|euroc_rectify_node'
MP="mappoint_assoc:=true mappoint_refine:=true"

run() {  # $1=name  $2=extra-args
  local out="results/mh1/$1"
  mkdir -p "$out/okvis"
  echo "==== RUN $1  extra=[$2] ===="
  setsid ros2 launch slamko_ros pa_okvis_euroc_x.launch.py \
    bag_path:="$BAG" seq:="$SEQ" out_dir:="$PWD/$out" rate:=1.0 \
    atlas_break_on_loss:=true $MP $2 > "$out/launch.log" 2>&1 &
  sleep 15
  local el=0
  while [ $el -lt 60 ] && ! pgrep -f "ros2 bag play $BAG" >/dev/null; do sleep 3; el=$((el+3)); done
  el=0
  while [ $el -lt 400 ] && pgrep -f "ros2 bag play $BAG" >/dev/null; do sleep 5; el=$((el+5)); done
  sleep 10
  pkill -INT -f provider_fusion_node 2>/dev/null; sleep 8   # seals + re-tags components.csv
  for p in $(pgrep -f "$PAT"); do kill -9 "$p" 2>/dev/null; done
  sleep 3
  echo "  $1: submaps=$(wc -l < "$out/map/submaps.manifest" 2>/dev/null) poses=$(wc -l < "$out/fused.tum" 2>/dev/null)"
}

PRIOR="prior_map_dir:=$PWD/results/mh1/normal/map mappoint_xsession:=true"

run normal      ""
run bk_early    "$PRIOR force_loss_start:=40.0 force_loss_end:=44.0"
run bk_mid      "$PRIOR force_loss_start:=90.0 force_loss_end:=94.0"
run bk_long     "$PRIOR force_loss_start:=90.0 force_loss_end:=100.0"
run localize    "$PRIOR"

echo "===== MH_01 CAMPAIGN REPORT ====="
python3 scripts/mh1_report.py
echo "MH1_CAMPAIGN_DONE"
