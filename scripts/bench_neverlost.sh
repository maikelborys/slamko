#!/usr/bin/env bash
# Never-lost + cross-session REGRESSION (one command). Session 1 maps a sequence and
# saves the Atlas; session 2 replays a DIFFERENT sequence (same place), loads that
# Atlas, and localizes into it via continuous relocalization (no forced loss). Both
# runs are gated by scripts/check_neverlost.py. Exit 0 = the full never-lost +
# cross-session pipeline works on real data — the reproducible guard for the spine.
#
# usage:  scripts/bench_neverlost.sh [PRIOR_SEQ] [LIVE_SEQ]
#   env:  EUROC_ROOT (default /mnt/data/datasets/euroc), OUT_DIR, PRIOR_END_S, LIVE_END_S
#
# Reuses the zombie-guard discipline of bench_ate.sh (CLAUDE.md HARD rule): a stale
# slamko_vio_node/euroc_player is a duplicate /tf publisher that corrupts the run.
set -o pipefail

PRIOR_SEQ="${1:-V1_01_easy}"
LIVE_SEQ="${2:-V1_02_medium}"
EUROC="${EUROC_ROOT:-/mnt/data/datasets/euroc}"
SLAMKO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${OUT_DIR:-/tmp/slamko_neverlost_bench}"
MAP="$OUT/prior_map"
PRIOR_END_S="${PRIOR_END_S:-50.0}"
LIVE_END_S="${LIVE_END_S:-60.0}"

[ -d "$EUROC/$PRIOR_SEQ/mav0" ] || { echo "no $EUROC/$PRIOR_SEQ/mav0"; exit 2; }
[ -d "$EUROC/$LIVE_SEQ/mav0" ]  || { echo "no $EUROC/$LIVE_SEQ/mav0";  exit 2; }
mkdir -p "$OUT"; rm -rf "$MAP"

source /opt/ros/jazzy/setup.bash
source "$HOME/coding/isaac_ros_ws/install/setup.bash"
source "$SLAMKO/install/setup.bash"

reap() {  # reap bench-owned processes by name (they escape the launch pgroup)
  pkill -INT -f 'slamko_vio_node' 2>/dev/null; pkill -INT -f 'euroc_player' 2>/dev/null
  for _ in $(seq 1 15); do pgrep -f slamko_vio_node >/dev/null 2>&1 || break; sleep 1; done
  pkill -KILL -f 'slamko_vio_node' 2>/dev/null; pkill -KILL -f 'euroc_player' 2>/dev/null
  sleep 1
}

# run_session <seq> <end_s> <log> "<extra launch args>"
run_session() {
  local seq="$1" end_s="$2" log="$3" extra="$4"
  setsid ros2 launch slamko_vio vio_euroc.launch.py \
    seq:="$EUROC/$seq" rate:=1.0 feature_source:=xfeat \
    enable_neverlost:=true neverlost_use_pose_graph:=true dr_enabled:=true \
    start_s:=0.0 end_s:="$end_s" $extra > "$log" 2>&1 &
  local pid=$!
  local t0=$SECONDS
  while true; do
    grep -q "done\. imgs=" "$log" && break
    kill -0 "$pid" 2>/dev/null || { echo "  launch exited early"; break; }
    (( SECONDS - t0 > 300 )) && { echo "  TIMEOUT"; break; }
    sleep 2
  done
  reap                                   # SIGINT first → map save + landmark dump flush
}

echo "=== slamko never-lost regression ==="
echo "prior(map)=$PRIOR_SEQ  live(relocalize)=$LIVE_SEQ  out=$OUT"
pgrep -f 'slamko_vio_node|euroc_player' >/dev/null 2>&1 && { echo "pre-flight reap"; reap; }

echo "--- session 1: build + save map ---"
run_session "$PRIOR_SEQ" "$PRIOR_END_S" "$OUT/s1.log" "map_save_dir:=$MAP"
[ -f "$MAP/submaps.manifest" ] || { echo "FAIL: session 1 did not save a map"; exit 1; }
echo "  saved: $(grep -c . "$MAP/submaps.manifest") submap(s)"

echo "--- session 2: load prior map + continuous relocalization ---"
run_session "$LIVE_SEQ" "$LIVE_END_S" "$OUT/s2.log" \
  "neverlost_continuous_reloc:=true prior_map_dir:=$MAP pose_dump_path:=$OUT/s2.tum landmark_dump_path:=$OUT/s2_lm.csv"

# Did a CROSS-SESSION weld fire? (welded to a prior-map submap)
if grep -q "CROSS-SESSION" "$OUT/s2.log"; then
  echo "  cross-session weld: YES"
else
  echo "FAIL: no cross-session weld in session 2"; tail -20 "$OUT/s2.log"; exit 1
fi

GT="$EUROC/$LIVE_SEQ/mav0/state_groundtruth_estimate0/data_tum.txt"
echo "--- auto-check (session 2 cross-session) ---"
python3 "$SLAMKO/scripts/check_neverlost.py" --log "$OUT/s2.log" --gt "$GT" \
  --est "$OUT/s2.tum" --submaps "$OUT/s2_lm.csv.submaps" --pose-epoch "$OUT/s2.tum.epoch" \
  --expect-seals 0 --max-anchor-m 5.0 --max-ate-cm 50.0
rc=$?

reap
pgrep -f 'slamko_vio_node|euroc_player' >/dev/null 2>&1 && echo "WARN: processes still alive after teardown"
[ $rc -eq 0 ] && echo "=== RESULT: PASS ===" || echo "=== RESULT: FAIL ==="
exit $rc
