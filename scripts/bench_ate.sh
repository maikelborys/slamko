#!/usr/bin/env bash
# slamko_vio ATE bench: run slamko_vio_node on one EuRoC sequence, record
# /slamko_vio/odometry, convert to TUM, compute Sim3-aligned ATE vs ground truth
# via evo_ape. Ported from klt_vo/scripts/bench_ate.sh (the validated harness).
#
# usage: scripts/bench_ate.sh <sequence_name> [rate]
#   env: EUROC_ROOT (default ~/datasets/euroc), OUT_SUFFIX, SLAMKO_EXTRA_ARGS
set -o pipefail

SEQ="${1:?give sequence name (e.g. MH_01_easy)}"
RATE="${2:-1.0}"

SLAMKO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EUROC_ROOT="${EUROC_ROOT:-/home/maikel/datasets/euroc}"
SEQ_DIR="$EUROC_ROOT/$SEQ"
OUT_DIR="$SLAMKO_ROOT/results/vio/${SEQ}${OUT_SUFFIX:-}"
BAG_DIR="$OUT_DIR/bag"
EST_TUM="$OUT_DIR/est.tum"
GT_TUM="$SEQ_DIR/mav0/state_groundtruth_estimate0/data_tum.txt"
LOG="$OUT_DIR/run.log"
CSV="$OUT_DIR/timing.csv"
ATE_TXT="$OUT_DIR/ate.txt"

[ -d "$SEQ_DIR/mav0" ] || { echo "no $SEQ_DIR/mav0"; exit 2; }
[ -f "$GT_TUM" ]       || { echo "no GT TUM at $GT_TUM"; exit 2; }

mkdir -p "$OUT_DIR"
rm -rf "$BAG_DIR" "$LOG" "$CSV"

source /opt/ros/jazzy/setup.bash
source "$HOME/coding/isaac_ros_ws/install/setup.bash"
source "$SLAMKO_ROOT/install/setup.bash"

echo "=== slamko_vio ATE bench: $SEQ (rate=$RATE) ==="
echo "log -> $LOG"

# Pre-flight zombie check (CLAUDE.md "check state before you launch"): a stale
# slamko_vio_node / euroc_player from a crashed prior run is a DUPLICATE publisher
# on /slamko_vio/odometry + /tf and silently corrupts this run (the recorder locks
# onto the wrong node, the GPU is contended, the launch dies early). Reap leftover
# bench processes — these process names are bench-owned, never the user's stack.
# (Run benches SERIALLY; concurrent runs would reap each other.)
if pgrep -f 'slamko_vio_node|euroc_player' >/dev/null 2>&1; then
    echo "WARN: stale bench processes found — reaping before launch:"
    pgrep -af 'slamko_vio_node|euroc_player'
    pkill -INT -f 'slamko_vio_node' 2>/dev/null || true
    pkill -INT -f 'euroc_player'    2>/dev/null || true
    sleep 2
    pkill -KILL -f 'slamko_vio_node' 2>/dev/null || true
    pkill -KILL -f 'euroc_player'    2>/dev/null || true
    sleep 1
fi

setsid ros2 launch slamko_vio vio_euroc.launch.py \
    seq:="$SEQ_DIR" rate:="$RATE" timing_csv:="$CSV" \
    ${SLAMKO_EXTRA_ARGS:-} \
    > "$LOG" 2>&1 &
LAUNCH_PID=$!

for i in $(seq 1 20); do
    grep -q "slamko_vio_node sprint" "$LOG" && break
    sleep 0.5
done

setsid ros2 bag record -s mcap -o "$BAG_DIR" \
    /slamko_vio/odometry /tf /tf_static \
    > "$OUT_DIR/bag.log" 2>&1 &
BAG_PID=$!

TIMEOUT_S=600
t0=$SECONDS
while true; do
    grep -q "done\. imgs=" "$LOG" && { echo "player reached end of sequence"; break; }
    kill -0 "$LAUNCH_PID" 2>/dev/null || { echo "launch exited early"; break; }
    (( SECONDS - t0 > TIMEOUT_S )) && { echo "TIMEOUT after ${TIMEOUT_S}s"; break; }
    sleep 2
done

sleep 1
kill -INT  -- "-$BAG_PID" 2>/dev/null || true
for i in $(seq 1 10); do kill -0 "$BAG_PID" 2>/dev/null || break; sleep 1; done
kill -KILL -- "-$BAG_PID" 2>/dev/null || true
wait "$BAG_PID" 2>/dev/null || true

[ -f "$BAG_DIR/metadata.yaml" ] || ros2 bag reindex "$BAG_DIR" -s mcap >> "$OUT_DIR/bag.log" 2>&1 || true

kill -INT  -- "-$LAUNCH_PID" 2>/dev/null || true
sleep 2
kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

# ros2 launch children can escape the process group — reap by name so this run
# leaves a CLEAN state (no zombies to corrupt the next bench), then verify.
pkill -KILL -f 'slamko_vio_node' 2>/dev/null || true
pkill -KILL -f 'euroc_player'    2>/dev/null || true
sleep 1
if pgrep -f 'slamko_vio_node|euroc_player' >/dev/null 2>&1; then
    echo "WARN: bench processes still alive after teardown — investigate:"
    pgrep -af 'slamko_vio_node|euroc_player'
fi

ros2 run euroc_publisher odom_to_tum --bag "$BAG_DIR" --out "$EST_TUM" \
    --topic /slamko_vio/odometry >> "$OUT_DIR/bag.log" 2>&1 || true

N=$(wc -l < "$EST_TUM" 2>/dev/null || echo 0)
echo "wrote $N poses -> $EST_TUM"
if [ "$N" -lt 100 ]; then
    echo "WARN: only $N poses — VO likely failed to initialise"; tail -30 "$LOG"; exit 1
fi

evo_ape tum "$GT_TUM" "$EST_TUM" -as --no_warnings > "$ATE_TXT" 2>&1 || true
echo ""
echo "=== ATE ($SEQ) ==="
grep -E "(rmse|mean|max|median|min|std|sse)" "$ATE_TXT" || cat "$ATE_TXT"
