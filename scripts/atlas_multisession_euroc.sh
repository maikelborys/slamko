#!/usr/bin/env bash
# ATLAS MULTI-SESSION TEST (the ORB-SLAM3 Atlas validation experiment): process the
# EuRoC Machine Hall sequences as SEPARATE sessions, each with prior_map_dir = the
# ACCUMULATED map of all previous sessions; measure whether the multimap welds them
# into one coherent map. MH ground truths share the Leica frame, so the merged map
# admits a SINGLE joint Sim3 vs the concatenated GT (the ORB-SLAM3 metric).
#
#   scripts/atlas_multisession_euroc.sh [out_root] [rate]
#   SEQS override: SEQS="MH_01_easy MH_03_medium MH_05_difficult" (default all 5)
#
# Accumulation model (provider_fusion_node): loadSubMaps(prior_dir) sets
# next_submap_id_ = max(prior id)+1, so each session's NEW submap_<id>.smap files are
# id-unique and the accumulated map is the plain UNION of files. Analysis:
# scripts/atlas_multisession_report.py.
set -o pipefail
cd "$(dirname "$0")/.."

OUT_ROOT=${1:-$PWD/results/atlas_ms}
RATE=${2:-1.0}
SEQS=${SEQS:-"MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult"}
declare -A BAGS=(
  [MH_01_easy]=/mnt/data/euroc_bags/mh_01_okvis
  [MH_02_easy]=/mnt/data/euroc_bags/mh_02_okvis
  [MH_03_medium]=/mnt/data/euroc_bags/mh_03_okvis
  [MH_04_difficult]=/mnt/data/euroc_bags/mh_04_okvis
  [MH_05_difficult]=/mnt/data/euroc_bags/mh_05_okvis
)

source /opt/ros/jazzy/setup.bash
source ~/coding/OKVIS2-X/install/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null   # libnvblox_lib.so
source install/setup.bash

PATTERN='[o]kvis2x_stereo|[p]rovider_fusion_node|[e]uroc_rectify|[b]ag play'
if pgrep -f "$PATTERN" > /dev/null; then
  echo "ABORT: stale processes:"; pgrep -af "$PATTERN"; exit 2
fi

ACCUM="$OUT_ROOT/map_accum"
rm -rf "$OUT_ROOT"; mkdir -p "$ACCUM"
i=0
for seq in $SEQS; do
  i=$((i+1)); out="$OUT_ROOT/s${i}_${seq}"
  bag=${BAGS[$seq]}
  [ -d "$bag" ] || { echo "MISSING BAG $bag — skipping $seq"; continue; }
  prior=""
  [ "$(ls -A "$ACCUM" 2>/dev/null)" ] && prior="$ACCUM"
  # bag duration + margin (rate-scaled): parse once via metadata
  dur=$(python3 - "$bag" <<'PY'
import sys, yaml
m = yaml.safe_load(open(sys.argv[1] + '/metadata.yaml'))
print(int(m['rosbag2_bagfile_information']['duration']['nanoseconds'] / 1e9) + 1)
PY
)
  budget=$(( ${dur%.*} * 2 + 90 ))
  echo "=== session $i: $seq (dur ${dur}s, budget ${budget}s, prior: ${prior:-NONE}) ==="
  timeout "$budget" ros2 launch slamko_ros pa_okvis_euroc_x.launch.py \
    bag_path:="$bag" seq:="$seq" out_dir:="$out" rate:="$RATE" \
    prior_map_dir:="$prior" > "$out.launch.log" 2>&1
  pkill -INT -f '[p]rovider_fusion_node' 2>/dev/null; sleep 4
  pkill -KILL -f "$PATTERN" 2>/dev/null; sleep 2
  n_new=$(ls "$out/map/"submap_*.smap 2>/dev/null | wc -l)
  # UNION-accumulate (ids are globally unique across sessions)
  cp -n "$out/map/"submap_*.smap "$ACCUM/" 2>/dev/null
  echo "    session $i done: $n_new new submaps, accum total $(ls "$ACCUM" | wc -l)"
  grep -cE 'LOOP CLOSED|weld|anchor' "$out/launch.log" 2>/dev/null || true
done
echo "=== ALL SESSIONS DONE -> $OUT_ROOT ==="
