#!/usr/bin/env bash
# Observe casa SUAVE then BRUTAL: how many submaps are sealed, how many times tracking
# goes bad (OKVIS TRACKING FAILURE + map-level ATLAS BREAK), and whether it re-localizes
# right after (loop close / proximity weld / cross-session). SERIAL (concurrent benches reap
# each other). rate 0.5 (don't outrun OKVIS on casa); VPR + atlas_break + assoc+refine ON.
set +u
cd "$(dirname "$0")/.."

timeline() {  # $1=run-dir  $2=label
  local log="$1/launch.log"
  echo "================  $2  ================"
  echo "submaps sealed : $(wc -l < "$1/map/submaps.manifest" 2>/dev/null || echo 0)"
  echo "OKVIS tracking-loss events (TRACKING FAILURE) : $(grep -c 'TRACKING FAILURE' "$log" 2>/dev/null)"
  echo "map-level ATLAS BREAKs                        : $(grep -c 'ATLAS BREAK' "$log" 2>/dev/null)"
  echo "re-localizations (loop / weld / proximity)    : $(grep -cE 'LOOP CLOSED|proximity weld|LOCALIZED in prior|PROMOTED' "$log" 2>/dev/null)"
  local comps=0
  [ -f "$1/map/components.csv" ] && comps=$(tail -n +2 "$1/map/components.csv" | cut -d, -f2 | sort -u | wc -l)
  echo "final atlas components (fused/dangling)       : $comps"
  echo "-- event timeline (break -> does it re-localize soon?) --"
  grep -hnE 'ATLAS BREAK|LOOP CLOSED|proximity weld|LOCALIZED in prior|PROMOTED|sealed submap' "$log" 2>/dev/null \
    | sed 's/\x1b\[[0-9;]*m//g;s/.*node]: //' | grep -E 'ATLAS BREAK|LOOP CLOSED|weld|LOCALIZED|PROMOTED' | head -20
  echo
}

ENV='VPR=true ATLAS_BREAK=true MAPPOINT_ASSOC=true MAPPOINT_REFINE=true'

echo "#### RUN 1/2: casa SUAVE @0.5 ####"
env $ENV bash scripts/bench_pa.sh /mnt/data/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO results/observe/suave 0.5
echo "SUAVE_DONE"

echo "#### RUN 2/2: casa BRUTAL @0.5 ####"
env $ENV bash scripts/bench_pa.sh /mnt/data/bno_ab/CASA1_brutal1_Stereo60_RGB30_BNO_trim results/observe/brutal 0.5
echo "BRUTAL_DONE"

echo; echo "########  OBSERVATION REPORT  ########"
timeline results/observe/suave  "casa SUAVE"
timeline results/observe/brutal "casa BRUTAL"
echo "OBSERVE_DONE"
