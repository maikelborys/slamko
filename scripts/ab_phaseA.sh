#!/usr/bin/env bash
# Phase A (persistent-MapPoint drift-tolerant association) A/B on the REVISIT proof bag.
# Runs SERIALLY (concurrent benches reap each other via the global pkill race —
# slamko CLAUDE.md): baseline (mappoint_assoc OFF) then Phase A (ON), same bag/rate/VPR.
# Then sums landmarks + submaps for each and renders a side-by-side top-down so the
# doubling collapse is visible. The trajectory must stay neutral (loose fuser unchanged).
#
#   scripts/ab_phaseA.sh        # uses the casa brutal revisit bag @0.5 VPR=true
set +u
cd "$(dirname "$0")/.."

BAG=${BAG:-/mnt/data/bno_ab/CASA1_brutal1_Stereo60_RGB30_BNO_trim}
RATE=${RATE:-0.5}
OFF=results/mp/phaseA_off
ON=results/mp/phaseA_on

echo "######## Phase A A/B — bag=$(basename "$BAG") rate=$RATE VPR=true ########"

echo "==== RUN 1/2: BASELINE (mappoint_assoc OFF) ===="
VPR=true MAPPOINT_ASSOC=false bash scripts/bench_pa.sh "$BAG" "$OFF" "$RATE"
echo "BASELINE_DONE"

echo "==== RUN 2/2: PHASE A (mappoint_assoc ON) ===="
VPR=true MAPPOINT_ASSOC=true bash scripts/bench_pa.sh "$BAG" "$ON" "$RATE"
echo "PHASEA_DONE"

echo "==== COMPARE ===="
python3 scripts/ab_phaseA_report.py "$OFF" "$ON" /tmp/phaseA_ab.png
echo "AB_ALL_DONE"
