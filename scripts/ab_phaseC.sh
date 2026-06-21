#!/usr/bin/env bash
# Phase C (cross-session MapPoint seeding) A/B — does a 2nd session of the SAME place
# double the prior map, or dedup into it? Three SERIAL runs (concurrent benches reap each
# other — slamko CLAUDE.md):
#   1. SESSION 1: map casa suave -> the prior map (consensus, Phase A+B on).
#   2. SESSION 2 OFF: replay + prior, xsession OFF  (voxel prior_occ_ only — misses drift).
#   3. SESSION 2 ON : replay + prior, xsession ON   (voxel + DESCRIPTOR store — drift-tolerant).
# Phase C wins where cross-session drift makes the voxel test miss but the descriptor matches.
set +u
cd "$(dirname "$0")/.."

BAG=${BAG:-/mnt/data/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO}
RATE=${RATE:-0.5}
PRIOR=results/mp/xsession_prior
OFF=results/mp/xsession_off
ON=results/mp/xsession_on

echo "######## Phase C A/B — bag=$(basename "$BAG") rate=$RATE ########"

echo "==== SESSION 1/3: build the prior map (Phase A+B) ===="
VPR=true MAPPOINT_ASSOC=true MAPPOINT_REFINE=true bash scripts/bench_pa.sh "$BAG" "$PRIOR" "$RATE"
echo "PRIOR_DONE: $(wc -l < "$PRIOR/map/submaps.manifest" 2>/dev/null) prior submaps"

echo "==== SESSION 2/3: revisit OFF (no cross-session store) ===="
PRIOR_MAP="$PWD/$PRIOR/map" VPR=true MAPPOINT_ASSOC=true \
  bash scripts/bench_pa.sh "$BAG" "$OFF" "$RATE"
echo "OFF_DONE"

echo "==== SESSION 3/3: revisit ON (Phase C cross-session store) ===="
PRIOR_MAP="$PWD/$PRIOR/map" VPR=true MAPPOINT_ASSOC=true MAPPOINT_XSESSION=true \
  bash scripts/bench_pa.sh "$BAG" "$ON" "$RATE"
echo "ON_DONE"

echo "==== COMPARE: how much NEW map did the revisit seal? ===="
python3 scripts/ab_phaseA_report.py "$OFF" "$ON" /tmp/phaseC_ab.png
echo "-- cross-session events (ON) --"
grep -hE "LOCALIZED in prior|CULLED redundant|data-association|Phase C" "$ON/launch.log" \
  | sed 's/\x1b\[[0-9;]*m//g;s/.*node]: //' | tail -12
echo "AB_C_DONE"
