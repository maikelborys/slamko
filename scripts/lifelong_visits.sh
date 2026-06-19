#!/usr/bin/env bash
# Lifelong immortality test: visit the SAME house N times against a map that GROWS
# between visits (visit k sees everything learned in visits 1..k-1). The question:
# does the map keep growing, or does new-submaps-per-visit shrink toward zero as the
# accumulated map covers more ground? (GAP-2 "don't re-map what you already see".)
#
#   scripts/lifelong_visits.sh <bag> <pass1_map> <work_dir> <n_visits>
# Accumulates into <work_dir>/acc_map; each visit's output in <work_dir>/visit_k.
cd "$(dirname "$0")/.."
BAG=${1:-/mnt/data/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO}
PASS1=${2:-results/r01/suave_force3s/map}
WORK=${3:-results/r01/lifelong}
N=${4:-4}
RATE=${RATE:-0.5}

ACC="$WORK/acc_map"
rm -rf "$ACC"; mkdir -p "$ACC"
cp "$PASS1"/submap_*.smap "$ACC"/ 2>/dev/null
cp "$PASS1"/submaps.manifest "$ACC"/submaps.manifest
echo "visit,prior_submaps,new_sealed,suppressed,new_landmarks" > "$WORK/growth.csv"

for k in $(seq 2 "$N"); do
  PRIOR_N=$(wc -l < "$ACC/submaps.manifest")
  OUT="$WORK/visit_$k"
  echo "== VISIT $k: prior has $PRIOR_N submaps -> $OUT"
  PRIOR_MAP="$PWD/$ACC" VPR=true bash scripts/bench_pa.sh "$BAG" "$OUT" "$RATE" \
    > "$OUT.log" 2>&1
  SUP=$(grep -c "SUPPRESSED duplicate" "$OUT/launch.log" 2>/dev/null || echo 0)
  NEWLM=$(grep -oE "\-\>[0-9]+ lm" "$OUT/launch.log" 2>/dev/null | grep -oE "[0-9]+" \
          | paste -sd+ | bc 2>/dev/null); NEWLM=${NEWLM:-0}
  # merge this visit's new submaps into the accumulated prior
  NEW=0
  if [ -f "$OUT/map/submaps.manifest" ]; then
    while read -r id; do
      [ -f "$OUT/map/submap_$id.smap" ] || continue
      cp "$OUT/map/submap_$id.smap" "$ACC"/
      echo "$id" >> "$ACC/submaps.manifest"
      NEW=$((NEW+1))
    done < "$OUT/map/submaps.manifest"
  fi
  echo "$k,$PRIOR_N,$NEW,$SUP,$NEWLM" >> "$WORK/growth.csv"
  echo "   visit $k: sealed $NEW new, suppressed $SUP, +$NEWLM lm  (acc now $(wc -l < "$ACC/submaps.manifest") submaps)"
done
echo "== growth table =="; cat "$WORK/growth.csv"
