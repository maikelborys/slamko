#!/usr/bin/env bash
# Phase D (lifelong maturity) demo — does MapPoint confidence COMPOUND across sessions?
# Two SERIAL runs of the SAME place with refine ON:
#   1. SESSION 1: build the map (Phase B refine) -> SMP6 submaps persist per-landmark n_obs.
#   2. SESSION 2: prior = session 1 + xsession + refine -> the seed RESTORES the prior n_obs
#      (not reset to 1), the revisit re-confirms, n_obs climbs higher than a single session.
# Proof = max n_obs (session 2) > max n_obs (session 1), and session 1's map is SMP6.
set +u
cd "$(dirname "$0")/.."

BAG=${BAG:-/mnt/data/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO}
RATE=${RATE:-0.5}
S1=results/mp/phaseD_s1
S2=results/mp/phaseD_s2

echo "######## Phase D — maturity compounding, bag=$(basename "$BAG") ########"

echo "==== SESSION 1/2: build the prior (refine -> SMP6 n_obs) ===="
VPR=true MAPPOINT_ASSOC=true MAPPOINT_REFINE=true bash scripts/bench_pa.sh "$BAG" "$S1" "$RATE"
echo "S1_DONE"

echo "==== SESSION 2/2: revisit with prior (xsession + refine -> restore + compound) ===="
PRIOR_MAP="$PWD/$S1/map" VPR=true MAPPOINT_ASSOC=true MAPPOINT_REFINE=true MAPPOINT_XSESSION=true \
  bash scripts/bench_pa.sh "$BAG" "$S2" "$RATE"
echo "S2_DONE"

echo "==== PROOF ===="
echo -n "session 1 map magic: "; head -c4 "$S1/map/$(head -1 "$S1/map/submaps.manifest").smap" 2>/dev/null \
  || head -c4 "$(ls "$S1"/map/submap_*.smap 2>/dev/null | head -1)"; echo " (expect SMP6)"
python3 - "$S1" "$S2" <<'PY'
import sys, os, numpy as np
for tag, run in (("session 1", sys.argv[1]), ("session 2 (compounded)", sys.argv[2])):
    csv = os.path.join(run, "map", "mappoints.csv")
    if not os.path.exists(csv):
        print(f"  {tag}: NO mappoints.csv"); continue
    d = np.genfromtxt(csv, delimiter=",", names=True)
    nobs = np.atleast_1d(d["n_obs"]).astype(int)
    print(f"  {tag}: {len(nobs)} MapPoints | n_obs max={nobs.max()} mean={nobs.mean():.2f} "
          f"| confirmed>=4: {(nobs>=4).sum()}")
PY
echo "-- session 2 prior-seed (store should start NON-empty, maturity restored) --"
grep -hE "prior map loaded|MapPoint store:" "$S2/launch.log" | sed 's/\x1b\[[0-9;]*m//g;s/.*node]: //' | head -2
echo "AB_D_DONE"
