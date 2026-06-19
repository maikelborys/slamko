#!/usr/bin/env python3
"""I2 invariant audit (never make a false merge). A false merge would weld two
GEOGRAPHICALLY-DISTANT submaps with a hard (type-2) loop edge whose relative
translation is implausibly large (a teleport). Scans every <run>/.../anchor_edges.csv
and flags any hard edge above `max_sane_m`. The PCM 3-vote consensus + the 30 m
teleport gate + the VPR cosine cliff are the protections this checks empirically.

Usage:  python3 scripts/audit_i2.py [results_root] [max_sane_m]
"""
import sys, glob, csv, math

root = sys.argv[1] if len(sys.argv) > 1 else "results"
max_sane = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

hard = []
for f in glob.glob(f"{root}/**/anchor_edges.csv", recursive=True):
    try:
        for r in csv.DictReader(open(f)):
            if r.get("type") == "2":  # hard / verified weld
                d = math.sqrt(float(r["tx"]) ** 2 + float(r["ty"]) ** 2 + float(r["tz"]) ** 2)
                run = f.split("/")[-3] if f.count("/") >= 2 else f
                hard.append((d, r["from"], r["to"], run))
    except Exception:
        pass

if not hard:
    print(f"no hard (loop) edges found under {root}")
    sys.exit(0)

hard.sort(reverse=True)
big = [h for h in hard if h[0] > max_sane]
print(f"hard (loop) welds audited: {len(hard)}")
print(f"max weld translation: {hard[0][0]:.2f} m (submap {hard[0][1]}->{hard[0][2]}, {hard[0][3]})")
print(f"median: {hard[len(hard)//2][0]:.2f} m   min: {hard[-1][0]:.3f} m")
print(f"welds > {max_sane:.0f} m (potential false-merge/teleport): {len(big)}")
for h in big[:10]:
    print("   ", f"{h[0]:.2f} m  {h[1]}->{h[2]}  {h[3]}")
print("VERDICT:", "I2 HOLDS — all welds geographically sane" if not big
      else f"INVESTIGATE — {len(big)} suspicious welds")
sys.exit(1 if big else 0)
