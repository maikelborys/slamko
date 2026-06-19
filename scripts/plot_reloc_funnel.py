#!/usr/bin/env python3
"""Reloc funnel diagnostic — answers "is the recall bottleneck RETRIEVAL (VPR global
descriptor) or the LOCAL descriptor (XFeat verify)?" by parsing the relocalizer's
`[reloc] topN: id(cos) ... -> verified/none` stderr lines from a launch.log.

If VERIFIED attempts cluster at HIGH top-1 cosine and FAILED at LOW cosine (disjoint),
the gate is RETRIEVAL (EigenPlaces) — XFeat verifies fine whenever a candidate surfaces.
If failures sit at HIGH cosine (candidate surfaced) but still didn't verify, the gate is
the VERIFIER (XFeat/PnP on hard viewpoints) — then a stronger matcher (LighterGlue/LoFTR)
is the lever. See memory slamko-loopclosure-recall-bottleneck.

Usage:  python3 scripts/plot_reloc_funnel.py <launch.log> [out.png]
"""
import sys, re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

log = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else log.rsplit("/", 1)[0] + "/reloc_funnel.png"
ver, non = [], []
for ln in open(log):
    if "[reloc] top" not in ln:
        continue
    m = re.search(r"top\d+:\s+\d+\(([0-9.]+)\)", ln)  # top-1 candidate cosine
    if not m:
        continue
    (ver if "verified" in ln else non).append(float(m.group(1)))

if not ver and not non:
    sys.exit(f"no '[reloc] topN:' lines in {log} (was VPR on? is the relocalizer verbose?)")

def stats(a):
    return (min(a), sorted(a)[len(a) // 2], max(a)) if a else (0, 0, 0)
vmn, vmd, vmx = stats(ver); fmn, fmd, fmx = stats(non)
print(f"VERIFIED n={len(ver)} cosine min/med/max = {vmn:.2f}/{vmd:.2f}/{vmx:.2f}")
print(f"FAILED   n={len(non)} cosine min/med/max = {fmn:.2f}/{fmd:.2f}/{fmx:.2f}")
gate = "RETRIEVAL (VPR/EigenPlaces)" if (non and ver and fmd < vmd - 0.2) else "VERIFIER (XFeat/PnP) or mixed"
print(f"=> bottleneck looks like: {gate}")

fig, ax = plt.subplots(figsize=(8, 4.4))
bins = [i / 20 for i in range(21)]
ax.hist(ver, bins=bins, alpha=0.7, color="#5cb85c", label=f"VERIFIED (n={len(ver)})")
ax.hist(non, bins=bins, alpha=0.7, color="#d9534f", label=f"FAILED -> none (n={len(non)})")
ax.set_xlabel("top-1 VPR cosine (EigenPlaces global descriptor)")
ax.set_ylabel("reloc attempts")
ax.set_title(f"Reloc funnel — bottleneck: {gate}\n"
             f"verified med {vmd:.2f} vs failed med {fmd:.2f}")
ax.legend()
fig.tight_layout()
fig.savefig(out, dpi=130)
print(f"wrote {out}")
