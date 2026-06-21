#!/usr/bin/env python3
"""Phase B deliverable: render the CONSENSUS map coloured by visit confidence (n_obs),
and report the refinement stats. A point seen from many visits (high n_obs) is trusted;
a one-shot point (n_obs=1) is provisional — exactly ORB-SLAM3/PLVS MapPoint maturity.

  render_confidence.py <run_dir> <out.png>     # reads <run_dir>/map/mappoints.csv

mappoints.csv: id,x,y,z,n_obs  (written at shutdown when mappoint_refine is ON).
"""
import os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

run, out = sys.argv[1], sys.argv[2]
csv = os.path.join(run, "map", "mappoints.csv")
if not os.path.exists(csv):
    print(f"NO mappoints.csv in {run}/map — was mappoint_refine ON?"); sys.exit(1)

d = np.genfromtxt(csv, delimiter=",", names=True)
x, y, nobs = np.atleast_1d(d["x"]), np.atleast_1d(d["y"]), np.atleast_1d(d["n_obs"]).astype(int)
n = len(x)
multi = int((nobs > 1).sum())
print(f"  MapPoints: {n}")
print(f"  multi-observed (n_obs>1, refined): {multi} ({100*multi/max(1,n):.0f}%)")
print(f"  n_obs: max={nobs.max()} mean={nobs.mean():.2f} "
      f"| hist 1:{(nobs==1).sum()} 2:{(nobs==2).sum()} 3:{(nobs==3).sum()} 4+:{(nobs>=4).sum()}")

fig, ax = plt.subplots(1, 2, figsize=(19, 9))
# left: the map coloured by confidence
order = np.argsort(nobs)  # draw low-confidence first so trusted points sit on top
sc = ax[0].scatter(x[order], y[order], c=nobs[order], s=4, cmap="viridis",
                   vmin=1, vmax=max(2, np.percentile(nobs, 99)), linewidths=0)
ax[0].set_aspect("equal"); ax[0].grid(True, alpha=0.3)
ax[0].set_title(f"Consensus map — coloured by visit confidence (n_obs)\n"
                f"{n} MapPoints, {multi} multi-observed")
ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
plt.colorbar(sc, ax=ax[0], label="times re-observed (n_obs)", shrink=0.8)
# right: n_obs histogram (the maturity distribution)
ax[1].hist(np.clip(nobs, 1, 8), bins=np.arange(0.5, 9.5, 1), color="#2ca02c",
           edgecolor="white")
ax[1].set_title("MapPoint maturity (how many visits confirmed each point)")
ax[1].set_xlabel("n_obs (8 = 8 or more)"); ax[1].set_ylabel("MapPoints")
ax[1].grid(True, alpha=0.3, axis="y")
plt.tight_layout(); plt.savefig(out, dpi=100)
print(f"  wrote {out}")
