#!/usr/bin/env python3
"""Cross-session doubling A/B: a COMBINED map of two sessions of the same place.
RAW (session-2 at its own odometry frame) → the house appears TWICE (offset);
CORRECTED (session-2 re-anchored onto session-1 via slamko's cross-session reloc)
→ the two overlap into ONE coherent house. This is slamko's lifelong value made
visual — what a raw-odometry nvblox can't do.

  viz_xsession_doubling.py <s1.ply> <s2_corrected.ply> <s2_raw.ply> <out_prefix>
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def midband(path, half=0.06):
    L = open(path).readlines()
    h = next(i for i, l in enumerate(L) if l.startswith("end_header"))
    nv = next(int(l.split()[2]) for l in L[:h] if l.startswith("element vertex"))
    X = np.array([l.split()[:3] for l in L[h + 1:h + 1 + nv]], float)
    z = X[:, 2]
    m = np.median(z)
    b = (z > m - half) & (z < m + half)
    return X[b, 0], X[b, 1]


s1, s2c, s2r, out = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
x1, y1 = midband(s1)
xc, yc = midband(s2c)
xr, yr = midband(s2r)

fig, axs = plt.subplots(1, 2, figsize=(20, 10))
# RAW: session-2 in its own frame
axs[0].scatter(x1, y1, s=2, c="#1971c2", alpha=0.4, linewidths=0, label="session 1 (casa40)")
axs[0].scatter(xr, yr, s=2, c="#e03131", alpha=0.4, linewidths=0, label="session 2 RAW (own odom frame)")
axs[0].set_title("RAW — two odometry frames → the house DOUBLES")
# CORRECTED: session-2 re-anchored onto session-1
axs[1].scatter(x1, y1, s=2, c="#1971c2", alpha=0.4, linewidths=0, label="session 1 (casa40)")
axs[1].scatter(xc, yc, s=2, c="#2f9e44", alpha=0.4, linewidths=0, label="session 2 CORRECTED (re-anchored)")
axs[1].set_title("CORRECTED — slamko cross-session reloc → ONE house")
for ax in axs:
    ax.set_aspect("equal"); ax.legend(loc="upper right", markerscale=4)
    ax.grid(alpha=0.3); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
fig.suptitle("slamko cross-session bend — combined map of two sessions of the same house",
             fontsize=14)
fig.tight_layout()
fig.savefig(out + "_doubling.png", dpi=120)
print("wrote", out + "_doubling.png")
