#!/usr/bin/env python3
"""Top-down (XY) + side (XZ) PNG of an XFeat landmark map straight from a dir of
submap_*.smap files (no manifest / graph.tum needed) — for judging map coherence:
bent walls, doubled surfaces, wrong-way returns are obvious at a glance.
  render_submaps_png.py --dir <dir with submap_*.smap> --out /tmp/map.png
"""
import argparse, glob, os, struct, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_smap(path):
    """Return (anchor_R(3x3), anchor_t(3), landmarks_local Nx3). Mirrors render_map_png.py."""
    with open(path, "rb") as f:
        d = f.read()
    qx, qy, qz, qw = struct.unpack_from("<4d", d, 12)
    tx, ty, tz = struct.unpack_from("<3d", d, 44)
    n = qx * qx + qy * qy + qz * qz + qw * qw
    s = 2.0 / n if n > 0 else 0.0
    R = np.array([
        [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)]])
    t = np.array([tx, ty, tz])
    off = 68
    nk = struct.unpack_from("<Q", d, off)[0]; off += 8
    off += nk * (8 + 8 + 56)
    nl = struct.unpack_from("<Q", d, off)[0]; off += 8
    pts = np.empty((nl, 3))
    for i in range(nl):
        off += 8
        pts[i] = struct.unpack_from("<3d", d, off); off += 24
        off += 4
    return R, t, pts


ap = argparse.ArgumentParser()
ap.add_argument("--dir", required=True)
ap.add_argument("--out", required=True)
a = ap.parse_args()

files = sorted(glob.glob(os.path.join(a.dir, "submap_*.smap")),
               key=lambda p: int(p.split("submap_")[-1].split(".")[0]))
if not files:
    sys.exit(f"no submap_*.smap in {a.dir}")

cmap = plt.cm.turbo(np.linspace(0, 1, len(files)))
fig, (axT, axS) = plt.subplots(1, 2, figsize=(18, 8))
anchors = []
total = 0
for k, f in enumerate(files):
    R, t, pts = load_smap(f)
    if len(pts) == 0:
        continue
    w = (R @ pts.T).T + t           # local -> world
    total += len(w)
    anchors.append(t)
    sid = os.path.basename(f).split("submap_")[-1].split(".")[0]
    axT.scatter(w[:, 0], w[:, 1], s=0.4, c=[cmap[k]], alpha=0.45, label=f"sm{sid} ({len(w)})")
    axS.scatter(w[:, 0], w[:, 2], s=0.4, c=[cmap[k]], alpha=0.45)
anchors = np.array(anchors)
for ax, (i, j), name in [(axT, (0, 1), "TOP-DOWN  (X-Y)"), (axS, (0, 2), "SIDE  (X-Z, height)")]:
    if len(anchors):
        ax.plot(anchors[:, i], anchors[:, j], "-o", c="k", ms=5, lw=1.2, label="submap anchors")
    ax.set_title(name)
    ax.set_xlabel("X (m)"); ax.set_ylabel(["X", "Y", "Z"][j] + " (m)")
    ax.set_aspect("equal", "datalim"); ax.grid(alpha=0.3)
axT.legend(markerscale=8, fontsize=7, loc="best")
fig.suptitle(f"XFeat landmark map — {os.path.basename(a.dir.rstrip('/'))}  |  "
             f"{len(files)} submaps  {total} landmarks", fontsize=13)
fig.tight_layout()
fig.savefig(a.out, dpi=110)
print(f"wrote {a.out}  ({len(files)} submaps, {total} landmarks)")
