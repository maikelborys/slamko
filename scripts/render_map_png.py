#!/usr/bin/env python3
"""Lightweight top-down (XY) map PNG for judging map coherence offline — no viewer.
Draws the optimized graph trajectory (numbered submap anchors) over the landmark
cloud, so a bent corridor / wrong-way return is obvious at a glance.
  render_map_png.py --run-dir results/viz/x --out /tmp/map.png [--traj graph|fused]
"""
import argparse, os, struct, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("--run-dir", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--traj", default="graph", choices=["graph", "fused"])
a = ap.parse_args()


def load_smap(path):
    """Return (anchor_R(3x3), anchor_t(3), landmarks_local Nx3)."""
    with open(path, "rb") as f:
        d = f.read()
    # magic(4) id(u64) anchor: quat xyzw (4 f64 @12) + t (3 f64 @44) = 56B; nk @68
    qx, qy, qz, qw = struct.unpack_from("<4d", d, 12)
    tx, ty, tz = struct.unpack_from("<3d", d, 44)
    # quat (x,y,z,w) -> R
    n = qx * qx + qy * qy + qz * qz + qw * qw
    s = 2.0 / n if n > 0 else 0.0
    R = np.array([
        [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)]])
    t = np.array([tx, ty, tz])
    off = 68
    nk = struct.unpack_from("<Q", d, off)[0]; off += 8
    off += nk * (8 + 8 + 56)  # skip keyframes (id,t,pose)
    nl = struct.unpack_from("<Q", d, off)[0]; off += 8
    pts = np.empty((nl, 3))
    for i in range(nl):
        _id = struct.unpack_from("<Q", d, off)[0]; off += 8
        pts[i] = struct.unpack_from("<3d", d, off); off += 24
        off += 4  # descriptor row (i32)
    return R, t, pts


run, mapd = a.run_dir, os.path.join(a.run_dir, "map")
fig, ax = plt.subplots(figsize=(11, 11))

# landmark cloud (per submap, transformed to map frame), faint grey
ids = []
mani = os.path.join(mapd, "submaps.manifest")
if os.path.exists(mani):
    ids = [int(x) for x in open(mani) if x.strip()]
anchors = {}
for sid in ids:
    p = os.path.join(mapd, f"submap_{sid}.smap")
    if not os.path.exists(p):
        continue
    R, t, pts = load_smap(p)
    anchors[sid] = t
    if len(pts):
        w = (R @ pts.T).T + t
        ax.scatter(w[:, 0], w[:, 1], s=0.5, c="#bbbbbb", alpha=0.35, linewidths=0)

# optimized trajectory, colored by progress
tum = os.path.join(run, f"{a.traj}.tum")
if os.path.exists(tum):
    g = np.array([list(map(float, l.split())) for l in open(tum)
                  if l.strip() and not l.startswith("#")])
    if len(g):
        ax.scatter(g[:, 1], g[:, 2], s=3, c=np.arange(len(g)), cmap="viridis", linewidths=0)
        ax.plot(g[:, 1], g[:, 2], "-", c="#1f77b4", lw=0.6, alpha=0.5)
        ax.plot(g[0, 1], g[0, 2], "go", ms=10, label="start")
        ax.plot(g[-1, 1], g[-1, 2], "rs", ms=10, label="end")

# numbered submap anchors
for sid, t in sorted(anchors.items()):
    ax.plot(t[0], t[1], "k^", ms=7)
    ax.annotate(str(sid), (t[0], t[1]), fontsize=9, fontweight="bold",
                xytext=(4, 4), textcoords="offset points")

ax.set_aspect("equal")
ax.grid(True, alpha=0.3)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title(f"{os.path.basename(run)} — top-down ({a.traj} traj, {len(anchors)} submaps)\n"
             "grey=landmarks  line=trajectory(green start→red end)  ▲=submap anchor")
ax.legend(loc="best")
plt.tight_layout()
plt.savefig(a.out, dpi=110)
print(f"wrote {a.out}  ({len(anchors)} submaps)")
