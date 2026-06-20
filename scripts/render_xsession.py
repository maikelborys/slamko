#!/usr/bin/env python3
"""Top-down overlay of a PRIOR session map (grey) + a NEW session map (colour) +
both trajectories. Each map is in its OWN OKVIS frame (sessions start at their own
origin); if cross-session reloc has NOT welded them, they appear OFFSET — exactly
the "detected-but-not-fused" state. After a real weld they would co-locate.

  render_xsession.py --prior results/euroc/x_s1 --new results/euroc/x_s2 --out /tmp/x.png
"""
import argparse, os, struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def quat_to_R(qx, qy, qz, qw):
    n = qx*qx+qy*qy+qz*qz+qw*qw
    s = 2.0/n if n > 0 else 0.0
    return np.array([
        [1-s*(qy*qy+qz*qz), s*(qx*qy-qz*qw), s*(qx*qz+qy*qw)],
        [s*(qx*qy+qz*qw), 1-s*(qx*qx+qz*qz), s*(qy*qz-qx*qw)],
        [s*(qx*qz-qy*qw), s*(qy*qz+qx*qw), 1-s*(qx*qx+qy*qy)]])


def load_smap_pts(path):
    with open(path, "rb") as f:
        d = f.read()
    qx, qy, qz, qw = struct.unpack_from("<4d", d, 12)
    tx, ty, tz = struct.unpack_from("<3d", d, 44)
    Ra, ta = quat_to_R(qx, qy, qz, qw), np.array([tx, ty, tz])
    off = 68
    nk = struct.unpack_from("<Q", d, off)[0]; off += 8 + nk*(8+8+56)
    nl = struct.unpack_from("<Q", d, off)[0]; off += 8
    pts = np.empty((nl, 3))
    for i in range(nl):
        off += 8
        pts[i] = struct.unpack_from("<3d", d, off); off += 24 + 4
    return (Ra @ pts.T).T + ta if nl else np.empty((0, 3))


def load_map(run_dir):
    mapd = os.path.join(run_dir, "map")
    mf = os.path.join(mapd, "submaps.manifest")
    if not os.path.exists(mf):
        return np.empty((0, 3))
    allp = []
    for sid in [int(x) for x in open(mf) if x.strip()]:
        p = os.path.join(mapd, f"submap_{sid}.smap")
        if os.path.exists(p):
            allp.append(load_smap_pts(p))
    return np.vstack(allp) if allp else np.empty((0, 3))


def load_tum(path):
    if not os.path.exists(path):
        return np.empty((0, 3))
    xs = []
    for l in open(path):
        if l.strip() and not l.startswith("#"):
            v = l.split()
            xs.append([float(v[1]), float(v[2]), float(v[3])])
    return np.array(xs) if xs else np.empty((0, 3))


ap = argparse.ArgumentParser()
ap.add_argument("--prior", required=True)
ap.add_argument("--new", required=True)
ap.add_argument("--out", required=True)
a = ap.parse_args()

pm, nm = load_map(a.prior), load_map(a.new)
pt, nt = load_tum(os.path.join(a.prior, "fused.tum")), load_tum(os.path.join(a.new, "fused.tum"))

fig, ax = plt.subplots(figsize=(13, 11))
if len(pm):
    ax.scatter(pm[:, 0], pm[:, 1], s=0.5, c="#888", alpha=0.30, linewidths=0,
               label=f"prior session map ({len(pm)} lm)")
if len(nm):
    ax.scatter(nm[:, 0], nm[:, 1], s=0.5, c="#1f77b4", alpha=0.30, linewidths=0,
               label=f"new session map ({len(nm)} lm)")
if len(pt):
    ax.plot(pt[:, 0], pt[:, 1], "-", c="#333", lw=1.5, label="prior traj")
if len(nt):
    ax.plot(nt[:, 0], nt[:, 1], "-", c="#d62728", lw=1.5, label="new traj")
ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title(f"cross-session overlay (own frames): {os.path.basename(a.prior)} + {os.path.basename(a.new)}")
ax.legend(loc="best", markerscale=8)
plt.tight_layout(); plt.savefig(a.out, dpi=100)
print(f"wrote {a.out}  prior_lm={len(pm)} new_lm={len(nm)}")
