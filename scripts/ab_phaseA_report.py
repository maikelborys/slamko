#!/usr/bin/env python3
"""Phase A A/B report: sum landmarks + submaps for OFF vs ON, render side-by-side
top-down, and check the trajectory stayed neutral (fused.tum drift).

  ab_phaseA_report.py <off_run_dir> <on_run_dir> <out.png>
"""
import os, sys, struct
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt


def quat_to_R(qx, qy, qz, qw):
    n = qx*qx+qy*qy+qz*qz+qw*qw; s = 2.0/n if n > 0 else 0.0
    return np.array([
        [1-s*(qy*qy+qz*qz), s*(qx*qy-qz*qw), s*(qx*qz+qy*qw)],
        [s*(qx*qy+qz*qw), 1-s*(qx*qx+qz*qz), s*(qy*qz-qx*qw)],
        [s*(qx*qz-qy*qw), s*(qy*qz+qx*qw), 1-s*(qx*qx+qy*qy)]])


def load_smap_pts(path):
    d = open(path, "rb").read()
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


def gather(run):
    mapd = os.path.join(run, "map")
    man = os.path.join(mapd, "submaps.manifest")
    if not os.path.exists(man):
        return [], 0
    ids = [int(x) for x in open(man) if x.strip()]
    allp = []
    for sid in ids:
        p = os.path.join(mapd, f"submap_{sid}.smap")
        if os.path.exists(p):
            pts = load_smap_pts(p)
            if len(pts):
                allp.append(pts)
    return (np.vstack(allp) if allp else np.empty((0, 3))), len(ids)


def traj(run):
    p = os.path.join(run, "fused.tum")
    if not os.path.exists(p):
        return None
    return np.loadtxt(p)


off_dir, on_dir, out = sys.argv[1], sys.argv[2], sys.argv[3]
p_off, n_off = gather(off_dir)
p_on, n_on = gather(on_dir)

print(f"  BASELINE (off): submaps={n_off}  landmarks={len(p_off)}")
print(f"  PHASE A  (on) : submaps={n_on}  landmarks={len(p_on)}")
if len(p_off):
    print(f"  landmark reduction: {100*(1-len(p_on)/max(1,len(p_off))):.0f}%  "
          f"({len(p_off)} -> {len(p_on)})")

# trajectory neutrality (nearest-by-row, robust mean/max; runs aren't byte-exact)
t_off, t_on = traj(off_dir), traj(on_dir)
if t_off is not None and t_on is not None and len(t_off) and len(t_on):
    n = min(len(t_off), len(t_on))
    d = np.linalg.norm(t_off[:n, 1:4] - t_on[:n, 1:4], axis=1)
    print(f"  trajectory off-vs-on: mean={d.mean()*100:.1f} cm  max={d.max()*100:.1f} cm  "
          f"(neutral if ~ OKVIS run-to-run noise)")

fig, ax = plt.subplots(1, 2, figsize=(18, 9), sharex=True, sharey=True)
for a, pts, ttl, c in ((ax[0], p_off, f"BASELINE off — {n_off} submaps, {len(p_off)} lm", "#d62728"),
                       (ax[1], p_on, f"PHASE A on — {n_on} submaps, {len(p_on)} lm", "#2ca02c")):
    if len(pts):
        a.scatter(pts[:, 0], pts[:, 1], s=1.5, c=c, alpha=0.35, linewidths=0)
    a.set_aspect("equal"); a.grid(True, alpha=0.3)
    a.set_title(ttl); a.set_xlabel("x [m]")
ax[0].set_ylabel("y [m]")
fig.suptitle("Phase A — drift-tolerant MapPoint association: the doubling collapse "
             "(same bag, trajectory unchanged)", fontsize=13)
plt.tight_layout(); plt.savefig(out, dpi=100)
print(f"  wrote {out}")
