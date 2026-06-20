#!/usr/bin/env python3
"""Render the map twice: with the FROZEN .smap anchors (seal-time) vs the loop-CORRECTED
anchors (each submap re-placed at its first keyframe's optimized pose from graph.tum,
matched by timestamp). If the doubling collapses in the corrected view, the map was
coherent all along and the 'doubling' was a render/persist artifact (stale anchors).
  render_map_corrected.py --run-dir results/viz/x --out /tmp/x.png
"""
import argparse, os, struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("--run-dir", required=True)
ap.add_argument("--out", required=True)
a = ap.parse_args()


def quat_to_R(qx, qy, qz, qw):
    n = qx*qx+qy*qy+qz*qz+qw*qw
    s = 2.0/n if n > 0 else 0.0
    return np.array([
        [1-s*(qy*qy+qz*qz), s*(qx*qy-qz*qw), s*(qx*qz+qy*qw)],
        [s*(qx*qy+qz*qw), 1-s*(qx*qx+qz*qz), s*(qy*qz-qx*qw)],
        [s*(qx*qz-qy*qw), s*(qy*qz+qx*qw), 1-s*(qx*qx+qy*qy)]])


def load_smap(path):
    with open(path, "rb") as f:
        d = f.read()
    qx, qy, qz, qw = struct.unpack_from("<4d", d, 12)
    tx, ty, tz = struct.unpack_from("<3d", d, 44)
    Ra, ta = quat_to_R(qx, qy, qz, qw), np.array([tx, ty, tz])
    off = 68
    nk = struct.unpack_from("<Q", d, off)[0]; off += 8
    first_kf_t = struct.unpack_from("<d", d, off + 8)[0] if nk else 0.0  # kf0: id(8) then t
    off += nk*(8+8+56)
    nl = struct.unpack_from("<Q", d, off)[0]; off += 8
    pts = np.empty((nl, 3))
    for i in range(nl):
        off += 8
        pts[i] = struct.unpack_from("<3d", d, off); off += 24 + 4
    return Ra, ta, first_kf_t, pts


# corrected poses by timestamp from graph.tum
gt = {}
gpath = os.path.join(a.run_dir, "graph.tum")
if os.path.exists(gpath):
    for l in open(gpath):
        if l.strip() and not l.startswith("#"):
            v = list(map(float, l.split()))
            gt[round(v[0], 6)] = v
gt_ts = np.array(sorted(gt.keys())) if gt else np.array([])


def corrected_anchor(first_kf_t):
    if not len(gt_ts):
        return None
    i = int(np.argmin(np.abs(gt_ts - first_kf_t)))
    if abs(gt_ts[i] - first_kf_t) > 0.5:
        return None
    v = gt[gt_ts[i]]
    return quat_to_R(v[4], v[5], v[6], v[7]), np.array([v[1], v[2], v[3]])


mapd = os.path.join(a.run_dir, "map")
ids = [int(x) for x in open(os.path.join(mapd, "submaps.manifest")) if x.strip()]
fig, axs = plt.subplots(1, 2, figsize=(20, 11))
n_corr = 0
for sid in ids:
    p = os.path.join(mapd, f"submap_{sid}.smap")
    if not os.path.exists(p):
        continue
    Ra, ta, fkt, pts = load_smap(p)
    if not len(pts):
        continue
    # FROZEN
    wf = (Ra @ pts.T).T + ta
    axs[0].scatter(wf[:, 0], wf[:, 1], s=0.5, c="#999", alpha=0.35, linewidths=0)
    # CORRECTED
    ca = corrected_anchor(fkt)
    if ca is not None:
        Rc, tc = ca
        # landmarks are submap-local (relative to seal anchor); re-place under corrected anchor
        local = (Ra.T @ (wf - ta).T).T            # back to local
        wc = (Rc @ local.T).T + tc
        n_corr += 1
    else:
        wc = wf
    axs[1].scatter(wc[:, 0], wc[:, 1], s=0.5, c="#999", alpha=0.35, linewidths=0)

for ax, t in zip(axs, [f"FROZEN .smap anchors (seal-time)",
                       f"loop-CORRECTED anchors ({n_corr}/{len(ids)} matched)"]):
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_xlabel("x[m]"); ax.set_ylabel("y[m]"); ax.set_title(t)
fig.suptitle(f"{os.path.basename(a.run_dir)} — frozen vs loop-corrected anchors", fontsize=14)
plt.tight_layout()
plt.savefig(a.out, dpi=100)
print(f"wrote {a.out}  ({n_corr}/{len(ids)} submaps re-anchored)")
