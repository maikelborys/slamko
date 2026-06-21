#!/usr/bin/env python3
"""Top-down render of the Atlas COLOURED BY COMPONENT (disjoint islands, etapa 1c).

With atlas_break_on_loss, a tracking loss BREAKS the map into a new disjoint component
that floats until a feature match welds it. This colours each component separately so a
brutal run shows as FRAGMENTS (not one warped map) — exactly the user's mental model:
"el mapa estará partido a varias porque no tendrán coincidencias".

  render_atlas.py --run-dir results/mp/brutal_atlas --out /tmp/atlas.png

Reads map/components.csv (submap_id,component); falls back to one component if absent.
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


ap = argparse.ArgumentParser()
ap.add_argument("--run-dir", required=True)
ap.add_argument("--out", required=True)
a = ap.parse_args()

mapd = os.path.join(a.run_dir, "map")
ids = [int(x) for x in open(os.path.join(mapd, "submaps.manifest")) if x.strip()]

comp = {}
cpath = os.path.join(mapd, "components.csv")
if os.path.exists(cpath):
    for l in open(cpath):
        if l.strip() and not l.startswith("submap_id"):
            sid, c = l.split(","); comp[int(sid)] = int(c)
n_comp = (max(comp.values()) + 1) if comp else 1

cmap = plt.get_cmap("tab10")
fig, ax = plt.subplots(figsize=(12, 11))
per_comp = {}
for sid in ids:
    p = os.path.join(mapd, f"submap_{sid}.smap")
    if not os.path.exists(p):
        continue
    pts = load_smap_pts(p)
    if not len(pts):
        continue
    c = comp.get(sid, 0)
    per_comp.setdefault(c, []).append(pts)

for c in sorted(per_comp):
    allp = np.vstack(per_comp[c])
    ax.scatter(allp[:, 0], allp[:, 1], s=0.6, c=[cmap(c % 10)], alpha=0.4,
               linewidths=0, label=f"map {c} ({len(allp)} lm)")

ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title(f"{os.path.basename(a.run_dir)} — Atlas by component "
             f"({n_comp} disjoint map{'s' if n_comp != 1 else ''})")
ax.legend(loc="best", markerscale=8)
plt.tight_layout(); plt.savefig(a.out, dpi=100)
print(f"wrote {a.out}  components={n_comp} submaps={len(ids)}")
