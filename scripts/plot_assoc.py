#!/usr/bin/env python3
"""P1 proof: rotatable 3D landmark map with the RE-RECOGNISED points highlighted.
Grey = all map landmarks; YELLOW = the ones the relocalizer re-associated on revisit
(loop_assoc.csv, written by the node). Shows "slamko recognised these SAME points when
it came back" — the hook the merge (P2) will fuse instead of duplicate.
  plot_assoc.py --run-dir results/viz/x --out-html /tmp/x.html --out-png /tmp/x.png
"""
import argparse, os, struct
import numpy as np
import plotly.graph_objects as go

ap = argparse.ArgumentParser()
ap.add_argument("--run-dir", required=True)
ap.add_argument("--out-html", required=True)
ap.add_argument("--out-png", default="")
ap.add_argument("--traj", default="graph")
a = ap.parse_args()


def load_smap(path):
    with open(path, "rb") as f:
        d = f.read()
    qx, qy, qz, qw = struct.unpack_from("<4d", d, 12)
    tx, ty, tz = struct.unpack_from("<3d", d, 44)
    n = qx*qx+qy*qy+qz*qz+qw*qw; s = 2.0/n if n > 0 else 0.0
    R = np.array([
        [1-s*(qy*qy+qz*qz), s*(qx*qy-qz*qw), s*(qx*qz+qy*qw)],
        [s*(qx*qy+qz*qw), 1-s*(qx*qx+qz*qz), s*(qy*qz-qx*qw)],
        [s*(qx*qz-qy*qw), s*(qy*qz+qx*qw), 1-s*(qx*qx+qy*qy)]])
    t = np.array([tx, ty, tz])
    off = 68
    nk = struct.unpack_from("<Q", d, off)[0]; off += 8
    off += nk*(8+8+56)
    nl = struct.unpack_from("<Q", d, off)[0]; off += 8
    ids = np.empty(nl, np.uint64); pts = np.empty((nl, 3))
    for i in range(nl):
        ids[i] = struct.unpack_from("<Q", d, off)[0]; off += 8
        pts[i] = struct.unpack_from("<3d", d, off); off += 24 + 4
    return R, t, ids, pts


mapd = os.path.join(a.run_dir, "map")
ids = [int(x) for x in open(os.path.join(mapd, "submaps.manifest")) if x.strip()]

# (submap_id, lm_id) -> world position
pos = {}
allw = []
for sid in ids:
    p = os.path.join(mapd, f"submap_{sid}.smap")
    if not os.path.exists(p):
        continue
    R, t, lids, pts = load_smap(p)
    w = (R @ pts.T).T + t
    allw.append(w)
    for k in range(len(lids)):
        pos[(sid, int(lids[k]))] = w[k]

# recognised correspondences
recog = set()
af = os.path.join(mapd, "loop_assoc.csv")
if os.path.exists(af):
    for l in open(af):
        c = l.strip().split(",")
        if len(c) == 3:
            recog.add((int(c[1]), int(c[2])))
rec_pts = np.array([pos[k] for k in recog if k in pos]) if recog else np.empty((0, 3))
allw = np.concatenate(allw) if allw else np.empty((0, 3))

fig = go.Figure()
if len(allw):
    s = allw[::3]
    fig.add_trace(go.Scatter3d(x=s[:, 0], y=s[:, 1], z=s[:, 2], mode="markers",
        marker=dict(size=1.3, color="#999999", opacity=0.35), name=f"map landmarks ({len(allw)})"))
if len(rec_pts):
    fig.add_trace(go.Scatter3d(x=rec_pts[:, 0], y=rec_pts[:, 1], z=rec_pts[:, 2], mode="markers",
        marker=dict(size=4, color="#FFD000"), name=f"RE-RECOGNISED on revisit ({len(rec_pts)})"))
tum = os.path.join(a.run_dir, f"{a.traj}.tum")
if os.path.exists(tum):
    g = np.array([list(map(float, l.split())) for l in open(tum)
                  if l.strip() and not l.startswith("#")])
    if len(g):
        fig.add_trace(go.Scatter3d(x=g[:, 1], y=g[:, 2], z=g[:, 3], mode="lines",
            line=dict(color="#1f77b4", width=3), name="trajectory"))

fig.update_layout(template="plotly_dark",
    title=f"P1 proof — {len(rec_pts)} landmarks RE-RECOGNISED on revisit (yellow). Drag to rotate.",
    scene=dict(aspectmode="data", xaxis_title="x[m]", yaxis_title="y[m]", zaxis_title="z[m]"))
fig.write_html(a.out_html)
print(f"wrote {a.out_html}  ({len(rec_pts)} recognised of {len(allw)} landmarks)")
if a.out_png:
    try:
        fig.update_layout(scene_camera=dict(eye=dict(x=1.6, y=1.6, z=1.2)))
        fig.write_image(a.out_png, width=1200, height=1000, scale=1)
        print(f"wrote {a.out_png}")
    except Exception as e:
        print(f"png export failed: {e}")
