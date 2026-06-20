#!/usr/bin/env python3
"""Rotatable 3D Plotly of a (cross-session) fused map — drag to rotate, scroll to zoom,
click legend to isolate. Shows height (z) the 2D views hide: walls as vertical point
planes, levels, and how the session sits INSIDE the prior cloud where it's verified.
  prior map  -> gray
  run submaps -> colored per submap
  trajectory  -> GREEN where verified-against-prior (cross-session match nearby),
                 RED where odom-only (dangling / drift)
Usage: plot_map3d.py --run-dir results/run/x [--prior-dir results/run/suave/map]
       --out results/run/x/map3d.html [--win 18]"""
import argparse, os, re, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_neverlost import load_smap
import plotly.graph_objects as go
import matplotlib.cm as cm

ap = argparse.ArgumentParser()
ap.add_argument("--run-dir", required=True)
ap.add_argument("--prior-dir", default="")
ap.add_argument("--out", required=True)
ap.add_argument("--win", type=int, default=18)
ap.add_argument("--traj", default="graph", choices=["graph", "fused"])
a = ap.parse_args()

def L(p):
    return np.array([list(map(float, l.split())) for l in open(p)
                     if l.strip() and not l.startswith("#")])
cols = cm.tab20(np.linspace(0, 1, 20))
rgb = lambda c: f"rgb({int(c[0]*255)},{int(c[1]*255)},{int(c[2]*255)})"
fig = go.Figure()

if a.prior_dir and os.path.exists(os.path.join(a.prior_dir, "submaps.manifest")):
    pids = [int(x) for x in open(os.path.join(a.prior_dir, "submaps.manifest"))]
    pr = np.concatenate([load_smap(os.path.join(a.prior_dir, f"submap_{i}.smap")) for i in pids])[::6]
    fig.add_trace(go.Scatter3d(x=pr[:, 0], y=pr[:, 1], z=pr[:, 2], mode="markers",
        marker=dict(size=1.2, color="#777777", opacity=0.4), name="prior (original)"))

bids = [int(x) for x in open(os.path.join(a.run_dir, "map", "submaps.manifest"))]
for n, i in enumerate(bids):
    p = load_smap(os.path.join(a.run_dir, "map", f"submap_{i}.smap"))
    if not len(p):
        continue
    p = p[::max(1, len(p) // 1500)]
    fig.add_trace(go.Scatter3d(x=p[:, 0], y=p[:, 1], z=p[:, 2], mode="markers",
        marker=dict(size=1.6, color=rgb(cols[n % 20]), opacity=0.6),
        name=f"sm{i}", showlegend=False))

g = L(os.path.join(a.run_dir, f"{a.traj}.tum")); N = len(g)
cert = set()
log = os.path.join(a.run_dir, "launch.log")
if os.path.exists(log):
    for l in open(log, errors="ignore"):
        m = re.search(r"(X-SESSION prior #\d+|RE-ANCHORED|LOCALIZED).*kf (\d+)", l)
        if m:
            cert.add(int(m.group(2)))
ic = np.zeros(N, bool)
for c in cert:
    ic[max(0, c - a.win):min(N, c + a.win)] = True
fig.add_trace(go.Scatter3d(x=g[ic][:, 1], y=g[ic][:, 2], z=g[ic][:, 3], mode="markers",
    marker=dict(size=3, color="#1a9e1a"), name=f"CERTAIN ({100*ic.mean():.0f}%)"))
fig.add_trace(go.Scatter3d(x=g[~ic][:, 1], y=g[~ic][:, 2], z=g[~ic][:, 3], mode="markers",
    marker=dict(size=3, color="#e03030"), name="UNCERTAIN (dangling/drift)"))

fig.update_layout(template="plotly_dark",
    title=f"{os.path.basename(a.run_dir)} 3D — drag to rotate · green=verified red=dangling",
    scene=dict(aspectmode="data", xaxis_title="x[m]", yaxis_title="y[m]", zaxis_title="z[m]"))
fig.write_html(a.out)
print(f"wrote {a.out}  ({100*ic.mean():.0f}% certain)")
