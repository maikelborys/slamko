#!/usr/bin/env python3
"""Static (no-timeline) map-coherence debug viz — light, interactive Plotly.
Top-down 2D: landmarks colored per submap (legend-toggle each), the optimized
trajectory as a line, numbered submap anchors, typed edges (gray=odom,
orange-dash=SOFT, green=HARD). Use it to spot where the map twists/drifts.

Usage: plot_map_static.py --run-dir results/run/x --out results/run/x/mapdebug.html
       [--traj graph|fused]  (graph = loop-optimized; default)"""
import argparse, os, re, csv, sys, struct
import numpy as np
import plotly.graph_objects as go
import matplotlib.cm as cm
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_neverlost import load_smap

def anchor_pose(path):
    """The submap ANCHOR translation (camera pose origin in map frame) — the
    correct place to draw the anchor, NOT the landmark centroid (which sits on
    the walls the submap sees)."""
    with open(path, "rb") as fh:
        d = fh.read(76)
    t = struct.unpack_from("<3d", d, 4 + 8 + 32)   # after magic+id+quat(4d) -> translation
    return np.array(t)

ap = argparse.ArgumentParser()
ap.add_argument("--run-dir", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--traj", default="graph", choices=["graph", "fused", "provider"])
ap.add_argument("--prior-dir", default="", help="overlay a prior map (gray) in the same frame")
a = ap.parse_args()
MAP = os.path.join(a.run_dir, "map")

def load_tum(p):
    if not os.path.exists(p): return np.zeros((0, 8))
    r = [list(map(float, l.split())) for l in open(p) if l.strip() and not l.startswith("#")]
    return np.array(r) if r else np.zeros((0, 8))
traj = load_tum(os.path.join(a.run_dir, f"{a.traj}.tum"))
if not len(traj): traj = load_tum(os.path.join(a.run_dir, "fused.tum"))

ids = [int(x) for x in open(os.path.join(MAP, "submaps.manifest"))]
subs, cents = {}, {}
for i in ids:
    sp = os.path.join(MAP, f"submap_{i}.smap")
    p = load_smap(sp)
    subs[i] = p
    cents[i] = anchor_pose(sp)   # anchor = camera pose origin, not landmark centroid

edges = []
ec = os.path.join(MAP, "anchor_edges.csv")
if os.path.exists(ec):
    for rr in csv.DictReader(open(ec)):
        f_, t_, ty = int(rr["from"]), int(rr["to"]), int(rr["type"])
        if f_ in cents and t_ in cents and f_ != t_: edges.append((f_, t_, ty))

colors = cm.tab20(np.linspace(0, 1, 20))
rgb = lambda c: f"rgb({int(c[0]*255)},{int(c[1]*255)},{int(c[2]*255)})"
ES = {0: ("#888888", "solid", "odom"), 1: ("#ff7f0e", "dash", "SOFT"), 2: ("#2ca02c", "solid", "HARD")}

fig = go.Figure()
# optional prior map (gray, behind everything)
if a.prior_dir and os.path.exists(os.path.join(a.prior_dir, "submaps.manifest")):
    pids = [int(x) for x in open(os.path.join(a.prior_dir, "submaps.manifest"))]
    pp = np.concatenate([load_smap(os.path.join(a.prior_dir, f"submap_{i}.smap")) for i in pids], axis=0)
    pp = pp[::max(1, len(pp) // 20000)]
    fig.add_trace(go.Scattergl(x=pp[:, 0], y=pp[:, 1], mode="markers",
        marker=dict(size=2, color="#555555"), name=f"PRIOR ({len(pids)}sm)"))
# landmarks per submap (toggleable)
for n, i in enumerate(ids):
    p = subs[i]
    if not len(p): continue
    pp = p[::max(1, len(p) // 5000)]
    fig.add_trace(go.Scattergl(x=pp[:, 0], y=pp[:, 1], mode="markers",
        marker=dict(size=2.5, color=rgb(colors[n % 20])), name=f"sm{i} ({len(p)}lm)"))
# trajectory
if len(traj):
    fig.add_trace(go.Scattergl(x=traj[:, 1], y=traj[:, 2], mode="lines",
        line=dict(color="#ffd700", width=2), name=f"traj ({a.traj})"))
# edges
shown = set()
for (f_, t_, ty) in edges:
    col, dash, nm = ES[ty]
    fig.add_trace(go.Scatter(x=[cents[f_][0], cents[t_][0]], y=[cents[f_][1], cents[t_][1]],
        mode="lines", line=dict(color=col, width=3, dash=dash),
        name=nm, legendgroup=nm, showlegend=(ty not in shown))); shown.add(ty)
# numbered anchors
fig.add_trace(go.Scatter(x=[cents[i][0] for i in ids], y=[cents[i][1] for i in ids],
    mode="markers+text", marker=dict(size=16, color="white", symbol="diamond",
    line=dict(color="black", width=1)), text=[str(i) for i in ids],
    textposition="middle center", textfont=dict(size=9, color="black"), name="anchors"))
ne = {0: 0, 1: 0, 2: 0}
for _, _, t in edges: ne[t] += 1
fig.update_layout(template="plotly_dark",
    title=f"{os.path.basename(a.run_dir)} map: {len(ids)} submaps · edges {ne[0]}odom/{ne[1]}soft/{ne[2]}hard · traj={a.traj}",
    xaxis=dict(title="x [m]", scaleanchor="y", scaleratio=1), yaxis=dict(title="y [m]"))
fig.write_html(a.out)

# static png too (for quick inline look)
png = a.out.rsplit(".", 1)[0] + ".png"
f2, ax = plt.subplots(figsize=(9, 11), dpi=110)
if a.prior_dir and os.path.exists(os.path.join(a.prior_dir, "submaps.manifest")):
    pids = [int(x) for x in open(os.path.join(a.prior_dir, "submaps.manifest"))]
    pp = np.concatenate([load_smap(os.path.join(a.prior_dir, f"submap_{i}.smap")) for i in pids], axis=0)
    ax.scatter(pp[::max(1, len(pp)//20000), 0], pp[::max(1, len(pp)//20000), 1], s=0.4, color="#777777", alpha=0.35, zorder=1)
for n, i in enumerate(ids):
    p = subs[i]
    if len(p): ax.scatter(p[::max(1, len(p)//8000), 0], p[::max(1, len(p)//8000), 1], s=0.6, color=colors[n % 20], alpha=0.55)
if len(traj): ax.plot(traj[:, 1], traj[:, 2], color="#ffd700", lw=1.6, zorder=3)
ps = {0: ("#888888", "-"), 1: ("#ff7f0e", "--"), 2: ("#2ca02c", "-")}
for (f_, t_, ty) in edges:
    c, ls = ps[ty]; ax.plot([cents[f_][0], cents[t_][0]], [cents[f_][1], cents[t_][1]], ls, color=c, lw=2.4, zorder=4)
for i in ids:
    ax.scatter(cents[i][0], cents[i][1], s=150, c="w", edgecolors="k", marker="D", zorder=5)
    ax.annotate(str(i), (cents[i][0], cents[i][1]), color="k", fontsize=8, ha="center", va="center", zorder=6)
ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title(f"{os.path.basename(a.run_dir)} map · {len(ids)} submaps · gold=traj({a.traj}) · {ne[1]}soft/{ne[2]}hard")
ax.grid(alpha=0.2); f2.tight_layout(); f2.savefig(png)
print(f"wrote {a.out} + {png}  ({len(ids)} submaps, {ne[0]}/{ne[1]}/{ne[2]} edges)")
