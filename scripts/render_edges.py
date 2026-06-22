#!/usr/bin/env python3
"""Show the pose-graph edges BY TYPE — hard (stiff) vs soft (uncertain) — so you can see
which connections the system trusts. Reads <run>/map/anchor_edges.csv (from,to,type,...,
sigma_t,sigma_r): type 0 = chain (stiff/hard), 1 = soft (loss-bridge), 2+ = loop. Edge
width ~ 1/sigma_t (thick = stiff/trusted, thin = soft/uncertain). Plus the LOOP CLOSED
inlier strengths and the quality/atlas events from the launch log.

  render_edges.py <run_dir> <launch.log> <out.html>
"""
import os, sys, struct, re
import numpy as np
import plotly.graph_objects as go

run, log, out = sys.argv[1], sys.argv[2], sys.argv[3]
mapd = os.path.join(run, "map")


def anchor_xyz(path):  # submap anchor translation (map frame)
    d = open(path, "rb").read()
    return np.array(struct.unpack_from("<3d", d, 44))


anch = {}
for sid in [int(x) for x in open(os.path.join(mapd, "submaps.manifest")) if x.strip()]:
    p = os.path.join(mapd, f"submap_{sid}.smap")
    if os.path.exists(p):
        anch[sid] = anchor_xyz(p)

# ---- classify edges ----
edges = {"chain": [], "soft": [], "loop": []}
sig = {"chain": [], "soft": [], "loop": []}
ep = os.path.join(mapd, "anchor_edges.csv")
if os.path.exists(ep):
    for l in open(ep):
        if l.startswith("from") or "," not in l:
            continue
        c = l.strip().split(",")
        fr, to, typ = int(c[0]), int(c[1]), int(c[2])
        st = float(c[10])
        kind = "chain" if typ == 0 else ("soft" if typ == 1 else "loop")
        if fr in anch and to in anch:
            edges[kind].append((anch[fr], anch[to]))
            sig[kind].append(st)

# ---- loop closures from the log (strength = inliers) ----
loops = []
if os.path.exists(log):
    for l in open(log, errors="ignore"):
        m = re.search(r"LOOP CLOSED.*inliers=(\d+)", l)
        if m:
            loops.append(int(m.group(1)))
qlost = len(re.findall(r"QUALITY LOST", open(log, errors="ignore").read())) if os.path.exists(log) else 0
abreak = len(re.findall(r"ATLAS BREAK", open(log, errors="ignore").read())) if os.path.exists(log) else 0

print(f"  submaps: {len(anch)}")
print(f"  anchor edges:  chain(hard)={len(edges['chain'])}  soft={len(edges['soft'])}  loop={len(edges['loop'])}")
for k in ("chain", "soft", "loop"):
    if sig[k]:
        print(f"    {k:5s} sigma_t: min={min(sig[k]):.3f} max={max(sig[k]):.3f} m")
print(f"  LOOP CLOSED events: {len(loops)}  inliers={sorted(loops, reverse=True)[:12]}")
if loops:
    strong = sum(1 for x in loops if x >= 40)
    print(f"    strong(>=40 inl, HARD-trustworthy)={strong}  weak(<40)={len(loops) - strong}")
print(f"  QUALITY LOST={qlost}  ATLAS BREAK={abreak}")

# ---- plot ----
fig = go.Figure()
g = os.path.join(run, "graph.tum")
if os.path.exists(g):
    t = np.loadtxt(g)
    if t.ndim == 2:
        fig.add_trace(go.Scatter3d(x=t[:, 1], y=t[:, 2], z=t[:, 3], mode="lines",
                      line=dict(color="#cccccc", width=2), name="trajectory"))
style = {"chain": ("#1f77b4", "chain (HARD/stiff)"), "soft": ("#ff7f0e", "soft (uncertain)"),
         "loop": ("#d62728", "loop (HARD)")}
for k, (col, lab) in style.items():
    xs, ys, zs = [], [], []
    for a, b in edges[k]:
        xs += [a[0], b[0], None]; ys += [a[1], b[1], None]; zs += [a[2], b[2], None]
    if xs:
        w = 8 if k == "loop" else (6 if k == "chain" else 3)
        fig.add_trace(go.Scatter3d(x=xs, y=ys, z=zs, mode="lines",
                      line=dict(color=col, width=w), name=f"{lab} ×{len(edges[k])}"))
for sid, p in anch.items():
    fig.add_trace(go.Scatter3d(x=[p[0]], y=[p[1]], z=[p[2]], mode="markers",
                  marker=dict(size=4, color="black"), showlegend=False))
fig.update_layout(title=f"{os.path.basename(run)} — pose-graph edges by type "
                  f"(HARD blue/red, SOFT orange) · {len(loops)} loops",
                  scene=dict(aspectmode="data"))
fig.write_html(out)
print(f"  wrote {out}")
