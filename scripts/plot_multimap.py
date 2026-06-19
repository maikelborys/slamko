#!/usr/bin/env python3
# Multi-map (Atlas) structure viz — the "federation of islands" view. Each submap
# is an island (distinct color); its landmark centroid is the anchor; HARD edges
# (verified welds, parsed from the run log "LOOP CLOSED: kf X -> submap Y") are
# drawn between connected submap anchors. SOFT (dead-reckoning) edges will render
# here too once R1.3 lands (PLAN_ROBUSTNESS_01).
#
#   python3 scripts/plot_multimap.py --map-dir results/r0/suave_golden_dedup/map \
#       --log results/r0/suave_golden_dedup/launch.log --out results/r0/multimap.html
import argparse, os, re, sys
import numpy as np
import plotly.graph_objects as go
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.cm as cm
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_neverlost import load_smap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-dir", required=True)
    ap.add_argument("--log", default="")
    ap.add_argument("--kf-per-submap", type=int, default=50)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    ids = [int(x) for x in open(os.path.join(a.map_dir, "submaps.manifest"))]
    subs, cents = {}, {}
    for i in ids:
        p = load_smap(os.path.join(a.map_dir, f"submap_{i}.smap"))
        subs[i] = p
        cents[i] = p.mean(axis=0) if len(p) else np.zeros(3)

    # edges: prefer the persisted anchor_edges.csv (from,to,type); else parse the
    # log (all hard). type 0=chain-odom, 1=SOFT (dead-reckoning across a loss),
    # 2=HARD (verified weld).
    import csv
    edges = []  # (from, to, type)
    ecsv = os.path.join(a.map_dir, "anchor_edges.csv")
    if os.path.exists(ecsv):
        with open(ecsv) as fh:
            for rr in csv.DictReader(fh):
                f_, t_, ty = int(rr["from"]), int(rr["to"]), int(rr["type"])
                if f_ in cents and t_ in cents and f_ != t_:
                    edges.append((f_, t_, ty))
    elif a.log and os.path.exists(a.log):
        for m in re.finditer(r"LOOP CLOSED: kf (\d+) -> submap (\d+)", open(a.log).read()):
            qs, tgt = int(m.group(1)) // a.kf_per_submap, int(m.group(2))
            if qs in cents and tgt in cents and qs != tgt:
                edges.append((qs, tgt, 2))
    edges = sorted(set(edges))
    n_by = {0: 0, 1: 0, 2: 0}
    for _, _, ty in edges:
        n_by[ty] += 1

    colors = cm.tab20(np.linspace(0, 1, 20))
    rgb = lambda c: f"rgb({int(c[0]*255)},{int(c[1]*255)},{int(c[2]*255)})"

    # ---- interactive plotly 3D ----
    fig = go.Figure()
    for n, i in enumerate(ids):
        p = subs[i]
        if len(p) > 6000:
            p = p[::len(p) // 6000]
        fig.add_trace(go.Scatter3d(
            x=p[:, 0], y=p[:, 1], z=p[:, 2], mode="markers",
            marker=dict(size=1.3, color=rgb(colors[n % 20]), opacity=0.5),
            name=f"submap {i} ({len(subs[i])} lm)"))
    fig.add_trace(go.Scatter3d(
        x=[cents[i][0] for i in ids], y=[cents[i][1] for i in ids], z=[cents[i][2] for i in ids],
        mode="markers+text", marker=dict(size=6, color="white", symbol="diamond"),
        text=[str(i) for i in ids], textposition="top center", name="anchors"))
    estyle = {0: ("#888888", 2, "solid", f"chain-odom ({n_by[0]})"),
              1: ("#ff7f0e", 5, "dash", f"SOFT dead-reckoning ({n_by[1]})"),
              2: ("#2ca02c", 6, "solid", f"HARD verified weld ({n_by[2]})")}
    shown = set()
    for (f_, t_, ty) in edges:
        col, w, dash, nm = estyle[ty]
        fig.add_trace(go.Scatter3d(
            x=[cents[f_][0], cents[t_][0]], y=[cents[f_][1], cents[t_][1]],
            z=[cents[f_][2], cents[t_][2]], mode="lines",
            line=dict(width=w, color=col, dash=dash),
            name=nm, showlegend=(ty not in shown)))
        shown.add(ty)
    fig.update_layout(
        title=f"Multi-map Atlas: {len(ids)} islands · {n_by[0]} odom / {n_by[1]} soft / {n_by[2]} hard edges",
        scene=dict(aspectmode="data"), template="plotly_dark")
    fig.write_html(a.out)
    print(f"wrote {a.out}  ({len(ids)} submaps · {n_by[0]} odom / {n_by[1]} soft / {n_by[2]} hard)")

    # ---- static top-down PNG ----
    png = a.out.rsplit(".", 1)[0] + ".png"
    f2, ax = plt.subplots(figsize=(11, 9), dpi=110)
    for n, i in enumerate(ids):
        p = subs[i]
        if len(p) > 20000:
            p = p[::len(p) // 20000]
        ax.scatter(p[:, 0], p[:, 1], s=0.5, color=colors[n % 20], alpha=0.5)
    pstyle = {0: ("#888888", 1.2, "-"), 1: ("#ff7f0e", 2.6, "--"), 2: ("#2ca02c", 2.6, "-")}
    for (f_, t_, ty) in edges:
        col, lw, ls = pstyle[ty]
        ax.plot([cents[f_][0], cents[t_][0]], [cents[f_][1], cents[t_][1]],
                ls, color=col, lw=lw, zorder=4)
    for i in ids:
        ax.scatter(cents[i][0], cents[i][1], s=130, c="k", marker="D", zorder=5)
        ax.annotate(str(i), (cents[i][0], cents[i][1]), color="w", fontsize=8,
                    ha="center", va="center", zorder=6)
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(f"Multi-map Atlas: {len(ids)} islands · gray=odom orange-dash=SOFT(DR) "
                 f"green=HARD  ({n_by[0]}/{n_by[1]}/{n_by[2]})")
    ax.grid(alpha=0.2); f2.tight_layout(); f2.savefig(png)
    print(f"wrote {png}")


if __name__ == "__main__":
    main()
