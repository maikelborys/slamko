#!/usr/bin/env python3
# T2 differential view (PLAN_ROBUSTNESS_01 §7): overlay a STRESS run's map on the
# GOLDEN reference map of the SAME route. Both maps live in their own map frame
# anchored at the (shared) start, so coherent stress submaps overlay the golden;
# garbage/distorted ones show as displaced or misshapen clouds.
#
#   python3 scripts/plot_compare_maps.py \
#       --golden-map results/r0/suave_golden/map  --golden-traj results/r0/suave_golden/graph.tum \
#       --stress-map results/r0/suave_blackout/map --stress-traj results/r0/suave_blackout/graph.tum \
#       --out results/r0/cmp_suave_blackout.html --title "Suave: golden vs blackout"
#
# Pure-geometry overlay — NO alignment is applied (the point is to SEE drift, not
# hide it under Umeyama). Reuses plot_neverlost loaders (SMP1-SMP5).
import argparse, os, sys
import numpy as np
import plotly.graph_objects as go
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_neverlost import load_tum, load_prior_map


def _sub(p, n):
    return p if len(p) <= n else p[:: (len(p) + n - 1) // n]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--golden-map", required=True)
    ap.add_argument("--golden-traj", default="")
    ap.add_argument("--stress-map", required=True)
    ap.add_argument("--stress-traj", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="golden vs stress")
    ap.add_argument("--max-landmarks", type=int, default=40000)
    ap.add_argument("--match-thr", type=float, default=0.10,
                    help="m; a stress lm within this of a golden lm counts as the SAME point")
    a = ap.parse_args()

    g = _sub(load_prior_map(a.golden_map), a.max_landmarks)
    s = _sub(load_prior_map(a.stress_map), a.max_landmarks)

    # Geometric association (proxy for ORB-SLAM's descriptor data-association): a
    # stress landmark is "the same physical point" as the golden if a golden point
    # sits within match_thr. Splits stress into matched (green) vs stress-only (red).
    from scipy.spatial import cKDTree
    d, _ = cKDTree(g).query(s, k=1)
    matched = d < a.match_thr
    frac = 100.0 * matched.mean()
    print(f"golden={len(g)} stress={len(s)}  matched<{a.match_thr*100:.0f}cm="
          f"{frac:.1f}%  median_NN={100*np.median(d):.1f}cm")

    fig = go.Figure()
    fig.add_trace(go.Scatter3d(
        x=g[:, 0], y=g[:, 1], z=g[:, 2], mode="markers",
        marker=dict(size=1.1, color="#888", opacity=0.35), name=f"GOLDEN ({len(g)})"))
    sm, su = s[matched], s[~matched]
    fig.add_trace(go.Scatter3d(
        x=sm[:, 0], y=sm[:, 1], z=sm[:, 2], mode="markers",
        marker=dict(size=1.4, color="#2ca02c", opacity=0.7),
        name=f"SAME ({len(sm)}, {frac:.0f}%)"))
    fig.add_trace(go.Scatter3d(
        x=su[:, 0], y=su[:, 1], z=su[:, 2], mode="markers",
        marker=dict(size=1.8, color="#e1452f", opacity=0.8),
        name=f"STRESS-ONLY ({len(su)})"))

    for traj, col, nm in ((a.golden_traj, "#1f77b4", "golden traj"),
                          (a.stress_traj, "#d62728", "stress traj")):
        if traj and os.path.exists(traj):
            _, x = load_tum(traj)
            fig.add_trace(go.Scatter3d(
                x=x[:, 0], y=x[:, 1], z=x[:, 2], mode="lines",
                line=dict(width=4, color=col), name=nm))

    fig.update_layout(title=a.title, scene=dict(aspectmode="data"),
                      template="plotly_dark")
    fig.write_html(a.out)
    print(f"wrote {a.out}")

    # Static top-down (X-Y) PNG screenshot — matplotlib is always available, and the
    # bird's-eye view is the clearest for "same vs different" without rotating a 3D scene.
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    png = a.out.rsplit(".", 1)[0] + ".png"
    f2, ax = plt.subplots(figsize=(11, 9), dpi=110)
    ax.scatter(g[:, 0], g[:, 1], s=0.5, c="#bbb", alpha=0.4, label=f"GOLDEN ({len(g)})")
    ax.scatter(sm[:, 0], sm[:, 1], s=0.6, c="#2ca02c", alpha=0.6, label=f"SAME ({frac:.0f}%)")
    ax.scatter(su[:, 0], su[:, 1], s=1.4, c="#e1452f", alpha=0.85,
               label=f"STRESS-ONLY ({len(su)})")
    for traj, col, nm in ((a.golden_traj, "#1f77b4", "golden traj"),
                          (a.stress_traj, "#d62728", "stress traj")):
        if traj and os.path.exists(traj):
            _, x = load_tum(traj)
            ax.plot(x[:, 0], x[:, 1], col, lw=1.5, label=nm)
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(a.title, fontsize=10); ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.2); f2.tight_layout(); f2.savefig(png)
    print(f"wrote {png}")


if __name__ == "__main__":
    main()
