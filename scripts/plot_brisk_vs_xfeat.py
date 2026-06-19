#!/usr/bin/env python3
# Compare OKVIS's internal BRISK landmark map (sparse, VIO-survived, high-precision)
# against slamko's XFeat submap landmarks (dense, for place recognition). Density +
# spatial agreement (NN of each BRISK pt to the nearest XFeat pt: if they coincide,
# both front-ends agree on geometry → cross-validation of precision).
#
#   python3 scripts/plot_brisk_vs_xfeat.py \
#       --okvis-csv results/r0/suave_golden/okvis/okvis2-vio-final_map.csv \
#       --xfeat-map results/r0/suave_golden/map \
#       --out results/r0/brisk_vs_xfeat.html --title "Suave: OKVIS BRISK vs slamko XFeat"
import argparse, os, sys
import numpy as np
import plotly.graph_objects as go
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_neverlost import load_prior_map


def load_okvis_csv(path):
    pts, started = [], False
    for ln in open(path):
        ln = ln.strip()
        if ln == "landmarks:":
            started = True; continue
        if not started or not ln:
            continue
        f = ln.split(",")
        if len(f) >= 4:
            pts.append([float(f[1]), float(f[2]), float(f[3])])
    return np.array(pts)


def load_okvis_g2o(path):
    """OKVIS final map: VERTEX_TRACKXYZ id x y z [quality] — the persistent,
    marginalization-survived landmarks (the real relocalization map)."""
    pts = []
    for ln in open(path):
        if ln.startswith("VERTEX_TRACKXYZ"):
            f = ln.split()
            pts.append([float(f[2]), float(f[3]), float(f[4])])
    return np.array(pts)


def _drop_outliers(p, r=80.0):
    """Drop points further than r m from the median centre (OKVIS debug dumps keep
    badly-triangulated landmarks at huge coords)."""
    if len(p) == 0:
        return p
    keep = np.linalg.norm(p - np.median(p, axis=0), axis=1) < r
    return p[keep]


def _sub(p, n):
    return p if len(p) <= n else p[:: (len(p) + n - 1) // n]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--okvis-csv", default="")
    ap.add_argument("--okvis-g2o", default="", help="preferred: the clean persistent map")
    ap.add_argument("--xfeat-map", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="OKVIS BRISK vs slamko XFeat")
    ap.add_argument("--max-landmarks", type=int, default=60000)
    a = ap.parse_args()

    b = load_okvis_g2o(a.okvis_g2o) if a.okvis_g2o else load_okvis_csv(a.okvis_csv)
    b = _drop_outliers(b)
    x = _drop_outliers(_sub(load_prior_map(a.xfeat_map, max_pts=1000000), a.max_landmarks))

    from scipy.spatial import cKDTree
    d, _ = cKDTree(x).query(b, k=1)   # each BRISK pt -> nearest XFeat pt
    print(f"BRISK={len(b)}  XFeat={len(x)}  density_ratio={len(x)/max(len(b),1):.1f}x")
    print(f"BRISK-to-XFeat NN: median={100*np.median(d):.1f}cm  "
          f"<5cm={100*np.mean(d<0.05):.0f}%  <10cm={100*np.mean(d<0.10):.0f}%  "
          f"<20cm={100*np.mean(d<0.20):.0f}%")

    fig = go.Figure()
    fig.add_trace(go.Scatter3d(
        x=x[:, 0], y=x[:, 1], z=x[:, 2], mode="markers",
        marker=dict(size=1.0, color="#888", opacity=0.3), name=f"XFeat dense ({len(x)})"))
    fig.add_trace(go.Scatter3d(
        x=b[:, 0], y=b[:, 1], z=b[:, 2], mode="markers",
        marker=dict(size=2.6, color="#1f77b4", opacity=0.9), name=f"BRISK sparse ({len(b)})"))
    fig.update_layout(title=a.title, scene=dict(aspectmode="data"), template="plotly_dark")
    fig.write_html(a.out)
    print(f"wrote {a.out}")

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    png = a.out.rsplit(".", 1)[0] + ".png"
    f2, ax = plt.subplots(figsize=(11, 9), dpi=110)
    ax.scatter(x[:, 0], x[:, 1], s=0.4, c="#ccc", alpha=0.35, label=f"XFeat dense ({len(x)})")
    ax.scatter(b[:, 0], b[:, 1], s=6, c="#1f77b4", alpha=0.9, label=f"BRISK sparse ({len(b)})")
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(a.title, fontsize=10); ax.legend(loc="best", fontsize=8); ax.grid(alpha=0.2)
    f2.tight_layout(); f2.savefig(png); print(f"wrote {png}")


if __name__ == "__main__":
    main()
