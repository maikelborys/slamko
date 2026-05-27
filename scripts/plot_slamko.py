#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# slamko interactive 3D viz (Plotly → self-contained HTML). The DEFAULT way to
# look at a run: ground truth + Sim3-aligned estimate + the landmark map (and the
# prior map / forced-loss dead-reckoning, when relevant), all in one rotatable,
# zoomable 3D scene with the Sim3-ATE in the title. Pairs with plot_neverlost.py
# (the static PNG gate used by the bench); this one is for inspection.
#
# It reuses plot_neverlost's loaders + never-lost anchor correction + Umeyama, so
# the two never drift: same data, same alignment, different renderer.
#
# usage:
#   python3 scripts/plot_slamko.py --gt GT.tum --est est.tum --landmarks lm.csv \
#       --out run.html [--submaps lm.csv.submaps --pose-epoch est.tum.epoch] \
#       [--prior-map-dir DIR] [--loss s1 e1 ...] [--title "..."] [--max-landmarks 40000]
import argparse
import numpy as np
import plotly.graph_objects as go

from plot_neverlost import (
    load_tum, load_landmarks, load_submaps, load_epoch, load_prior_map,
    correct_landmarks, correct_poses, umeyama, associate,
)


def _subsample(xyz, n):
    if xyz is None or len(xyz) <= n:
        return xyz
    step = (len(xyz) + n - 1) // n
    return xyz[::step]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True)
    ap.add_argument("--est", required=True)
    ap.add_argument("--landmarks", default="")
    ap.add_argument("--out", required=True, help="output .html")
    ap.add_argument("--submaps", default="")
    ap.add_argument("--pose-epoch", default="")
    ap.add_argument("--prior-map-dir", default="")
    ap.add_argument("--loss", nargs="+", type=float, default=None,
                    help="forced-loss windows, flat pairs s1 e1 [s2 e2 ...] (s, rel)")
    ap.add_argument("--min-obs", type=int, default=2)
    ap.add_argument("--max-landmarks", type=int, default=40000,
                    help="cap landmark markers for browser performance")
    ap.add_argument("--title", default="slamko run")
    a = ap.parse_args()

    tg, xg = load_tum(a.gt)
    te, xe = load_tum(a.est)

    submaps = load_submaps(a.submaps) if a.submaps else None
    if submaps and a.pose_epoch:
        ets, esid = load_epoch(a.pose_epoch)
        xe = correct_poses(te, xe, ets, esid, submaps)

    lm_xyz = lm_n = None
    if a.landmarks:
        lid, lm_all, obs = load_landmarks(a.landmarks)
        keep = obs >= a.min_obs
        lid, lm_xyz = lid[keep], lm_all[keep]
        if submaps:
            lm_xyz = correct_landmarks(lid, lm_xyz, submaps)
        lm_n = len(lm_xyz)

    m, idx = associate(te, tg)
    if m.sum() < 10:
        raise SystemExit(f"too few GT↔est associations ({m.sum()})")
    s, R, t = umeyama(xe[m], xg[idx[m]])
    xe_a = (s * (R @ xe.T).T) + t
    rmse = float(np.sqrt(((xe_a[m] - xg[idx[m]]) ** 2).sum(1).mean()))
    lm_a = (s * (R @ lm_xyz.T).T) + t if lm_xyz is not None else None

    prior_a = None
    if a.prior_map_dir:
        prior_xyz = load_prior_map(a.prior_map_dir)
        prior_a = (s * (R @ prior_xyz.T).T) + t

    fig = go.Figure()

    def scatter3d(xyz, **kw):
        return go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2], **kw)

    # Map first (drawn under the trajectories).
    if prior_a is not None:
        p = _subsample(prior_a, a.max_landmarks)
        fig.add_trace(scatter3d(p, mode="markers", name=f"prior map ({len(prior_a)})",
                                marker=dict(size=1.4, color="orange", opacity=0.35)))
    if lm_a is not None:
        p = _subsample(lm_a, a.max_landmarks)
        fig.add_trace(scatter3d(p, mode="markers",
                                name=(f"live map ({lm_n})" if prior_a is not None
                                      else f"landmarks ({lm_n})"),
                                marker=dict(size=1.4, color="#888", opacity=0.45)))
    fig.add_trace(scatter3d(xg, mode="lines", name="ground truth",
                            line=dict(color="black", width=5)))
    fig.add_trace(scatter3d(xe_a, mode="lines", name="estimate (Sim3-aligned)",
                            line=dict(color="royalblue", width=3)))
    if a.loss is not None:
        tr = te - te[0]
        for w, (s0, s1) in enumerate(zip(a.loss[0::2], a.loss[1::2])):
            msk = (tr >= s0) & (tr <= s1)
            if msk.any():
                fig.add_trace(scatter3d(xe_a[msk], mode="lines",
                                        name="forced-loss (dead-reckon)" if w == 0 else None,
                                        showlegend=(w == 0),
                                        line=dict(color="red", width=6)))
    fig.add_trace(scatter3d(xg[:1], mode="markers", name="start",
                            marker=dict(size=6, color="limegreen")))

    tag = f" · {len(submaps)} submaps (anchor-corrected)" if submaps else ""
    fig.update_layout(
        title=f"{a.title}   |   Sim3-ATE {rmse*100:.1f} cm   |   {m.sum()} poses{tag}",
        scene=dict(xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
                   aspectmode="data"),
        legend=dict(itemsizing="constant"),
        margin=dict(l=0, r=0, t=40, b=0),
        template="plotly_white",
    )
    fig.write_html(a.out, include_plotlyjs=True)  # embed plotly.js → opens offline anywhere
    print(f"wrote {a.out}  (Sim3-ATE={rmse*100:.2f} cm, scale={s:.4f}, "
          f"{m.sum()} assoc poses, {lm_n or 0} landmarks)")


if __name__ == "__main__":
    main()
