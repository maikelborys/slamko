#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Offline viz: slamko estimated trajectory vs EuRoC ground truth + the landmark
# map. The estimate is Sim3-aligned to GT (Umeyama, timestamp-associated), and
# the SAME transform is applied to the landmarks so everything overlays in the GT
# frame. Renders top-down (X-Y) + side (X-Z) panels to a PNG. No ROS.
#
# usage: plot_neverlost.py --gt GT.tum --est est.tum --landmarks lm.csv --out fig.png
#                          [--loss 25 28] [--title "..."]
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_tum(path):
    d = np.loadtxt(path)
    return d[:, 0], d[:, 1:4]  # t, xyz


def load_landmarks(path):
    d = np.loadtxt(path, delimiter=",", skiprows=1)  # id,x,y,z,obs
    if d.ndim == 1:
        d = d[None, :]
    return d[:, 1:4], d[:, 4]


def umeyama(src, dst):
    """Sim3: find s,R,t so dst ~ s*R*src + t (Umeyama 1991)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    C = d0.T @ s0 / len(src)
    U, D, Vt = np.linalg.svd(C)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var = (s0 ** 2).sum() / len(src)
    s = float(np.trace(np.diag(D) @ S) / var)
    t = mu_d - s * R @ mu_s
    return s, R, t


def associate(t_est, t_gt, tol=0.02):
    idx = np.clip(np.searchsorted(t_gt, t_est), 1, len(t_gt) - 1)
    idx = np.where(np.abs(t_est - t_gt[idx - 1]) < np.abs(t_est - t_gt[idx]), idx - 1, idx)
    return (np.abs(t_gt[idx] - t_est) < tol), idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True)
    ap.add_argument("--est", required=True)
    ap.add_argument("--landmarks", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--loss", nargs="+", type=float, default=None,
                    help="forced-loss windows as flat pairs: s1 e1 [s2 e2 ...] (s, rel)")
    ap.add_argument("--min-obs", type=int, default=2, help="drop landmarks seen < this")
    ap.add_argument("--title", default="slamko never-lost")
    a = ap.parse_args()

    tg, xg = load_tum(a.gt)
    te, xe = load_tum(a.est)
    m, idx = associate(te, tg)
    if m.sum() < 10:
        raise SystemExit(f"too few GT↔est associations ({m.sum()})")
    s, R, t = umeyama(xe[m], xg[idx[m]])           # est → GT frame
    xe_a = (s * (R @ xe.T).T) + t                  # aligned estimate
    rmse = float(np.sqrt(((xe_a[m] - xg[idx[m]]) ** 2).sum(1).mean()))

    fig, ax = plt.subplots(1, 2, figsize=(15, 7))
    panels = [(0, 1, "X (m)", "Y (m)", "top-down"), (0, 2, "X (m)", "Z (m)", "side")]
    for k, (i, j, xl, yl, name) in enumerate(panels):
        if a.landmarks:
            lm, obs = load_landmarks(a.landmarks)
            lm = lm[obs >= a.min_obs]
            lm_a = (s * (R @ lm.T).T) + t
            ax[k].scatter(lm_a[:, i], lm_a[:, j], s=1.0, c="0.7", alpha=0.35,
                          label=f"landmarks ({len(lm_a)})", rasterized=True)
        ax[k].plot(xg[:, i], xg[:, j], "-", c="k", lw=2.0, label="ground truth")
        ax[k].plot(xe_a[:, i], xe_a[:, j], "-", c="tab:blue", lw=1.3, label="estimate (Sim3-aligned)")
        if a.loss is not None:
            tr = te - te[0]  # --loss is relative-to-start (est stamps are absolute)
            for w, (s0, s1) in enumerate(zip(a.loss[0::2], a.loss[1::2])):
                lm_mask = (tr >= s0) & (tr <= s1)
                if lm_mask.any():
                    ax[k].plot(xe_a[lm_mask, i], xe_a[lm_mask, j], "-", c="tab:red", lw=3.0,
                               label="forced-loss (dead-reckon)" if w == 0 else None)
        ax[k].plot(xg[0, i], xg[0, j], "o", c="g", ms=9, label="start")
        ax[k].set_xlabel(xl); ax[k].set_ylabel(yl); ax[k].set_title(name)
        ax[k].axis("equal"); ax[k].grid(alpha=0.3)
    ax[0].legend(loc="best", fontsize=9)
    fig.suptitle(f"{a.title}   |   Sim3-ATE {rmse*100:.1f} cm   |   {m.sum()} poses", fontsize=13)
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    print(f"wrote {a.out}  (ATE={rmse*100:.2f} cm, scale={s:.4f}, {m.sum()} assoc poses)")


if __name__ == "__main__":
    main()
