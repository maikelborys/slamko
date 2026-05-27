#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Offline viz: slamko estimated trajectory vs EuRoC ground truth + the landmark
# map. The estimate is Sim3-aligned to GT (Umeyama, timestamp-associated), and
# the SAME transform is applied to the landmarks so everything overlays in the GT
# frame. Renders top-down (X-Y) + side (X-Z) panels to a PNG. No ROS.
#
# Never-lost CORRECTED-MAP mode (--submaps / --pose-epoch): the raw dumps are in
# the continuous VIO odom frame, so dead-reckoning drift across a tracking blackout
# leaves the post-blackout submap shifted/rotated. The never-lost WELD measures that
# drift as each submap's anchor (map = anchor·odom). Given the per-submap sidecars
# the node writes, we move each submap's landmarks + each pose into the corrected MAP
# frame BEFORE the Sim3 fit — so the plot shows the merged map, not the drifted one.
#
# usage: plot_neverlost.py --gt GT.tum --est est.tum --landmarks lm.csv --out fig.png
#          [--loss s1 e1 s2 e2 ...] [--submaps lm.csv.submaps] [--pose-epoch est.tum.epoch]
import argparse
import os
import struct
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
    return d[:, 0].astype(np.int64), d[:, 1:4], d[:, 4]  # id, xyz, obs


def load_submaps(path):
    """Per-submap id range + welded anchor (3×4 row-major). Returns list of
    (id, id_lo, id_hi, R[3×3], t[3])."""
    r = np.loadtxt(path, delimiter=",", skiprows=1)
    if r.ndim == 1:
        r = r[None, :]
    out = []
    for row in r:
        A = row[3:15].reshape(3, 4)
        out.append((int(row[0]), int(row[1]), int(row[2]), A[:, :3].copy(), A[:, 3].copy()))
    return out


def load_epoch(path):
    d = np.loadtxt(path)            # ts active_submap_id
    if d.ndim == 1:
        d = d[None, :]
    return d[:, 0], d[:, 1].astype(np.int64)


def _quat_to_R(x, y, z, w):
    n = (x * x + y * y + z * z + w * w) ** 0.5
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_smap(path):
    """Read a .smap (submap_io.hpp "SMP1" binary): return the landmark positions
    already moved to the MAP frame (anchor·local). Keyframes + descriptors skipped."""
    with open(path, "rb") as fh:
        d = fh.read()
    if d[:4] != b"SMP1":
        raise SystemExit(f"{path}: bad magic {d[:4]!r}")
    off = 4 + 8                                   # magic + id(u64)
    q = struct.unpack_from("<7d", d, off); off += 56   # anchor quat(xyzw)+t
    R = _quat_to_R(q[0], q[1], q[2], q[3]); t = np.array(q[4:7])
    (nk,) = struct.unpack_from("<Q", d, off); off += 8
    off += nk * 72                                # skip keyframes (id+ts+pose)
    (nl,) = struct.unpack_from("<Q", d, off); off += 8
    lm = np.frombuffer(d, offset=off,             # id u64, xyz 3d, dr i32 = 36 B packed
                       dtype=np.dtype([("id", "<u8"), ("x", "<f8"), ("y", "<f8"),
                                       ("z", "<f8"), ("dr", "<i4")]), count=nl)
    xyz = np.stack([lm["x"], lm["y"], lm["z"]], axis=1)
    return (R @ xyz.T).T + t                       # → map frame


def load_prior_map(dirpath, max_pts=80000):
    """All prior-submap landmarks (map frame), read via the manifest. Subsampled."""
    ids = [int(x) for x in open(os.path.join(dirpath, "submaps.manifest"))]
    chunks = [load_smap(os.path.join(dirpath, f"submap_{i}.smap")) for i in ids]
    pts = np.concatenate(chunks, axis=0) if chunks else np.zeros((0, 3))
    if len(pts) > max_pts:
        pts = pts[:: (len(pts) + max_pts - 1) // max_pts]
    return pts


def correct_landmarks(ids, xyz, submaps):
    """map = anchor·odom, per landmark, by which submap's id range owns it."""
    out = xyz.copy()
    for _, lo, hi, R, t in submaps:
        msk = (ids >= lo) & (ids <= hi)
        if msk.any():
            out[msk] = (R @ xyz[msk].T).T + t
    return out


def correct_poses(ts, xyz, ets, esid, submaps):
    """Move each pose into the corrected map frame via its active submap's anchor."""
    anchor = {sid: (R, t) for sid, _, _, R, t in submaps}
    out = xyz.copy()
    j = np.clip(np.searchsorted(ets, ts), 0, len(ets) - 1)
    for i in range(len(ts)):
        a = anchor.get(int(esid[j[i]]))
        if a is not None:
            out[i] = a[0] @ xyz[i] + a[1]
    return out


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
    ap.add_argument("--submaps", default="", help="<lm>.submaps: per-submap id range + welded anchor")
    ap.add_argument("--pose-epoch", default="", help="<pose>.epoch: per-frame active submap id")
    ap.add_argument("--prior-map-dir", default="",
                    help="prior session's saved Atlas dir — overlay its map (cross-session merge)")
    ap.add_argument("--title", default="slamko never-lost")
    ap.add_argument("--dpi", type=int, default=130, help="output PNG dpi (lower = smaller file)")
    a = ap.parse_args()

    tg, xg = load_tum(a.gt)
    te, xe = load_tum(a.est)

    # Never-lost correction: move poses + landmarks into the merged MAP frame using
    # the welded per-submap anchors, BEFORE the Sim3 fit to GT.
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
    s, R, t = umeyama(xe[m], xg[idx[m]])           # est → GT frame
    xe_a = (s * (R @ xe.T).T) + t                  # aligned estimate
    rmse = float(np.sqrt(((xe_a[m] - xg[idx[m]]) ** 2).sum(1).mean()))
    lm_a = (s * (R @ lm_xyz.T).T) + t if lm_xyz is not None else None

    # Cross-session: the prior map (already in the shared frame via its anchors) under
    # the SAME Sim3 → it overlays the live map iff the cross-session weld was correct.
    prior_a = None
    if a.prior_map_dir:
        prior_xyz = load_prior_map(a.prior_map_dir)
        prior_a = (s * (R @ prior_xyz.T).T) + t

    fig, ax = plt.subplots(1, 2, figsize=(15, 7))
    panels = [(0, 1, "X (m)", "Y (m)", "top-down"), (0, 2, "X (m)", "Z (m)", "side")]
    for k, (i, j, xl, yl, name) in enumerate(panels):
        if prior_a is not None:
            ax[k].scatter(prior_a[:, i], prior_a[:, j], s=1.0, c="tab:orange", alpha=0.30,
                          label=f"prior map ({len(prior_a)})", rasterized=True)
        if lm_a is not None:
            ax[k].scatter(lm_a[:, i], lm_a[:, j], s=1.0, c="0.5", alpha=0.40,
                          label=f"live map ({lm_n})" if prior_a is not None
                          else f"landmarks ({lm_n})", rasterized=True)
        ax[k].plot(xg[:, i], xg[:, j], "-", c="k", lw=2.0, label="ground truth")
        ax[k].plot(xe_a[:, i], xe_a[:, j], "-", c="tab:blue", lw=1.3,
                   label="estimate (Sim3-aligned)")
        if a.loss is not None:
            tr = te - te[0]  # --loss is relative-to-start (est stamps are absolute)
            for w, (s0, s1) in enumerate(zip(a.loss[0::2], a.loss[1::2])):
                lm_mask = (tr >= s0) & (tr <= s1)
                if lm_mask.any():
                    ax[k].plot(xe_a[lm_mask, i], xe_a[lm_mask, j], "-", c="tab:red", lw=3.0,
                               label="forced-loss (dead-reckon)" if w == 0 else None)
        ax[k].plot(xg[0, i], xg[0, j], "o", c="g", ms=9, label="start")
        ax[k].set_xlabel(xl); ax[k].set_ylabel(yl); ax[k].set_title(name)
        ax[k].set_aspect("equal")
        # Zoom to the trajectory extent (+margin) so noisy far landmarks don't blow up
        # the scale and hide the map overlap.
        mg = 2.0
        ax[k].set_xlim(xg[:, i].min() - mg, xg[:, i].max() + mg)
        ax[k].set_ylim(xg[:, j].min() - mg, xg[:, j].max() + mg)
        ax[k].grid(alpha=0.3)
    ax[0].legend(loc="best", fontsize=9)
    tag = f"  |  {len(submaps)} submaps (anchor-corrected)" if submaps else ""
    fig.suptitle(f"{a.title}   |   Sim3-ATE {rmse*100:.1f} cm   |   {m.sum()} poses{tag}",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(a.out, dpi=a.dpi)
    print(f"wrote {a.out}  (ATE={rmse*100:.2f} cm, scale={s:.4f}, {m.sum()} assoc poses"
          f"{', ' + str(len(submaps)) + ' submaps' if submaps else ''})")


if __name__ == "__main__":
    main()
