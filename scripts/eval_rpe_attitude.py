#!/usr/bin/env python3
"""eval_rpe_attitude.py — the precise VIO error-localizer (P0-2).

Sim3 ATE hides *what kind* of drift you have. This decomposes it the way the
rpg_trajectory_evaluation methodology (Zhang & Scaramuzza, IROS 2018) prescribes:

  1. RELATIVE pose error (RPE) over fixed sub-trajectory lengths (1/5/10/25 m) —
     where drift ACCUMULATES. RPE flat across lengths => local solve is fine and
     the error is a constant offset; RPE growing ~linearly with length => an
     accumulating source (no marginalization prior / FEJ).
  2. Per-axis ATTITUDE error over time (roll/pitch vs yaw). Roll/pitch are
     gravity-observable: a CONSTANT non-zero offset from t=0 => init-gravity tilt;
     a near-zero mean => gravity is fine. Yaw is the weakly-observable DoF: a RAMP
     => gyro-bias / heading drift.
  3. Sim3 SCALE factor, reported separately (VI scale should be ~1.00).

Reads two TUM files (`ts tx ty tz qx qy qz qw`, '#' comments ok). No ROS needed.

usage:
  scripts/eval_rpe_attitude.py --est results/vio/MH_01_easy/est.tum \\
      --gt /mnt/data/datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data_tum.txt \\
      --out results/vio/MH_01_easy/rpe_attitude [--segments 1,5,10,25]
"""
import argparse
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot


def load_tum(path):
    ts, xyz, quat = [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = line.replace(",", " ").split()
            if len(v) < 8:
                continue
            ts.append(float(v[0]))
            xyz.append([float(v[1]), float(v[2]), float(v[3])])
            quat.append([float(v[4]), float(v[5]), float(v[6]), float(v[7])])  # x y z w
    return np.array(ts), np.array(xyz), np.array(quat)


def associate(ts_e, ts_g, max_dt=0.02):
    """Nearest-neighbour ts association est->gt. Returns matched index pairs."""
    pairs = []
    j = 0
    n = len(ts_g)
    for i, t in enumerate(ts_e):
        while j + 1 < n and abs(ts_g[j + 1] - t) <= abs(ts_g[j] - t):
            j += 1
        if abs(ts_g[j] - t) <= max_dt:
            pairs.append((i, j))
    return np.array(pairs, dtype=int)


def umeyama(src, dst, with_scale=True):
    """Least-squares src->dst similarity. Returns (R, t, s)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    Sc, Dc = src - mu_s, dst - mu_d
    cov = (Dc.T @ Sc) / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = (np.trace(np.diag(D) @ S) / (Sc ** 2).sum() * len(src)) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return R, t, s


def mats_from(xyz, quat):
    R = Rot.from_quat(quat).as_matrix()
    T = np.tile(np.eye(4), (len(xyz), 1, 1))
    T[:, :3, :3] = R
    T[:, :3, 3] = xyz
    return T


def rpe_over_length(T_e, T_g, L, stride=1):
    """rpg-style relative error over sub-trajectories of GT arc-length ~L (m).
    Returns (trans_drift_pct, rot_deg) arrays."""
    seg = np.linalg.norm(np.diff(T_g[:, :3, 3], axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    trans_pct, rot_deg = [], []
    n = len(T_g)
    j = 0
    for i in range(0, n - 1, stride):
        # advance j to first index >= L metres along GT from i
        while j < n and (cum[j] - cum[i]) < L:
            j += 1
        if j >= n:
            break
        d_g = np.linalg.inv(T_g[i]) @ T_g[j]
        d_e = np.linalg.inv(T_e[i]) @ T_e[j]
        E = np.linalg.inv(d_g) @ d_e
        trans_pct.append(np.linalg.norm(E[:3, 3]) / L * 100.0)
        ang = np.degrees(np.arccos(np.clip((np.trace(E[:3, :3]) - 1) / 2, -1, 1)))
        rot_deg.append(ang)
    return np.array(trans_pct), np.array(rot_deg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--est", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--out", required=True, help="output prefix")
    ap.add_argument("--segments", default="1,5,10,25")
    ap.add_argument("--max_dt", type=float, default=0.02)
    args = ap.parse_args()

    ts_e, xyz_e, q_e = load_tum(args.est)
    ts_g, xyz_g, q_g = load_tum(args.gt)
    if len(ts_e) < 10 or len(ts_g) < 10:
        sys.exit(f"too few poses: est={len(ts_e)} gt={len(ts_g)}")

    pairs = associate(ts_e, ts_g, args.max_dt)
    if len(pairs) < 10:
        sys.exit(f"only {len(pairs)} associations (max_dt={args.max_dt}s)")
    ie, ig = pairs[:, 0], pairs[:, 1]
    xyz_e, q_e, ts_e = xyz_e[ie], q_e[ie], ts_e[ie]
    xyz_g, q_g = xyz_g[ig], q_g[ig]
    t0 = ts_e[0]
    tsec = ts_e - t0

    # ---- Sim3 scale (reported) + SE3 alignment (for attitude/ATE) ----
    _, _, scale = umeyama(xyz_e, xyz_g, with_scale=True)
    R_al, t_al, _ = umeyama(xyz_e, xyz_g, with_scale=False)  # SE3, scale=1
    xyz_e_al = (R_al @ xyz_e.T).T + t_al
    ape = np.linalg.norm(xyz_e_al - xyz_g, axis=1)

    # ---- per-axis attitude error (after SE3 rotation alignment) ----
    Re = Rot.from_quat(q_e).as_matrix()
    Rg = Rot.from_quat(q_g).as_matrix()
    Re_al = R_al[None, :, :] @ Re
    R_err = np.transpose(Rg, (0, 2, 1)) @ Re_al   # gt^T * est
    eul = Rot.from_matrix(R_err).as_euler("xyz", degrees=True)  # roll,pitch,yaw

    # ---- RPE over segment lengths ----
    T_e = mats_from(xyz_e, q_e)
    T_g = mats_from(xyz_g, q_g)
    Ls = [float(x) for x in args.segments.split(",")]
    rpe = {L: rpe_over_length(T_e, T_g, L) for L in Ls}

    # ---- summary ----
    print(f"=== eval_rpe_attitude: {args.est} ===")
    print(f"  associated poses : {len(pairs)}  (span {tsec[-1]:.1f}s)")
    print(f"  Sim3 ATE RMSE    : {np.sqrt((ape**2).mean())*100:.2f} cm  (SE3-aligned here)")
    print(f"  Sim3 scale       : {scale:.4f}   (VI should be ~1.00)")
    print(f"  ATTITUDE error (deg, mean +/- std over time):")
    for k, nm in enumerate(["roll ", "pitch", "yaw  "]):
        m, s = eul[:, k].mean(), eul[:, k].std()
        # linear drift rate (deg/min) — the yaw ramp is the gyro-bias signature
        A = np.vstack([tsec, np.ones_like(tsec)]).T
        slope = np.linalg.lstsq(A, eul[:, k], rcond=None)[0][0] * 60.0
        print(f"    {nm}: mean {m:+6.2f}  std {s:5.2f}  drift {slope:+6.3f} deg/min")
    print(f"  RPE (rpg-style, relative):")
    for L in Ls:
        tp, rd = rpe[L]
        if len(tp):
            print(f"    {L:5.1f} m: trans {np.median(tp):5.2f}% (drift)   "
                  f"rot {np.median(rd):5.3f} deg   (n={len(tp)})")

    # ---- plots ----
    fig, ax = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle(f"VIO error decomposition — {args.est}", fontsize=12)

    # (0,0) attitude error over time
    ax[0, 0].plot(tsec, eul[:, 0], label="roll (grav-obs)", lw=0.8)
    ax[0, 0].plot(tsec, eul[:, 1], label="pitch (grav-obs)", lw=0.8)
    ax[0, 0].plot(tsec, eul[:, 2], label="yaw (drift DoF)", lw=1.0, color="crimson")
    ax[0, 0].axhline(0, color="k", lw=0.5)
    ax[0, 0].set_title("Per-axis attitude error vs time\n"
                       "(roll/pitch offset=grav tilt · yaw ramp=gyro-bias)")
    ax[0, 0].set_xlabel("t (s)"); ax[0, 0].set_ylabel("deg"); ax[0, 0].legend(fontsize=8)
    ax[0, 0].grid(alpha=0.3)

    # (0,1) RPE trans drift % vs segment length (box)
    data = [rpe[L][0] for L in Ls if len(rpe[L][0])]
    labs = [f"{L:g}m" for L in Ls if len(rpe[L][0])]
    if data:
        ax[0, 1].boxplot(data, labels=labs, showfliers=False)
    ax[0, 1].set_title("RPE translation drift vs sub-traj length\n"
                       "(flat=offset · rising=accumulating drift)")
    ax[0, 1].set_ylabel("% of segment length"); ax[0, 1].grid(alpha=0.3)

    # (1,0) ATE over time
    ax[1, 0].plot(tsec, ape * 100, lw=0.8, color="darkorange")
    ax[1, 0].set_title(f"SE3-aligned position error vs time "
                       f"(scale={scale:.4f})")
    ax[1, 0].set_xlabel("t (s)"); ax[1, 0].set_ylabel("cm"); ax[1, 0].grid(alpha=0.3)

    # (1,1) trajectory top-down
    ax[1, 1].plot(xyz_g[:, 0], xyz_g[:, 1], label="GT", lw=1.0, color="k")
    ax[1, 1].plot(xyz_e_al[:, 0], xyz_e_al[:, 1], label="est (SE3-aln)", lw=0.8, color="tab:blue")
    ax[1, 1].set_title("Trajectory (top-down)"); ax[1, 1].axis("equal")
    ax[1, 1].legend(fontsize=8); ax[1, 1].grid(alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = args.out + ".png"
    fig.savefig(out, dpi=110)
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
