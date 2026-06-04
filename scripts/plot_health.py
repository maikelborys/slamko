#!/usr/bin/env python3
"""plot_health.py — the per-frame VIO health trace overlay (P0-1).

Reads the instrumented timing.csv (reproj_rms, inlier_ratio, pnp_ok, dr_active,
n_imu_interval, interval_dt, max_imu_gap, ba_init/final_cost, ba_iters,
ba_converged) and, if est.tum + GT are given, the per-pose position error.
Stacks every health signal against the ATE error curve on a shared frame axis so
an error spike is CO-LOCATED with its cause — outlier contamination (high
inlier_ratio + high reproj_rms), feature starvation (low n_3d_prev/inliers),
silent freeze (pnp_ok=0 with dr_active=0), corrupt IMU window (n_imu << expected),
or a non-converged BA (ba_converged=0 / hit max iters).

CSV rows are per processed frame; est.tum poses are per published frame — aligned
by order (truncated to the shorter; a mismatch is reported, not hidden).

usage:
  scripts/plot_health.py --csv results/vio/MH_01_easy/timing.csv \\
      [--est results/vio/MH_01_easy/est.tum \\
       --gt  /mnt/data/.../state_groundtruth_estimate0/data_tum.txt] \\
      --out results/vio/MH_01_easy/health
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
            xyz.append([float(x) for x in v[1:4]])
            quat.append([float(x) for x in v[4:8]])
    return np.array(ts), np.array(xyz), np.array(quat)


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    Sc, Dc = src - mu_s, dst - mu_d
    cov = (Dc.T @ Sc) / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / (Sc ** 2).sum() * len(src)
    t = mu_d - s * R @ mu_s
    return R, t, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--est", default=None)
    ap.add_argument("--gt", default=None)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    d = np.genfromtxt(args.csv, delimiter=",", names=True)
    if d.size == 0:
        sys.exit("empty CSV")
    frame = d["frame"]
    have = lambda k: k in d.dtype.names
    nan_neg = lambda a: np.where(a < 0, np.nan, a)  # -1 sentinel -> NaN gap

    # ---- per-frame ATE (optional) ----
    ape = None
    if args.est and args.gt:
        ts_e, xyz_e, q_e = load_tum(args.est)
        ts_g, xyz_g, q_g = load_tum(args.gt)
        # associate est->gt by nearest ts, then Sim3-align
        j, pairs = 0, []
        for i, t in enumerate(ts_e):
            while j + 1 < len(ts_g) and abs(ts_g[j+1]-t) <= abs(ts_g[j]-t):
                j += 1
            if abs(ts_g[j]-t) <= 0.02:
                pairs.append((i, j))
        pairs = np.array(pairs, dtype=int)
        if len(pairs) > 10:
            R, t, s = umeyama(xyz_e[pairs[:,0]], xyz_g[pairs[:,1]])
            al = (s * (R @ xyz_e[pairs[:,0]].T).T + t)
            err = np.linalg.norm(al - xyz_g[pairs[:,1]], axis=1) * 100  # cm
            # map est-pose index -> csv row by order (both per-frame, in sequence)
            n = min(len(err), len(frame))
            if abs(len(err) - len(frame)) > 0.05 * len(frame):
                print(f"  NOTE est poses ({len(err)}) vs csv rows ({len(frame)}) "
                      f"differ >5% — index alignment approximate")
            ape = np.full(len(frame), np.nan)
            ape[:n] = err[:n]

    # ---- summary stats (printed) ----
    print(f"=== plot_health: {args.csv} ({len(frame)} frames) ===")
    if have("pnp_ok"):
        froze = np.sum((d["pnp_ok"] == 0) & (d["dr_active"] == 0))
        print(f"  pnp_ok=0           : {int(np.sum(d['pnp_ok']==0))} frames "
              f"({100*np.mean(d['pnp_ok']==0):.1f}%)")
        print(f"  SILENT FREEZE (pnp=0 & dr=0): {int(froze)} frames"
              + ("   <-- silent-freeze defect (rank 3)" if froze else ""))
        print(f"  dead-reckoning     : {int(np.sum(d['dr_active']==1))} frames")
    if have("inlier_ratio"):
        ir = nan_neg(d["inlier_ratio"]); print(f"  inlier_ratio median: {np.nanmedian(ir):.3f}")
    if have("reproj_rms"):
        rr = nan_neg(d["reproj_rms"]); print(f"  reproj_rms median  : {np.nanmedian(rr):.3f} px")
    if have("ba_converged"):
        solved = d["ba_solved"] == 1
        nconv = np.sum(solved & (d["ba_converged"] == 0))
        print(f"  BA solves          : {int(np.sum(solved))}   "
              f"NOT converged: {int(nconv)} ({100*nconv/max(1,np.sum(solved)):.1f}%)")
        if have("ba_fail"):
            # only count rows where a KF insert happened (BA was attempted)
            att = d["n_imu_interval"] >= 0 if have("n_imu_interval") else solved
            f1 = int(np.sum((d["ba_fail"] == 1)))  # window<2 (rebuilt/empty)
            f2 = int(np.sum((d["ba_fail"] == 2)))  # no landmarks post-prune
            print(f"  BA bail reasons    : window<2={f1}  no-landmarks={f2}   "
                  f"(of ~{int(np.sum(att))} KF attempts) "
                  f"<-- VI-BA dropout localizer")
    if have("n_imu_interval"):
        ni = d["n_imu_interval"]; dt = d["interval_dt"]
        kf = ni >= 0
        if np.any(kf):
            exp = dt[kf] * 200.0  # nominal EuRoC 200 Hz
            starved = np.sum(ni[kf] < 0.5 * exp)
            print(f"  KF IMU windows     : {int(np.sum(kf))}   "
                  f"STARVED (<50% expected): {int(starved)}")

    # ---- stacked plot ----
    panels = []
    panels.append(("ATE error (cm)", "ate"))
    panels.append(("reproj_rms (px) & inlier_ratio", "track"))
    panels.append(("feature counts", "feat"))
    panels.append(("BA cost (log) & iters", "ba"))
    panels.append(("IMU window integrity", "imu"))
    panels.append(("flags: pnp_ok / dr_active", "flags"))
    npan = len(panels)
    fig, axs = plt.subplots(npan, 1, figsize=(15, 2.1 * npan), sharex=True)
    fig.suptitle(f"VIO per-frame health trace — {args.csv}", fontsize=12)

    for ax, (title, key) in zip(axs, panels):
        if key == "ate":
            if ape is not None:
                ax.plot(frame, ape, lw=0.7, color="darkorange")
                ax.set_ylabel("cm")
            else:
                ax.text(0.5, 0.5, "no --est/--gt: ATE overlay skipped",
                        ha="center", transform=ax.transAxes, color="gray")
        elif key == "track":
            if have("reproj_rms"):
                ax.plot(frame, nan_neg(d["reproj_rms"]), lw=0.6, color="tab:red", label="reproj_rms px")
            if have("inlier_ratio"):
                ax2 = ax.twinx()
                ax2.plot(frame, nan_neg(d["inlier_ratio"]), lw=0.6, color="tab:green", label="inlier_ratio")
                ax2.set_ylabel("ratio", color="tab:green"); ax2.set_ylim(0, 1.05)
            ax.set_ylabel("px", color="tab:red")
        elif key == "feat":
            for c, col in [("n_3d_prev", "tab:blue"), ("n_pnp_inliers", "tab:cyan"),
                           ("n_active", "tab:gray")]:
                if have(c):
                    ax.plot(frame, d[c], lw=0.6, label=c, color=col)
            ax.axhline(12, color="crimson", lw=0.6, ls="--", label="min_inliers=12 cliff")
            ax.legend(fontsize=7, ncol=4); ax.set_ylabel("count")
        elif key == "ba":
            if have("ba_final_cost"):
                ax.plot(frame, nan_neg(d["ba_init_cost"]), lw=0.5, color="plum", label="ba_init_cost")
                ax.plot(frame, nan_neg(d["ba_final_cost"]), lw=0.6, color="purple", label="ba_final_cost")
                ax.set_yscale("log"); ax.set_ylabel("cost"); ax.legend(fontsize=7)
                if have("ba_converged"):
                    nc = (d["ba_solved"] == 1) & (d["ba_converged"] == 0)
                    if np.any(nc):
                        ax.plot(frame[nc], nan_neg(d["ba_final_cost"])[nc], "x",
                                color="red", ms=4, label="NOT converged")
                        ax.legend(fontsize=7)
        elif key == "imu":
            if have("n_imu_interval"):
                m = d["n_imu_interval"] >= 0
                ax.plot(frame[m], d["n_imu_interval"][m], ".", ms=2, color="teal", label="n_imu/KF")
                if have("interval_dt"):
                    ax.plot(frame[m], d["interval_dt"][m] * 200.0, lw=0.5, color="orange",
                            label="expected (~200Hz)")
                ax.legend(fontsize=7); ax.set_ylabel("samples")
        elif key == "flags":
            if have("pnp_ok"):
                ax.fill_between(frame, 0, (d["pnp_ok"] == 0).astype(float),
                                step="mid", color="red", alpha=0.5, label="PnP FAIL")
            if have("dr_active"):
                ax.fill_between(frame, 0, -(d["dr_active"] == 1).astype(float),
                                step="mid", color="navy", alpha=0.5, label="dead-reckoning")
            ax.set_ylim(-1.1, 1.1); ax.legend(fontsize=7); ax.set_ylabel("flag")
        ax.set_title(title, fontsize=9, loc="left"); ax.grid(alpha=0.25)

    axs[-1].set_xlabel("frame")
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    out = args.out + ".png"
    fig.savefig(out, dpi=110)
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
