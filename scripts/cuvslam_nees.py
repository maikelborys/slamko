#!/usr/bin/env python3
"""Experiment 7-A analysis — is cuVSLAM's 6x6 covariance statistically calibrated?

Computes, from a recorded odom.csv (cuvslam_record_odom.py) + EuRoC GT (TUM):
  1. Healthy-tracking covariance-trace distribution (translation & rotation blocks)
     -> sets nominal_var_t and proves the cov is real/varying (no GT needed).
  2. RELATIVE-pose NEES over a fixed lag: the proper odometry consistency check.
     error e = log(rel_gt^-1 * rel_est) in se3 (6-vec), normalized by the relative
     covariance approx Sigma_rel ~= cov_i + cov_j (the same approximation slamko's
     edgeInformation flags as imperfect, acceptable for a first calibration read).
     Per-block NEES: trans (3-DOF) and rot (3-DOF). CALIBRATED <=> mean ~= 3.
     mean >> 3 = over-confident (cov too small); mean << 3 = under-confident.

Covariance order assumed ROS nav_msgs: [x,y,z,rx,ry,rz] (Isaac node already reordered).
SE3-aligns est->GT (scale reported separately; VIO should be ~1.0).

Usage: python3 cuvslam_nees.py --odom <dir>/odom.csv --gt <gt_tum.txt> [--lag 10]
"""
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R


def load_odom(path):
    rows = np.genfromtxt(path, delimiter=',', names=True)
    t = rows['t']
    xyz = np.column_stack([rows['x'], rows['y'], rows['z']])
    quat = np.column_stack([rows['qx'], rows['qy'], rows['qz'], rows['qw']])
    cov = np.column_stack([rows[f'cov{i}'] for i in range(36)]).reshape(-1, 6, 6)
    return t, xyz, quat, cov


def load_tum(path):
    d = np.loadtxt(path)
    return d[:, 0], d[:, 1:4], d[:, 4:8]  # t, xyz, quat(xyzw)


def assoc(t_a, t_b, max_dt=0.02):
    """nearest-neighbour timestamp association a->b."""
    idx = []
    j = 0
    for i, ta in enumerate(t_a):
        while j + 1 < len(t_b) and abs(t_b[j + 1] - ta) <= abs(t_b[j] - ta):
            j += 1
        if abs(t_b[j] - ta) <= max_dt:
            idx.append((i, j))
    return idx


def umeyama_se3(src, dst):
    """rigid (scale-free) align src->dst; also report best-fit scale for diagnostics."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    H = S.T @ D / len(src)
    U, sig, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    Rm = Vt.T @ np.diag([1, 1, d]) @ U.T
    var_s = (S ** 2).sum() / len(src)
    scale = (sig * [1, 1, d]).sum() / var_s
    t = mu_d - Rm @ mu_s
    return Rm, t, scale


def se3(Rm, p):
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = p
    return T


def logSE3(T):
    """se3 log -> 6-vec [trans; rot] (rot in tangent)."""
    Rm = T[:3, :3]
    rv = R.from_matrix(Rm).as_rotvec()
    return np.concatenate([T[:3, 3], rv])  # small-motion approx for trans part is fine for relative edges


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--odom', required=True)
    ap.add_argument('--gt', required=True)
    ap.add_argument('--lag', type=int, default=10, help='relative-pose lag in frames')
    ap.add_argument('--healthy-trace-pctl', type=float, default=90.0)
    a = ap.parse_args()

    te, pe, qe, cov = load_odom(a.odom)
    tg, pg, qg = load_tum(a.gt)
    pairs = assoc(te, tg)
    if len(pairs) < 50:
        print(f'!! only {len(pairs)} associations (est {len(te)} / gt {len(tg)}) — check time offset/units')
        return
    ie = np.array([p[0] for p in pairs])
    ig = np.array([p[1] for p in pairs])
    pe_a, qe_a, cov_a = pe[ie], qe[ie], cov[ie]
    pg_a, qg_a = pg[ig], qg[ig]

    Rm, tt, scale = umeyama_se3(pe_a, pg_a)
    print(f'associations: {len(pairs)}   SE3-align scale (diagnostic) = {scale:.4f}')

    # --- (1) covariance-trace distribution (no GT) ---
    tr_t = np.array([np.trace(c[:3, :3]) for c in cov_a])
    tr_r = np.array([np.trace(c[3:6, 3:6]) for c in cov_a])
    nonzero = np.count_nonzero(tr_t > 1e-12)
    print('\n===== (1) covariance trace distribution =====')
    print(f'non-zero trans-cov frames: {nonzero}/{len(tr_t)} '
          f'({"REAL/varying" if nonzero > 0.5 * len(tr_t) else "PLACEHOLDER?"})')
    for name, tr in [('trans (m^2)', tr_t), ('rot (rad^2)', tr_r)]:
        v = tr[tr > 1e-15]
        if len(v):
            print(f'  {name:14s} min={v.min():.2e} median={np.median(v):.2e} '
                  f'p{a.healthy_trace_pctl:.0f}={np.percentile(v, a.healthy_trace_pctl):.2e} max={v.max():.2e}')
    print(f'  -> suggested nominal_var_t ~= median trans-trace/3 = {np.median(tr_t[tr_t>1e-15])/3:.2e}'
          if (tr_t > 1e-15).any() else '  -> trans cov all zero')

    # --- (2) relative-pose NEES over fixed lag ---
    Ralign = Rm
    Te = [se3(R.from_quat(q).as_matrix(), p) for p, q in zip(pe_a, qe_a)]
    Tg = [se3(R.from_quat(q).as_matrix(), p) for p, q in zip(pg_a, qg_a)]
    lag = a.lag
    nees_t, nees_r = [], []
    for i in range(len(Te) - lag):
        j = i + lag
        rel_e = np.linalg.inv(Te[i]) @ Te[j]
        rel_g = np.linalg.inv(Tg[i]) @ Tg[j]
        err = logSE3(np.linalg.inv(rel_g) @ rel_e)  # 6-vec [t;r], frame i (rotation-invariant to global align)
        Sig = cov_a[i] + cov_a[j]
        St, Sr = Sig[:3, :3], Sig[3:6, 3:6]
        if np.linalg.cond(St) < 1e12 and np.linalg.cond(Sr) < 1e12:
            nees_t.append(err[:3] @ np.linalg.solve(St, err[:3]))
            nees_r.append(err[3:] @ np.linalg.solve(Sr, err[3:]))
    nees_t, nees_r = np.array(nees_t), np.array(nees_r)
    # robust over/under-confidence read, independent of NEES composition subtleties:
    # compare the ACTUAL relative-error RMS against the covariance-PREDICTED 1-sigma.
    et = np.array([logSE3(np.linalg.inv(np.linalg.inv(Tg[i]) @ Tg[i+lag])
                          @ (np.linalg.inv(Te[i]) @ Te[i+lag]))[:3] for i in range(len(Te)-lag)])
    er = np.array([logSE3(np.linalg.inv(np.linalg.inv(Tg[i]) @ Tg[i+lag])
                          @ (np.linalg.inv(Te[i]) @ Te[i+lag]))[3:] for i in range(len(Te)-lag)])
    rms_t = np.sqrt((et**2).sum(1).mean()); rms_r = np.sqrt((er**2).sum(1).mean())
    sig_t = np.sqrt(np.mean([np.trace((cov_a[i]+cov_a[i+lag])[:3,:3]) for i in range(len(Te)-lag)]))
    sig_r = np.sqrt(np.mean([np.trace((cov_a[i]+cov_a[i+lag])[3:6,3:6]) for i in range(len(Te)-lag)]))
    print(f'\n===== (2b) actual relative error vs predicted sigma (lag={lag}, robust) =====')
    print(f'  trans: actual RMS {rms_t*100:.2f} cm  vs predicted 1-sigma {sig_t*100:.2f} cm  '
          f'-> ratio {rms_t/max(1e-9,sig_t):.1f}x ({"OVER" if rms_t>sig_t else "UNDER"}-confident)')
    print(f'  rot  : actual RMS {np.degrees(rms_r):.2f} deg vs predicted 1-sigma {np.degrees(sig_r):.2f} deg '
          f'-> ratio {rms_r/max(1e-9,sig_r):.1f}x ({"OVER" if rms_r>sig_r else "UNDER"}-confident)')
    print(f'\n===== (2) relative-pose NEES (lag={lag} frames, {len(nees_t)} samples) =====')
    print('  block        mean   (calib=3.0)   median    verdict')
    for name, nv in [('trans', nees_t), ('rot  ', nees_r)]:
        if len(nv):
            m = nv.mean()
            verdict = ('OVER-confident (cov too small)' if m > 4.5
                       else 'UNDER-confident (cov too large)' if m < 1.5
                       else 'CALIBRATED-ish')
            print(f'  {name}    {m:8.2f}                {np.median(nv):8.2f}   {verdict}')
    print('\n  scale factor to calibrate (multiply that block cov by): '
          f'trans x{nees_t.mean()/3:.2f}, rot x{nees_r.mean()/3:.2f}' if len(nees_t) and len(nees_r) else '')


if __name__ == '__main__':
    main()
