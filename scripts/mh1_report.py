#!/usr/bin/env python3
"""MH_01 blackout/localization campaign report: a stats table across all runs + an
interactive Plotly overlay of every run's fused trajectory (Umeyama-aligned to GT) on the
ground truth, so the blackout deviations are visible.

  mh1_report.py            # reads results/mh1/<run>/ + the EuRoC MH_01 GT
"""
import os, re, glob, subprocess
import numpy as np

SEQ = "MH_01_easy"
GT = f"/mnt/data/datasets/euroc/{SEQ}/mav0/state_groundtruth_estimate0/data.csv"
RUNS = [("normal", "—"), ("bk_early", "40–44 s"), ("bk_mid", "90–94 s"),
        ("bk_long", "90–100 s (10s)"), ("localize", "— (prior)")]
ATE = os.path.expanduser("~/coding/RTABmap/euroc_ate.py")


def load_tum(p):
    if not os.path.exists(p):
        return None
    a = np.loadtxt(p)
    return a if a.ndim == 2 and len(a) else None


def load_gt():
    d = np.genfromtxt(GT, delimiter=",")
    return np.column_stack([d[:, 0] / 1e9, d[:, 1:4]])  # t[s], x, y, z


def ate_rmse(tum):
    if not os.path.exists(tum):
        return None
    try:
        out = subprocess.run(["python3", ATE, tum, SEQ], capture_output=True, text=True).stdout
        m = re.search(r"rmse=\s*([\d.]+)\s*cm", out)
        return float(m.group(1)) if m else None
    except Exception:
        return None


def umeyama(X, Y):  # align source X (Nx3) onto target Y (Nx3), with scale
    mx, my = X.mean(0), Y.mean(0)
    Xc, Yc = X - mx, Y - my
    S = (Yc.T @ Xc) / len(X)
    U, D, Vt = np.linalg.svd(S)
    d = np.ones(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        d[-1] = -1
    R = U @ np.diag(d) @ Vt
    c = (D * d).sum() / (Xc ** 2).sum() * len(X)
    return (c * (X @ R.T) + (my - c * (R @ mx))), R, c


def aligned_to_gt(tum, gt):
    a = load_tum(tum)
    if a is None:
        return None
    ts, xyz = a[:, 0], a[:, 1:4]
    gts, gxyz = gt[:, 0], gt[:, 1:4]
    # match each fused sample to the nearest GT timestamp within 20 ms
    idx = np.searchsorted(gts, ts)
    idx = np.clip(idx, 1, len(gts) - 1)
    pick = np.where(np.abs(gts[idx] - ts) < np.abs(gts[idx - 1] - ts), idx, idx - 1)
    ok = np.abs(gts[pick] - ts) < 0.02
    if ok.sum() < 10:
        return None
    Xa, _, _ = umeyama(xyz[ok], gxyz[pick[ok]])
    return Xa


def count(log, pat):
    if not os.path.exists(log):
        return 0
    return sum(1 for _ in open(log, errors="ignore") if re.search(pat, _))


gt = load_gt()
rows = []
print(f"\n{'run':<10} {'blackout':<16} {'ATE fused':>10} {'ATE graph':>10} "
      f"{'submaps':>8} {'comps':>6} {'reloc':>6} {'loops':>6} {'breaks':>7}")
print("-" * 92)
for name, bk in RUNS:
    d = f"results/mh1/{name}"
    log = f"{d}/launch.log"
    af = ate_rmse(f"{d}/fused.tum")
    ag = ate_rmse(f"{d}/graph.tum")
    nsub = len(open(f"{d}/map/submaps.manifest").readlines()) if os.path.exists(f"{d}/map/submaps.manifest") else 0
    comps = 0
    if os.path.exists(f"{d}/map/components.csv"):
        comps = len(set(l.split(",")[1] for l in open(f"{d}/map/components.csv").readlines()[1:] if "," in l))
    reloc = count(log, r"LOCALIZED in prior")
    loops = count(log, r"LOOP CLOSED|loop closed")
    breaks = count(log, r"ATLAS BREAK")
    rows.append((name, bk))
    fa = f"{af:.2f}cm" if af else "—"
    ga = f"{ag:.2f}cm" if ag else "—"
    print(f"{name:<10} {bk:<16} {fa:>10} {ga:>10} {nsub:>8} {comps:>6} "
          f"{reloc:>6} {loops:>6} {breaks:>7}")
print("-" * 92)

# ---- Plotly overlay: GT + every run's fused trajectory aligned to GT ----
try:
    import plotly.graph_objects as go
    fig = go.Figure()
    fig.add_trace(go.Scatter3d(x=gt[:, 1], y=gt[:, 2], z=gt[:, 3], mode="lines",
                  line=dict(color="black", width=5), name="ground truth"))
    cols = ["#2ca02c", "#ff7f0e", "#d62728", "#9467bd", "#1f77b4"]
    for (name, bk), c in zip(RUNS, cols):
        Xa = aligned_to_gt(f"results/mh1/{name}/fused.tum", gt)
        if Xa is None:
            continue
        fig.add_trace(go.Scatter3d(x=Xa[:, 0], y=Xa[:, 1], z=Xa[:, 2], mode="lines",
                      line=dict(color=c, width=3), name=f"{name} ({bk})"))
    fig.update_layout(title="MH_01 campaign — fused trajectories (Umeyama-aligned) vs ground truth",
                      scene=dict(aspectmode="data"))
    out = "results/mh1/mh1_trajectories.html"
    fig.write_html(out)
    print(f"\nwrote {out}  (interactive — GT black, runs coloured; blackout deviation visible)")
except Exception as e:
    print(f"\nplotly skipped: {e}")
