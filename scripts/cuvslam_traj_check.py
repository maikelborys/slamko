#!/usr/bin/env python3
"""Validate a cuVSLAM trajectory ALONE (no GT) for COHERENCE before slamko trusts it
(Hard Rule #5: un-aligned divergence / pose-jump detection, not Sim3-aligned ATE).
From odom.csv (cuvslam_record_odom.py): writes odom.tum + a top-down/side PNG and prints
  - path length, start-end gap (loop closure if the bag returns to start)
  - per-sample implied speed + the BIGGEST inter-sample JUMPS (the cuVSLAM pose-jump worry)
  - covariance-trace range, any nullopt gaps (time discontinuities)
Usage: cuvslam_traj_check.py --odom <dir>/odom.csv --out-prefix <dir>/cuvslam
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("--odom", required=True)
ap.add_argument("--out-prefix", required=True)
ap.add_argument("--jump-speed", type=float, default=4.0, help="m/s implausible for handheld")
a = ap.parse_args()

d = np.genfromtxt(a.odom, delimiter=",", names=True)
t = d["t"]; xyz = np.column_stack([d["x"], d["y"], d["z"]])
cov_t = np.array([d[f"cov{i}"] for i in (0, 7, 14)]).T.sum(1)  # trans-trace per sample

# TUM dump
with open(a.out_prefix + ".tum", "w") as f:
    for i in range(len(t)):
        f.write(f"{t[i]:.9f} {d['x'][i]:.6f} {d['y'][i]:.6f} {d['z'][i]:.6f} "
                f"{d['qx'][i]:.6f} {d['qy'][i]:.6f} {d['qz'][i]:.6f} {d['qw'][i]:.6f}\n")

dt = np.diff(t)
step = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
speed = step / np.maximum(dt, 1e-3)
path_len = step.sum()
start_end = np.linalg.norm(xyz[-1] - xyz[0])
gaps = np.where(dt > 0.2)[0]                  # time discontinuities (>0.2s = dropped/lost)
jumps = np.argsort(speed)[::-1][:5]           # 5 biggest speed spikes

print("===== cuVSLAM trajectory coherence (no GT) =====")
print(f"samples {len(t)} | duration {t[-1]-t[0]:.1f}s | path length {path_len:.2f} m")
print(f"start->end gap {start_end:.3f} m  ({'LOOP closes' if start_end < 0.6 else 'open-ended / drift'})")
print(f"speed: median {np.median(speed):.2f}  p99 {np.percentile(speed,99):.2f}  max {speed.max():.2f} m/s")
n_jump = int((speed > a.jump_speed).sum())
print(f"POSE-JUMPS (>{a.jump_speed} m/s implausible): {n_jump} samples"
      + ("  <-- INVESTIGATE" if n_jump else "  (none — clean)"))
for j in jumps[:3]:
    print(f"   t+{t[j]-t[0]:6.1f}s  step {step[j]*100:6.1f} cm in {dt[j]*1000:.0f} ms = {speed[j]:.1f} m/s")
print(f"time gaps >0.2s (lost/nullopt): {len(gaps)}")
print(f"cov trans-trace: min {cov_t.min():.2e}  median {np.median(cov_t):.2e}  max {cov_t.max():.2e}")

fig, (axT, axS) = plt.subplots(1, 2, figsize=(16, 7))
sc = axT.scatter(xyz[:, 0], xyz[:, 1], c=t - t[0], s=6, cmap="viridis")
axT.scatter([xyz[0, 0]], [xyz[0, 1]], c="lime", s=120, marker="*", label="start", zorder=5)
axT.scatter([xyz[-1, 0]], [xyz[-1, 1]], c="red", s=120, marker="X", label="end", zorder=5)
if n_jump:
    jm = speed > a.jump_speed
    axT.scatter(xyz[1:][jm, 0], xyz[1:][jm, 1], facecolors="none", edgecolors="red", s=80, label="pose-jump")
axT.set_title("TOP-DOWN (X-Y)  color=time"); axT.set_aspect("equal", "datalim")
axT.legend(); axT.grid(alpha=0.3); axT.set_xlabel("X (m)"); axT.set_ylabel("Y (m)")
axS.scatter(xyz[:, 0], xyz[:, 2], c=t - t[0], s=6, cmap="viridis")
axS.set_title("SIDE (X-Z, height)"); axS.set_aspect("equal", "datalim")
axS.grid(alpha=0.3); axS.set_xlabel("X (m)"); axS.set_ylabel("Z (m)")
fig.colorbar(sc, ax=axS, label="t (s)")
fig.suptitle(f"cuVSLAM alone — {path_len:.1f}m path, start-end {start_end:.2f}m, {n_jump} pose-jumps")
fig.tight_layout(); fig.savefig(a.out_prefix + "_traj.png", dpi=110)
print(f"wrote {a.out_prefix}.tum + {a.out_prefix}_traj.png")
