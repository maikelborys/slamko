#!/usr/bin/env python3
"""Certainty map (the "show only what's certain" view, user request 2026-06-19).
Colors the trajectory by whether it is VERIFIED against the prior/original map:
  GREEN  = a cross-session match landed near here (LOCALIZED/RE-ANCHORED/X-SESSION
           prior) -> the pose is tied to the original, the landmarks there cohere.
  RED    = odom-only, no verification -> dead-reckoned/drifted, do NOT trust its
           landmark placement (the incoherent-drawing problem the soft chain causes).
Prior map drawn gray underneath (the certain original). This both answers "draw the
map with only the certain skeleton" AND exposes recall-dead drift zones (red gaps
with no green = where proximity detection / item E is needed).

Usage: plot_certainty.py --run-dir results/run/x --prior-dir results/run/suave/map
       --out results/run/x/certainty.png [--win 18]"""
import argparse, os, re, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_neverlost import load_smap
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("--run-dir", required=True)
ap.add_argument("--prior-dir", default="")
ap.add_argument("--out", required=True)
ap.add_argument("--win", type=int, default=18, help="KF half-window counted as certain around a match")
ap.add_argument("--traj", default="graph", choices=["graph", "fused"])
a = ap.parse_args()

def L(p):
    return np.array([list(map(float, l.split())) for l in open(p)
                     if l.strip() and not l.startswith("#")])
g = L(os.path.join(a.run_dir, f"{a.traj}.tum"))
N = len(g)

# certain KFs = where a cross-session match was applied (row index ~ kf id, graph.tum
# is sorted by id from ~0). Parse the node log.
cert = set()
log = os.path.join(a.run_dir, "launch.log")
if os.path.exists(log):
    for l in open(log, errors="ignore"):
        m = re.search(r"(X-SESSION prior #\d+|RE-ANCHORED|LOCALIZED).*kf (\d+)", l)
        if m:
            cert.add(int(m.group(2)))
is_cert = np.zeros(N, bool)
for c in sorted(cert):
    is_cert[max(0, c - a.win):min(N, c + a.win)] = True

fig, ax = plt.subplots(figsize=(8, 11), dpi=115)
if a.prior_dir and os.path.exists(os.path.join(a.prior_dir, "submaps.manifest")):
    pids = [int(x) for x in open(os.path.join(a.prior_dir, "submaps.manifest"))]
    pr = np.concatenate([load_smap(os.path.join(a.prior_dir, f"submap_{i}.smap")) for i in pids])
    ax.scatter(pr[::8, 0], pr[::8, 1], s=0.4, c="#bbbbbb", alpha=0.35,
               label="prior map (original / certain)")
ax.scatter(g[is_cert][:, 1], g[is_cert][:, 2], s=8, c="#1a9e1a", zorder=4,
           label=f"CERTAIN (cross-session verified, {is_cert.sum()} kf)")
ax.scatter(g[~is_cert][:, 1], g[~is_cert][:, 2], s=8, c="#cc3333", zorder=3,
           label=f"UNCERTAIN (odom-only / drift, {(~is_cert).sum()} kf)")
ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(loc="upper right", fontsize=8)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title(f"{os.path.basename(a.run_dir)} — CERTAIN vs UNCERTAIN\n"
             f"green=verified-against-original  red=odom-only drift  "
             f"({100*is_cert.mean():.0f}% certain)")
fig.tight_layout(); fig.savefig(a.out)
print(f"wrote {a.out}  ({100*is_cert.mean():.0f}% certain, {len(cert)} match KFs)")
