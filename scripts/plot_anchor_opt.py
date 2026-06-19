#!/usr/bin/env python3
# Anchor-graph optimization demo (PLAN_ROBUSTNESS_01 R1.1): the submap anchors are
# nodes, the inter-map edges (chain-odom / SOFT dead-reckoning / HARD verified weld)
# are covariance-weighted constraints. Optimizing the graph CLOSES a loop and
# distributes the correction by covariance: the HARD edge pins, the SOFT edges
# flex (high sigma -> they absorb most of the drift), the ODOM edges stay rigid.
#
# To make the effect visible on a run whose keyframe graph already closed the loop,
# --drift injects a synthetic open-loop (accumulated chain drift) first; then the
# anchor-graph optimization closes it. Pure geometry of the edges; SE3 least-squares.
#
#   python3 scripts/plot_anchor_opt.py --edges results/r0/blackout_edges2/map/anchor_edges.csv \
#       --drift 1.2 --out results/r0/anchor_opt.png
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def se3(t, q):
    T = np.eye(4)
    T[:3, :3] = R.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


def load_edges(path):
    edges = []
    import csv
    with open(path) as fh:
        for r in csv.DictReader(fh):
            T = se3([float(r["tx"]), float(r["ty"]), float(r["tz"])],
                    [float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"])])
            edges.append(dict(f=int(r["from"]), t=int(r["to"]), ty=int(r["type"]),
                              T=T, st=float(r["sigma_t"]), sr=float(r["sigma_r"])))
    return edges


def chain_init(nodes, edges):
    T = {nodes[0]: np.eye(4)}
    for _ in range(len(nodes)):
        for e in edges:
            if e["f"] in T and e["t"] not in T:
                T[e["t"]] = T[e["f"]] @ e["T"]
            elif e["t"] in T and e["f"] not in T:
                T[e["f"]] = T[e["t"]] @ np.linalg.inv(e["T"])
    return T


def optimize(nodes, edges, init):
    free = nodes[1:]
    idx = {n: i for i, n in enumerate(free)}

    def pose(x, n):
        if n == nodes[0]:
            return np.eye(4)
        p = x[6 * idx[n]:6 * idx[n] + 6]
        T = np.eye(4); T[:3, :3] = R.from_rotvec(p[:3]).as_matrix(); T[:3, 3] = p[3:]
        return T

    def res(x):
        out = []
        for e in edges:
            Tf, Tt = pose(x, e["f"]), pose(x, e["t"])
            E = np.linalg.inv(e["T"]) @ np.linalg.inv(Tf) @ Tt
            out += list(E[:3, 3] / e["st"])
            out += list(R.from_matrix(E[:3, :3]).as_rotvec() / e["sr"])
        return out

    x0 = np.zeros(6 * len(free))
    for n in free:
        x0[6 * idx[n]:6 * idx[n] + 3] = R.from_matrix(init[n][:3, :3]).as_rotvec()
        x0[6 * idx[n] + 3:6 * idx[n] + 6] = init[n][:3, 3]
    sol = least_squares(res, x0, method="lm")
    return {n: pose(sol.x, n) for n in nodes}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--edges", required=True)
    ap.add_argument("--drift", type=float, default=0.0,
                    help="inject synthetic chain drift [m] to open the loop for the demo")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    edges = load_edges(a.edges)
    nodes = sorted({e["f"] for e in edges} | {e["t"] for e in edges})
    chain_edges = [e for e in edges if e["ty"] in (0, 1)]
    init = chain_init(nodes, chain_edges)

    # optional synthetic drift: bend each chain edge a little so the loop opens
    if a.drift > 0:
        for e in chain_edges:
            ang = a.drift * 0.04
            e["T"] = e["T"] @ se3([a.drift * 0.05, 0, 0], R.from_rotvec([0, 0, ang]).as_quat())
        init = chain_init(nodes, chain_edges)

    opt = optimize(nodes, edges, init)

    P0 = np.array([init[n][:3, 3] for n in nodes])
    P1 = np.array([opt[n][:3, 3] for n in nodes])
    ni = {n: i for i, n in enumerate(nodes)}
    estyle = {0: ("#888", "-", 1.2), 1: ("#ff7f0e", "--", 2.4), 2: ("#2ca02c", "-", 2.6)}

    fig, axs = plt.subplots(1, 2, figsize=(15, 7), dpi=100)
    for ax, P, ttl in ((axs[0], P0, "BEFORE: open loop (chain drift)"),
                       (axs[1], P1, "AFTER: anchor-graph optimized (loop closed)")):
        for e in edges:
            c, ls, lw = estyle[e["ty"]]
            i, j = ni[e["f"]], ni[e["t"]]
            ax.plot([P[i, 0], P[j, 0]], [P[i, 1], P[j, 1]], ls, color=c, lw=lw, zorder=3)
        ax.scatter(P[:, 0], P[:, 1], s=140, c="k", marker="D", zorder=5)
        for n in nodes:
            ax.annotate(str(n), (P[ni[n], 0], P[ni[n], 1]), color="w", fontsize=8,
                        ha="center", va="center", zorder=6)
        ax.set_aspect("equal"); ax.set_title(ttl, fontsize=10); ax.grid(alpha=0.2)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    fig.suptitle("Anchor-graph optimization (R1.1): gray=odom (rigid)  orange-dash=SOFT (flexes)  "
                 "green=HARD (pins). Covariance-weighted SE3 pose-graph.", fontsize=10)
    fig.tight_layout(); fig.savefig(a.out)
    # report per-chain-edge flex (how much each stretched to close the loop)
    print(f"nodes={len(nodes)} edges={len(edges)}  loop-close shift of last node: "
          f"{np.linalg.norm(P1[-1]-P0[-1]):.2f} m")
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
