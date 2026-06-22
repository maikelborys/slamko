#!/usr/bin/env python3
"""Visualize a slamko_tsdf nvblox mesh: a horizontal CUT at mid-height of the
TSDF (top-down cross-section — walls show as the outline) + an interactive 3D
view, with the slamko trajectory overlaid. The mesh was re-integrated at the
graph's CORRECTED poses, so this is the bent map.

  viz_tsdf.py <map.ply> <out_prefix> [traj.tum] [--band 0.06] [--slices]
"""
import argparse
import numpy as np


def read_ply_vertices(path):
    with open(path, "r") as f:
        lines = f.readlines()
    hdr = next(i for i, l in enumerate(lines) if l.startswith("end_header"))
    nv = next(int(l.split()[2]) for l in lines[:hdr]
              if l.startswith("element vertex"))
    rows = lines[hdr + 1:hdr + 1 + nv]
    xyz = np.array([l.split()[:3] for l in rows], dtype=np.float64)
    return xyz


def read_tum_xy(path):
    try:
        d = np.loadtxt(path)
        return d[:, 1], d[:, 2]  # tx, ty
    except Exception:
        return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ply")
    ap.add_argument("out_prefix")
    ap.add_argument("traj", nargs="?", default="")
    ap.add_argument("--band", type=float, default=0.06, help="cut half-thickness [m]")
    ap.add_argument("--slices", action="store_true")
    a = ap.parse_args()

    xyz = read_ply_vertices(a.ply)
    print(f"{len(xyz)} mesh vertices")
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    zmin, zmax = np.percentile(z, 2), np.percentile(z, 98)
    zmid = float(np.median(z))  # robust to remaining depth-noise tails
    print(f"z robust range [{zmin:.2f}, {zmax:.2f}] m  → mid-cut at z={zmid:.2f}")

    tx, ty = (None, None)
    if a.traj:
        tx, ty = read_tum_xy(a.traj)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # ---- (1) horizontal CUT at mid-height ----
    band = (z > zmid - a.band) & (z < zmid + a.band)
    fig, ax = plt.subplots(figsize=(11, 11))
    ax.scatter(x[band], y[band], s=1.2, c="#1f3b73", alpha=0.5, linewidths=0)
    if tx is not None:
        ax.plot(tx, ty, "-", color="#e8590c", lw=1.6, label="slamko trajectory")
        ax.legend(loc="upper right")
    # clip view to where the structure actually is (drop far noise speckle)
    if band.sum() > 100:
        ax.set_xlim(np.percentile(x[band], 1), np.percentile(x[band], 99))
        ax.set_ylim(np.percentile(y[band], 1), np.percentile(y[band], 99))
    ax.set_aspect("equal")
    ax.set_title(f"casa40 — TSDF horizontal cut @ z={zmid:.2f} m (±{a.band} m)\n"
                 f"nvblox mesh re-integrated at CORRECTED poses ({len(xyz)} verts)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(a.out_prefix + "_cut_mid.png", dpi=130)
    print("wrote", a.out_prefix + "_cut_mid.png")

    # ---- (2) a few stacked slices (floor → ceiling) ----
    if a.slices:
        hs = np.linspace(zmin + 0.1, zmax - 0.1, 6)
        fig, axs = plt.subplots(2, 3, figsize=(16, 11))
        for h, ax in zip(hs, axs.ravel()):
            b = (z > h - a.band) & (z < h + a.band)
            ax.scatter(x[b], y[b], s=0.8, c="#1f3b73", alpha=0.5, linewidths=0)
            ax.set_aspect("equal"); ax.set_title(f"z={h:.2f} m  ({b.sum()} pts)")
            ax.grid(alpha=0.3)
        fig.suptitle("casa40 TSDF — horizontal cuts floor→ceiling")
        fig.tight_layout(); fig.savefig(a.out_prefix + "_slices.png", dpi=110)
        print("wrote", a.out_prefix + "_slices.png")

    # ---- (2b) static 3D perspective (so it shows inline), colored by height ----
    n = len(xyz)
    step3 = max(1, n // 70000)
    s3 = slice(None, None, step3)
    keep = (z[s3] > zmin) & (z[s3] < zmax)
    fig = plt.figure(figsize=(13, 10))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(x[s3][keep], y[s3][keep], z[s3][keep], s=2.0,
               c=z[s3][keep], cmap="viridis", linewidths=0, alpha=0.7)
    if tx is not None:
        ax.plot(tx, ty, np.full_like(tx, zmid), color="#e8590c", lw=2.0)
    ax.set_title(f"casa40 — nvblox TSDF map (re-integrated at CORRECTED poses, "
                 f"{n} verts)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.view_init(elev=28, azim=-60)
    try:
        ax.set_box_aspect((np.ptp(x[s3][keep]), np.ptp(y[s3][keep]),
                           np.ptp(z[s3][keep])))
    except Exception:
        pass
    fig.tight_layout(); fig.savefig(a.out_prefix + "_3d.png", dpi=120)
    print("wrote", a.out_prefix + "_3d.png")

    # ---- (3) interactive 3D (downsampled), colored by height ----
    try:
        import plotly.graph_objects as go
        n = len(xyz)
        step = max(1, n // 120000)             # cap ~120k pts for the browser
        s = slice(None, None, step)
        fig = go.Figure(go.Scatter3d(
            x=x[s], y=y[s], z=z[s], mode="markers",
            marker=dict(size=1.5, color=z[s], colorscale="Viridis",
                        colorbar=dict(title="height [m]"))))
        if tx is not None:
            fig.add_trace(go.Scatter3d(x=tx, y=ty, z=np.full_like(tx, zmid),
                          mode="lines", line=dict(color="orange", width=4),
                          name="trajectory"))
        fig.update_layout(
            title=f"casa40 nvblox TSDF map — {n} verts (corrected poses)",
            scene=dict(aspectmode="data"))
        fig.write_html(a.out_prefix + "_3d.html")
        print("wrote", a.out_prefix + "_3d.html")
    except Exception as e:
        print("plotly 3d skipped:", e)


if __name__ == "__main__":
    main()
