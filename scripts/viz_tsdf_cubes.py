#!/usr/bin/env python3
"""nvblox-style VOXEL CUBES + trajectory, offline Plotly (interactive HTML).

Voxelize a slamko_tsdf nvblox mesh into occupied cubes (snap vertices to a grid,
one shaded cube per occupied voxel, coloured by HEIGHT = the classic nvblox
elevation look) and overlay the slamko graph trajectory. The mesh was
re-integrated at the CORRECTED graph poses (the live bend), so this is the
deformed, loop-consistent map.

  viz_tsdf_cubes.py <map.ply> <out.html> [traj.tum] [--voxel 0.08] [--max-cubes 60000]
"""
import argparse
import numpy as np


def read_ply_xyz(path):
    """ASCII PLY: x y z [nx ny nz r g b]. Returns Nx3 float."""
    with open(path) as f:
        lines = f.readlines()
    hdr = next(i for i, l in enumerate(lines) if l.startswith("end_header"))
    nv = next(int(l.split()[2]) for l in lines[:hdr] if l.startswith("element vertex"))
    rows = lines[hdr + 1:hdr + 1 + nv]
    return np.array([l.split()[:3] for l in rows], dtype=np.float64)


def read_tum_xyz(path):
    try:
        d = np.loadtxt(path)
        return d[:, 1:4]
    except Exception:
        return None


def viridis(t):
    """Height colormap — matches scripts/rerun_show.py (the established nvblox-cube look)."""
    A = np.array([[68, 1, 84], [59, 82, 139], [33, 144, 141],
                  [93, 201, 99], [253, 231, 37]], float)
    x = np.clip(t, 0, 1) * 4
    i = np.clip(x.astype(int), 0, 3)
    f = (x - i)[:, None]
    return (A[i] * (1 - f) + A[i + 1] * f).astype(np.uint8)


# unit cube: 8 corners, 12 triangles
_C = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
               [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]], float) - 0.5
_TRI = np.array([[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7],
                 [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6],
                 [1, 2, 6], [1, 6, 5], [0, 3, 7], [0, 7, 4]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ply")
    ap.add_argument("out_html")
    ap.add_argument("traj", nargs="?", default="")
    ap.add_argument("--voxel", type=float, default=0.08, help="cube size [m]")
    ap.add_argument("--max-cubes", type=int, default=18000, help="auto-coarsen to fit (small HTML)")
    ap.add_argument("--markers", action="store_true",
                    help="square markers instead of shaded boxes — ~5x smaller HTML, opens fast")
    a = ap.parse_args()
    import plotly.graph_objects as go

    xyz = read_ply_xyz(a.ply)
    # Voxelize: snap to grid, keep unique occupied cells (cube centres). Instead of
    # randomly dropping cubes (holes), AUTO-COARSEN the voxel until the whole map fits
    # the budget → full coverage, the honest nvblox-grid look (no missing walls).
    vox = a.voxel
    for _ in range(8):
        cells = np.unique(np.floor(xyz / vox).astype(np.int64), axis=0)
        if len(cells) <= a.max_cubes:
            break
        vox *= 1.25
    centres = (cells + 0.5) * vox
    n = len(centres)
    if vox != a.voxel:
        print(f"[auto-coarsen] {a.voxel:.3f}→{vox:.3f} m to fit {n} cubes (full coverage, no holes)")
    print(f"voxels: {n} @ {vox:.3f} m")

    # Build ONE Mesh3d of all cubes (8n verts, 12n tris). Colour by height via a
    # NUMERIC intensity + colorscale (NOT a per-vertex rgb-string list — that bloats
    # the HTML ~10x). Round coords to mm to keep the file small.
    V = np.round((centres[:, None, :] + _C[None, :, :] * vox).reshape(-1, 3), 3)
    F = (_TRI[None, :, :] + (np.arange(n) * 8)[:, None, None]).reshape(-1, 3)
    z = V[:, 2]
    intensity = (z - z.min()) / max(z.max() - z.min(), 1e-6)

    fig = go.Figure()
    if a.markers:  # square markers — one per voxel, tiny HTML, opens instantly
        zc = centres[:, 2]
        fig.add_trace(go.Scatter3d(
            x=centres[:, 0], y=centres[:, 1], z=centres[:, 2], mode="markers",
            marker=dict(size=3, symbol="square", color=zc, colorscale="Viridis",
                        showscale=False, opacity=0.9),
            name="nvblox voxels", hoverinfo="skip"))
    else:
        fig.add_trace(go.Mesh3d(
            x=V[:, 0], y=V[:, 1], z=V[:, 2],
            i=F[:, 0], j=F[:, 1], k=F[:, 2],
            intensity=intensity, intensitymode="vertex", colorscale="Viridis",
            showscale=False, flatshading=True, opacity=1.0,
            lighting=dict(ambient=0.55, diffuse=0.8, specular=0.15),
            name="nvblox voxels", hoverinfo="skip"))

    traj = read_tum_xyz(a.traj) if a.traj else None
    if traj is not None:
        fig.add_trace(go.Scatter3d(
            x=traj[:, 0], y=traj[:, 1], z=traj[:, 2],
            mode="lines", line=dict(color="red", width=6), name="trajectory"))
        fig.add_trace(go.Scatter3d(
            x=[traj[0, 0]], y=[traj[0, 1]], z=[traj[0, 2]],
            mode="markers", marker=dict(color="lime", size=6), name="start"))

    fig.update_layout(
        title=f"slamko live volumetric — {n} nvblox voxels @ {vox:.3f} m + trajectory",
        scene=dict(aspectmode="data", xaxis_title="x [m]", yaxis_title="y [m]",
                   zaxis_title="z [m]", bgcolor="rgb(12,12,16)"),
        paper_bgcolor="rgb(12,12,16)", font=dict(color="white"))
    fig.write_html(a.out_html)
    print("wrote", a.out_html)


if __name__ == "__main__":
    main()
