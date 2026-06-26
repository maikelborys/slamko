#!/usr/bin/env python3
# tsdf_slice.py — horizontal MID-HEIGHT cut of a TSDF mesh (a floor-plan-style cross-section).
# Loads the nvblox ascii PLY (x y z nx ny nz r g b), finds the vertical (up) axis as the smallest-
# extent axis (single-floor house), takes a thin slab at mid-height, and renders it top-down.
#   usage: tsdf_slice.py <mesh.ply> <out.png> [band_m=0.15] [height_frac=0.5]
import sys, numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

ply = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else ply.replace('.ply', '_slice.png')
band = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15      # slab half-thickness [m]
hfrac = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5       # 0=floor, 1=ceiling

# --- read ascii PLY vertices ---
xs = []; cols = []
with open(ply) as f:
    assert f.readline().startswith('ply')
    nv = 0
    for line in f:
        if line.startswith('element vertex'): nv = int(line.split()[-1])
        if line.startswith('end_header'): break
    for i, line in enumerate(f):
        if i >= nv: break
        p = line.split()
        if len(p) >= 3:
            xs.append((float(p[0]), float(p[1]), float(p[2])))
            if len(p) >= 9:
                cols.append((int(p[6])/255, int(p[7])/255, int(p[8])/255))
P = np.array(xs)
C = np.array(cols) if cols else None
print(f"loaded {len(P)} vertices")

# --- detect up-axis = smallest spatial extent (floor->ceiling << floor span) ---
ext = P.max(0) - P.min(0)
up = int(np.argmin(ext))
floor_axes = [a for a in range(3) if a != up]
names = ['x', 'y', 'z']
print(f"extents {ext.round(2)} -> up-axis = {names[up]} ({ext[up]:.2f} m); floor = {names[floor_axes[0]]},{names[floor_axes[1]]}")

# --- mid-height of the DENSE living space (exclude sparse stair/drift tails) ---
zv = P[:, up]
hist, edges = np.histogram(zv, bins=60)
dense = hist > 0.15 * hist.max()                       # occupied bands
zc_band = edges[:-1][dense]
zlo, zhi = zc_band.min(), zc_band.max() + (edges[1]-edges[0])
zc = zlo + hfrac * (zhi - zlo)
sel = np.abs(zv - zc) <= band
S = P[sel]
print(f"dense living z=[{zlo:.2f},{zhi:.2f}] -> mid-height z={zc:.2f} m (band ±{band}) -> {sel.sum()} verts")

u = S[:, floor_axes[0]]; v = S[:, floor_axes[1]]

# --- rasterize the slab to an occupancy-density grid (reads like a floor plan) ---
res = 0.04
ulo, uhi = u.min()-0.3, u.max()+0.3; vlo, vhi = v.min()-0.3, v.max()+0.3
H, xe, ye = np.histogram2d(u, v, bins=[int((uhi-ulo)/res), int((vhi-vlo)/res)],
                           range=[[ulo, uhi], [vlo, vhi]])
dens = np.log1p(H.T)

fig, ax = plt.subplots(figsize=(12, 12*(vhi-vlo)/(uhi-ulo)), dpi=120)
ax.imshow(dens, origin='lower', extent=[ulo, uhi, vlo, vhi], cmap='turbo',
          interpolation='nearest', aspect='equal')
# overlay the trajectory if present (graph.tum: t x y z qx..)
import os
traj = os.path.join(os.path.dirname(ply), 'graph.tum')
if os.path.exists(traj):
    T = np.loadtxt(traj)
    if T.ndim == 2 and T.shape[0] > 2:
        ax.plot(T[:, 1+floor_axes[0]], T[:, 1+floor_axes[1]], '-', color='#ffffff',
                lw=1.2, alpha=0.85, label='trajectory')
        ax.legend(loc='upper right', facecolor='#111', edgecolor='#444', labelcolor='#ddd')
ax.set_xlabel(f'{names[floor_axes[0]]} [m]', color='#bbb')
ax.set_ylabel(f'{names[floor_axes[1]]} [m]', color='#bbb')
ax.set_title(f'TSDF mid-height cut  ·  z={zc:.2f} m (±{band} m)  ·  {sel.sum()} verts  ·  casa 45fps flash bag',
             color='#eee', fontsize=12)
ax.tick_params(colors='#888')
for s in ax.spines.values(): s.set_color('#333')
fig.patch.set_facecolor('#0a0a0a')
fig.tight_layout()
fig.savefig(out, facecolor='#0a0a0a', bbox_inches='tight')
print(f"wrote {out}")
