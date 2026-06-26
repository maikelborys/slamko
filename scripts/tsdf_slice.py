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
hfrac = float(sys.argv[4]) if len(sys.argv) > 4 else -1.0      # <0 = auto (trajectory height); 0..1 = floor..ceiling

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

# --- choose the cut height ---
# DEFAULT: slice at the NAVIGABLE plane = the trajectory's own height (where walls and path must
# coexist). A slice taken above/below the camera height makes the path appear to cross walls that
# only exist at that other height (a pure projection artefact). height_frac<0 keeps this auto mode;
# height_frac>=0 forces a fraction of the dense vertical extent (0=floor, 1=ceiling).
import os
zv = P[:, up]
traj = os.path.join(os.path.dirname(ply), 'graph.tum')
T = np.loadtxt(traj) if os.path.exists(traj) else None
if hfrac < 0 and T is not None and T.ndim == 2:
    zc = float(np.median(T[:, 1+up]))
    print(f"AUTO cut at the trajectory's median height z={zc:.2f} m (the navigable plane)")
else:
    hist, edges = np.histogram(zv, bins=60)
    dense = hist > 0.15 * hist.max()
    zlo, zhi = edges[:-1][dense].min(), edges[:-1][dense].max() + (edges[1]-edges[0])
    zc = zlo + (hfrac if hfrac >= 0 else 0.5) * (zhi - zlo)
    print(f"dense living z=[{zlo:.2f},{zhi:.2f}] -> cut z={zc:.2f} m")
sel = np.abs(zv - zc) <= band
S = P[sel]
print(f"cut z={zc:.2f} m (band ±{band}) -> {sel.sum()} verts")

u = S[:, floor_axes[0]]; v = S[:, floor_axes[1]]

# --- rasterize the slab to an occupancy-density grid (reads like a floor plan) ---
res = 0.04
ulo, uhi = u.min()-0.3, u.max()+0.3; vlo, vhi = v.min()-0.3, v.max()+0.3
H, xe, ye = np.histogram2d(u, v, bins=[int((uhi-ulo)/res), int((vhi-vlo)/res)],
                           range=[[ulo, uhi], [vlo, vhi]])
dens = np.log1p(H.T)

asp = (vhi-vlo)/(uhi-ulo)
fig, ax = plt.subplots(figsize=(13, max(7, min(20, 13*asp))), dpi=135)
ax.imshow(dens, origin='lower', extent=[ulo, uhi, vlo, vhi], cmap='turbo',
          interpolation='nearest', aspect='equal')
# overlay the trajectory (graph.tum). Draw the FULL path faint + the portion AT the cut height
# bright, so "is the path in free space at this slice?" is read honestly (not a projection).
if T is not None and T.ndim == 2 and T.shape[0] > 2:
    tu, tv, tz = T[:, 1+floor_axes[0]], T[:, 1+floor_axes[1]], T[:, 1+up]
    ax.plot(tu, tv, '-', color='#888888', lw=0.8, alpha=0.5, label='full path (all heights)')
    atht = np.abs(tz - zc) <= max(band, 0.25)
    ax.plot(np.where(atht, tu, np.nan), np.where(atht, tv, np.nan), '-', color='#ffffff',
            lw=1.6, alpha=0.95, label=f'path at cut height (±{max(band,0.25):.2f} m)')
    ax.legend(loc='upper right', facecolor='#111', edgecolor='#444', labelcolor='#ddd', fontsize=9)
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
