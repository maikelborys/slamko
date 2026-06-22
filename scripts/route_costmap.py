#!/usr/bin/env python3
"""Build a floor-anchored 2D nav costmap from a slamko_tsdf nvblox mesh and plan an
A→B route over it with A*. Obstacles = mesh vertices in the robot's height band
(above the floor, below the ceiling) flattened to a grid + inflated by the robot
radius. Free = mapped area minus inflated obstacles. This is the global-planning
surface the navigation goal ("ruta de A a B") needs.

  route_costmap.py <map.ply> <traj.tum> <out_prefix>
      [--res 0.05] [--robot-radius 0.20] [--cam-height 0.40]
      [--band-low 0.10] [--band-high 1.60]
"""
import argparse
import heapq
import numpy as np


def read_ply_xyz(path):
    with open(path) as f:
        L = f.readlines()
    h = next(i for i, l in enumerate(L) if l.startswith("end_header"))
    nv = next(int(l.split()[2]) for l in L[:h] if l.startswith("element vertex"))
    return np.array([l.split()[:3] for l in L[h + 1:h + 1 + nv]], dtype=np.float64)


def astar(free, start, goal):
    """8-connected A* on a boolean free-mask (True = traversable). Cells = (r,c)."""
    H, W = free.shape
    sr, sc = start
    gr, gc = goal
    def hh(r, c): return ((r - gr) ** 2 + (c - gc) ** 2) ** 0.5
    openq = [(hh(sr, sc), 0.0, (sr, sc))]
    came, g = {}, {(sr, sc): 0.0}
    nbr = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
           (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    while openq:
        _, gc_, (r, c) = heapq.heappop(openq)
        if (r, c) == (gr, gc):
            path = [(r, c)]
            while (r, c) in came:
                r, c = came[(r, c)]
                path.append((r, c))
            return path[::-1]
        for dr, dc, w in nbr:
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and free[nr, nc]:
                ng = gc_ + w
                if ng < g.get((nr, nc), 1e18):
                    g[(nr, nc)] = ng
                    came[(nr, nc)] = (r, c)
                    heapq.heappush(openq, (ng + hh(nr, nc), ng, (nr, nc)))
    return None


def main():
    import scipy.ndimage as ndi
    ap = argparse.ArgumentParser()
    ap.add_argument("ply"); ap.add_argument("traj"); ap.add_argument("out")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--robot-radius", type=float, default=0.20)
    ap.add_argument("--cam-height", type=float, default=0.40)
    ap.add_argument("--band-low", type=float, default=0.10)
    ap.add_argument("--band-high", type=float, default=1.60)
    a = ap.parse_args()

    xyz = read_ply_xyz(a.ply)
    traj = np.loadtxt(a.traj)
    tx, ty, tz = traj[:, 1], traj[:, 2], traj[:, 3]
    floor = float(np.median(tz)) - a.cam_height
    lo, hi = floor + a.band_low, floor + a.band_high
    print(f"floor≈{floor:.2f} m  robot band [{lo:.2f}, {hi:.2f}] m")

    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    band = (z > lo) & (z < hi)

    # grid bounds from trajectory + obstacles (robust)
    xmin = min(np.percentile(x[band], 1), tx.min()) - 0.5
    xmax = max(np.percentile(x[band], 99), tx.max()) + 0.5
    ymin = min(np.percentile(y[band], 1), ty.min()) - 0.5
    ymax = max(np.percentile(y[band], 99), ty.max()) + 0.5
    W = int((xmax - xmin) / a.res) + 1
    H = int((ymax - ymin) / a.res) + 1

    def to_cell(px, py):
        return (np.clip(((py - ymin) / a.res).astype(int), 0, H - 1),
                np.clip(((px - xmin) / a.res).astype(int), 0, W - 1))

    occ = np.zeros((H, W), np.int32)
    rr, cc = to_cell(x[band], y[band])
    np.add.at(occ, (rr, cc), 1)
    obstacle = occ >= 2                              # ≥2 verts/cell = wall/furniture

    # mapped area = where the camera saw (trajectory tube) ∪ near obstacles
    seen = np.zeros((H, W), bool)
    tr, tc = to_cell(tx, ty)
    seen[tr, tc] = True
    seen = ndi.binary_dilation(seen, iterations=int(0.8 / a.res))  # 0.8 m tube
    seen |= ndi.binary_dilation(obstacle, iterations=2)

    infl = ndi.binary_dilation(obstacle, iterations=int(np.ceil(a.robot_radius / a.res)))
    free = seen & ~infl

    # A = trajectory start; B = the mapped free cell farthest from A (a real goal)
    def snap(px, py):
        r0, c0 = to_cell(np.array([px]), np.array([py]))
        r0, c0 = int(r0[0]), int(c0[0])
        if free[r0, c0]:
            return (r0, c0)
        fr, fc = np.where(free)
        i = np.argmin((fr - r0) ** 2 + (fc - c0) ** 2)
        return (int(fr[i]), int(fc[i]))
    start = snap(tx[0], ty[0])
    # goal = farthest free cell IN THE SAME connected component as A (guarantees a
    # route exists; an isolated free speck elsewhere would be unreachable).
    lab, _ = ndi.label(free, structure=np.ones((3, 3)))
    comp = lab == lab[start]
    fr, fc = np.where(comp)
    far = np.argmax((fr - start[0]) ** 2 + (fc - start[1]) ** 2)
    goal = (int(fr[far]), int(fc[far]))

    path = astar(free, start, goal)
    print(f"grid {H}x{W}  free {free.sum()} cells  "
          f"route: {'FOUND %d steps' % len(path) if path else 'NONE'}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    img = np.full((H, W), 0.6)         # unknown = grey
    img[seen] = 1.0                    # free = white
    img[infl] = 0.0                    # obstacle (inflated) = black
    fig, ax = plt.subplots(figsize=(11, 11 * H / W))
    ax.imshow(img, origin="lower", cmap="gray", vmin=0, vmax=1,
              extent=[xmin, xmax, ymin, ymax])
    ax.plot(tx, ty, "-", color="#e8590c", lw=1.0, alpha=0.7, label="recorded path")
    if path:
        py = ymin + (np.array([p[0] for p in path]) + 0.5) * a.res
        px = xmin + (np.array([p[1] for p in path]) + 0.5) * a.res
        ax.plot(px, py, "-", color="#2f9e44", lw=3, label="planned A→B")
    sa = (xmin + (start[1] + 0.5) * a.res, ymin + (start[0] + 0.5) * a.res)
    gb = (xmin + (goal[1] + 0.5) * a.res, ymin + (goal[0] + 0.5) * a.res)
    ax.scatter(*sa, c="#1971c2", s=160, marker="o", zorder=5, label="A (start)")
    ax.scatter(*gb, c="#e03131", s=200, marker="*", zorder=5, label="B (goal)")
    ax.set_aspect("equal"); ax.legend(loc="upper right")
    ax.set_title(f"casa40 — floor-anchored costmap + A* route (HITNet TSDF, "
                 f"corrected poses)\nrobot r={a.robot_radius} m, band [{lo:.1f},{hi:.1f}] m")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    fig.tight_layout(); fig.savefig(a.out + "_route.png", dpi=130)
    print("wrote", a.out + "_route.png")


if __name__ == "__main__":
    main()
