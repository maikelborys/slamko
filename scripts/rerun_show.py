#!/usr/bin/env python3
"""Show a slamko map in Rerun (rerun.io): the volumetric TSDF surface + the sparse
LANDMARKS + submap anchors + the pose-graph EDGES by type (chain / loop-weld) + the
camera trajectory animated over a timeline. Scrub the viewer to replay the build.

  rerun_show.py <mesh.ply> <traj.tum> <out.rrd> [--archive DIR] [--connect] [more_traj.tum]

--archive DIR : load landmarks + submap anchors + anchor_edges.csv from a .smap archive.
--connect     : stream to a running `rerun --serve-web` (live browser update) instead of .rrd.
Run with: /tmp/rrviewer/bin/python3 scripts/rerun_show.py ...
"""
import glob
import os
import struct
import sys
import numpy as np
import rerun as rr


def viridis(t):
    A = np.array([[68, 1, 84], [59, 82, 139], [33, 144, 141],
                  [93, 201, 99], [253, 231, 37]], float)
    x = np.clip(t, 0, 1) * 4
    i = np.clip(x.astype(int), 0, 3)
    f = (x - i)[:, None]
    return (A[i] * (1 - f) + A[i + 1] * f).astype(np.uint8)


def read_ply(path):
    L = open(path).readlines()
    h = next(i for i, l in enumerate(L) if l.startswith("end_header"))
    nv = next(int(l.split()[2]) for l in L[:h] if l.startswith("element vertex"))
    nf = next((int(l.split()[2]) for l in L[:h] if l.startswith("element face")), 0)
    V = np.array([l.split()[:3] for l in L[h + 1:h + 1 + nv]], np.float32)
    F = np.array([l.split()[1:4] for l in L[h + 1 + nv:h + 1 + nv + nf]], np.uint32) \
        if nf else None
    return V, F


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_archive(dirpath):
    """Return (landmarks Nx3 global, anchors{id:(R,t)}, keyframes Mx3 global)."""
    lms, anchors, kfs = [], {}, []
    for p in sorted(glob.glob(os.path.join(dirpath, "submap_*.smap"))):
        sid = int(os.path.basename(p)[7:-5])
        d = open(p, "rb").read()
        if d[:4] not in (b"SMP1", b"SMP2", b"SMP3", b"SMP4", b"SMP5", b"SMP6"):
            continue
        off = 4 + 8
        q = struct.unpack_from("<7d", d, off); off += 56
        R = quat_R(*q[:4]); t = np.array(q[4:7]); anchors[sid] = (R, t)
        (nk,) = struct.unpack_from("<Q", d, off); off += 8
        for _ in range(nk):
            kq = struct.unpack_from("<Qd7d", d, off); off += 72
            kt = np.array(kq[6:9])  # kf pose translation (after id,ts,quat)
            kfs.append(R @ kt + t)
        (nl,) = struct.unpack_from("<Q", d, off); off += 8
        lm = np.frombuffer(d, offset=off, count=nl, dtype=np.dtype(
            [("id", "<u8"), ("x", "<f8"), ("y", "<f8"), ("z", "<f8"), ("dr", "<i4")]))
        xyz = np.stack([lm["x"], lm["y"], lm["z"]], 1)
        if len(xyz):
            lms.append((R @ xyz.T).T + t)
    L = np.concatenate(lms) if lms else np.zeros((0, 3))
    return L, anchors, np.array(kfs) if kfs else np.zeros((0, 3))


def main():
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    flags = [x for x in sys.argv[1:] if x.startswith("--")]
    archive = next((f.split("=", 1)[1] for f in flags if f.startswith("--archive=")), None)
    if "--archive" in flags:
        archive = a[3] if len(a) > 3 else None
    connect = "--connect" in flags
    ply, out = a[0], a[2]
    trajs = [a[1]]

    V, F = read_ply(ply)
    z = V[:, 2]
    lo, hi = np.percentile(z, 2), np.percentile(z, 98)
    cols = viridis(np.clip((z - lo) / (hi - lo + 1e-9), 0, 1))

    rr.init("slamko_map", spawn=False)
    if connect:
        rr.connect_grpc("rerun+http://127.0.0.1:9876/proxy")
    else:
        rr.save(out)

    # 1) volumetric TSDF — either the marching-cubes surface, or nvblox-style voxel CUBES
    if "--cubes" in flags:
        vox = 0.05
        key = np.round(V / vox).astype(np.int64)
        _, idx = np.unique(key, axis=0, return_index=True)   # one cube per occupied voxel
        centers = (key[idx] * vox).astype(np.float32)
        cz = centers[:, 2]
        ccol = viridis(np.clip((cz - lo) / (hi - lo + 1e-9), 0, 1))
        rr.log("world/voxels", rr.Boxes3D(
            centers=centers, half_sizes=np.full((len(centers), 3), vox * 0.5, np.float32),
            colors=ccol, fill_mode="solid"), static=True)
        print(f"  voxel cubes: {len(centers)} occupied voxels @ {vox} m")
    elif F is not None:
        rr.log("world/volumetric",
               rr.Mesh3D(vertex_positions=V, triangle_indices=F, vertex_colors=cols),
               static=True)
    else:
        rr.log("world/volumetric", rr.Points3D(V, colors=cols, radii=0.012), static=True)

    # 2) sparse landmarks + submap anchors + pose-graph edges by type
    if archive and os.path.isdir(archive):
        L, anchors, kfs = load_archive(archive)
        if len(L):
            rr.log("world/landmarks", rr.Points3D(L, colors=[180, 180, 190], radii=0.015),
                   static=True)
        if len(kfs):
            rr.log("world/keyframes", rr.Points3D(kfs, colors=[80, 160, 230], radii=0.04),
                   static=True)
        ecsv = os.path.join(archive, "anchor_edges.csv")
        if os.path.exists(ecsv):
            chain, loop = [], []
            for line in open(ecsv).readlines()[1:]:
                c = line.split(",")
                fr, to, ty = int(c[0]), int(c[1]), int(c[2])
                if fr in anchors and to in anchors:
                    seg = [anchors[fr][1], anchors[to][1]]
                    (loop if ty != 0 else chain).append(seg)
            if chain:
                rr.log("world/edges/chain", rr.LineStrips3D(chain, colors=[120, 120, 120],
                       radii=0.006), static=True)
            if loop:  # type != 0 = loop closure / weld
                rr.log("world/edges/loop_weld", rr.LineStrips3D(loop, colors=[45, 200, 80],
                       radii=0.02), static=True)
            print(f"  edges: {len(chain)} chain, {len(loop)} loop/weld")
        print(f"  landmarks={len(L)} keyframes={len(kfs)} submaps={len(anchors)}")

    # 3) trajectory animated over a timeline
    P = np.loadtxt(trajs[0])[:, 1:4]
    for i in range(0, len(P), 3):
        rr.set_time("frame", sequence=i)
        rr.log("world/trajectory", rr.LineStrips3D([P[:i + 1]], colors=[230, 120, 40]))
        rr.log("world/camera", rr.Points3D([P[i]], colors=[230, 30, 30], radii=0.08))
    print(f"logged volumetric {len(V)} verts + trajectory {len(P)} "
          f"{'(LIVE stream)' if connect else '-> ' + out}")


if __name__ == "__main__":
    main()
