#!/usr/bin/env python3
"""Build a wavemap (hashed-chunked wavelet octree) from slamko HW depth (.skdf) at
the corrected keyframe world poses, and report its memory vs nvblox. Multi-resolution
global-tier candidate (DEFERRED to building/campus scale; see RESEARCH_LIFELONG_NAV_ARCH_01).
Run with the FFS venv python (has pywavemap):
  ~/coding/FFS/.venv/bin/python3 scripts/casa_wavemap.py
casa measured: 10 MB RAM / 5.3 MB .wvmp (498 frames) vs nvblox ~28 MB / mesh 16 MB."""
import glob, os, struct
import numpy as np
import pywavemap as wave

ARCH = '/tmp/slamko_casa/map'
SKDF = '/tmp/slamko_casa_skdf'
OUT = '/tmp/casa.wvmp'
FX = FY = 426.1532
CX, CY = 423.6672, 240.5506
T_BODY_CAM = np.array([[1, 0, 0, -0.03022], [0, 1, 0, 0.0074],
                       [0, 0, 1, 0.01602], [0, 0, 0, 1]], float)


def quatT(q):
    x, y, z, w = q[:4]
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = q[4:7]
    return T


kf_world = {}
for p in sorted(glob.glob(os.path.join(ARCH, 'submap_*.smap'))):
    d = open(p, 'rb').read()
    if d[:4] not in (b'SMP1', b'SMP2', b'SMP3', b'SMP4', b'SMP5', b'SMP6'):
        continue
    off = 4 + 8
    Tanchor = quatT(struct.unpack_from('<7d', d, off)); off += 56
    nk = struct.unpack_from('<Q', d, off)[0]; off += 8
    for _ in range(nk):
        kid = struct.unpack_from('<Q', d, off)[0]
        q = struct.unpack_from('<7d', d, off + 16)
        kf_world[kid] = Tanchor @ quatT(q)
        off += 72

your_map = wave.Map.create({"type": "hashed_chunked_wavelet_octree",
                            "min_cell_width": {"meters": 0.05}})
pipeline = wave.Pipeline(your_map)
pipeline.add_integrator("it", {
    "projection_model": {"type": "pinhole_camera_projector", "width": 848,
                         "height": 480, "fx": FX, "fy": FY, "cx": CX, "cy": CY},
    "measurement_model": {"type": "continuous_ray", "range_sigma": {"meters": 0.02},
                          "scaling_free": 0.2, "scaling_occupied": 0.4},
    "integration_method": {"type": "hashed_chunked_wavelet_integrator",
                           "min_range": {"meters": 0.3}, "max_range": {"meters": 4.0}}})

n = 0
for f in sorted(glob.glob(os.path.join(SKDF, 'kf_*.skdf'))):
    kid = int(os.path.basename(f)[3:-5])
    if kid not in kf_world:
        continue
    d = open(f, 'rb').read()
    w, h = struct.unpack_from('<ii', d, 16)
    depth = np.frombuffer(d, dtype='<f4', count=w*h, offset=184).reshape(h, w).astype(np.float32)
    img = wave.Image(depth.T.copy())
    pipeline.run_pipeline(["it"], wave.PosedImage(wave.Pose(kf_world[kid] @ T_BODY_CAM), img))
    n += 1
your_map.prune()
your_map.store(OUT)
print(f"integrated {n} frames | wavemap RAM {your_map.memory_usage/1e6:.2f} MB | "
      f".wvmp {os.path.getsize(OUT)/1e6:.2f} MB")
del pipeline, your_map
