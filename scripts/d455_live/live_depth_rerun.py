#!/usr/bin/env python3
"""Live D455 depth/IR/3D-cloud → Rerun web viewer. Night-friendly (active IR).
Streams ~depth 848x480@60 to a running `rerun --serve-web` via gRPC."""
import sys
import time
import numpy as np
import pyrealsense2 as rs
import rerun as rr

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
W, H, FPS = 848, 480, 60

rr.init("d455_live")
rr.connect_grpc("rerun+http://127.0.0.1:9876/proxy")

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
cfg.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)
prof = pipe.start(cfg)
ds = prof.get_device().first_depth_sensor()
ds.set_option(rs.option.emitter_enabled, 1)
ds.set_option(rs.option.laser_power, ds.get_option_range(rs.option.laser_power).max)
scale = ds.get_depth_scale()

# intrinsics for back-projection
intr = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
us, vs = np.meshgrid(np.arange(W), np.arange(H))
us = us.ravel(); vs = vs.ravel()

rr.log("d455", rr.ViewCoordinates.RDF, static=True)   # camera optical frame

VIEW_HZ = 12.0          # throttle what we PUSH to the viewer (web proxy bandwidth)
CLOUD_PTS = 7000        # target cloud size (web viewer chokes on 57k @ high rate)
t0 = time.time(); n = 0; tlast = t0; tpush = 0.0
while time.time() - t0 < DUR:
    fr = pipe.wait_for_frames()
    d = fr.get_depth_frame(); ir = fr.get_infrared_frame(1)
    if not d or not ir:
        continue
    n += 1
    now = time.time()
    if now - tpush < 1.0 / VIEW_HZ:    # drain camera at 60, push to viewer at 12
        continue
    tpush = now
    dm = np.asanyarray(d.get_data()).astype(np.float32) * scale
    irimg = np.asanyarray(ir.get_data())
    # NO custom timeline → Rerun uses log_time, the web viewer auto-follows latest.

    rr.log("d455/depth", rr.DepthImage(dm, meter=1.0, colormap="turbo"))
    rr.log("d455/ir", rr.Image(irimg))

    # 3D point cloud — subsample to ~CLOUD_PTS so the web viewer keeps up
    z = dm.ravel()
    m = (z > 0.2) & (z < 6.0)
    idx = np.where(m)[0]
    stride = max(1, len(idx) // CLOUD_PTS)
    sel = idx[::stride]
    zz = z[sel]
    x = (us[sel] - cx) / fx * zz
    y = (vs[sel] - cy) / fy * zz
    pts = np.stack([x, y, zz], 1)
    tcol = np.clip((zz - 0.2) / 5.8, 0, 1)
    A = np.array([[68, 1, 84], [33, 144, 141], [253, 231, 37]], np.float32)
    ci = np.clip((tcol * 2).astype(int), 0, 1)
    cf = (tcol * 2 - ci)[:, None]
    col = (A[ci] * (1 - cf) + A[ci + 1] * cf).astype(np.uint8)
    rr.log("d455/cloud", rr.Points3D(pts, colors=col, radii=0.006))

    if time.time() - tlast > 3.0:
        tlast = time.time()
        print(f"[{int(tlast - t0)}s] streamed {n} frames, last cloud {len(pts)} pts",
              flush=True)

fps = n / (time.time() - t0)
pipe.stop()
print(f"streamed {n} frames @ {fps:.1f} fps live to Rerun")
