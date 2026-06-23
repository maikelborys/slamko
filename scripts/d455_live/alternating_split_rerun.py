#!/usr/bin/env python3
"""Live D455 ALTERNATING emitter @ 90fps -> Rerun. Splits each frame into
'vio_eye' (clean, emitter off) and 'depth_eye' (dotted, emitter on) by image
DOT-ENERGY (robust to the 1-frame metadata offset), logs depth + the per-channel
FPS. Camera drained at 90; pushed to the web viewer throttled."""
import sys
import time
from collections import deque
import numpy as np
import cv2
import pyrealsense2 as rs

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 1800.0
W, H, FPS = 848, 480, 90
PUSH_HZ = 12.0

import rerun as rr
rr.init("d455_alt")
rr.connect_grpc("rerun+http://127.0.0.1:9876/proxy")

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
cfg.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)
prof = pipe.start(cfg)
ds = prof.get_device().first_depth_sensor()
ds.set_option(rs.option.emitter_enabled, 1)
ds.set_option(rs.option.emitter_on_off, 1)            # ALTERNATE on/off per frame
ds.set_option(rs.option.laser_power, ds.get_option_range(rs.option.laser_power).max)
if ds.supports(rs.option.enable_auto_exposure):
    ds.set_option(rs.option.enable_auto_exposure, 1)  # brighten the clean VIO frame
scale = ds.get_depth_scale()
clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))  # display contrast boost

intr = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
us, vs = np.meshgrid(np.arange(W), np.arange(H))
us = us.ravel(); vs = vs.ravel()


def dot_score(ir):
    # high-pass energy: the emitter dots are sharp high-frequency speckle
    hp = ir.astype(np.float32) - cv2.GaussianBlur(ir, (0, 0), 2.0)
    return float((np.abs(hp) > 12).mean())


scores = deque(maxlen=90)
for _ in range(40):                                   # warm up + seed scores
    f = pipe.wait_for_frames()
    scores.append(dot_score(np.asanyarray(f.get_infrared_frame(1).get_data())))

t0 = time.time(); tlast = t0
n_vio = n_dep = 0
push_vio = push_dep = 0.0
fps_vio = fps_dep = 0.0
while time.time() - t0 < DUR:
    f = pipe.wait_for_frames()
    irf = f.get_infrared_frame(1)
    ir = np.asanyarray(irf.get_data())
    s = dot_score(ir)
    thr = np.median(scores)
    scores.append(s)
    dotted = s > thr                                  # median split self-calibrates
    now = time.time()
    if dotted:
        n_dep += 1
        if now - push_dep >= 1.0 / PUSH_HZ:
            push_dep = now
            rr.log("depth_eye/ir", rr.Image(ir))
            dm = np.asanyarray(f.get_depth_frame().get_data()).astype(np.float32) * scale
            rr.log("depth_eye/depth", rr.DepthImage(dm, meter=1.0, colormap="turbo"))
            z = dm.ravel(); m = (z > 0.2) & (z < 6.0)
            idx = np.where(m)[0]; st = max(1, len(idx) // 7000); sel = idx[::st]
            zz = z[sel]
            pts = np.stack([(us[sel]-cx)/fx*zz, (vs[sel]-cy)/fy*zz, zz], 1)
            t = np.clip((zz - 0.2) / 5.8, 0, 1)        # vivid TURBO (cheerful)
            tcol = cv2.applyColorMap((t * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            rr.log("depth_eye/cloud", rr.Points3D(
                pts, colors=tcol.reshape(-1, 3)[:, ::-1], radii=0.008))  # BGR->RGB
    else:
        n_vio += 1
        if now - push_vio >= 1.0 / PUSH_HZ:
            push_vio = now
            rr.log("vio_eye/ir", rr.Image(clahe.apply(ir)))   # contrast-boosted

    if now - tlast >= 1.0:
        dt = now - tlast
        fps_vio, fps_dep = n_vio/dt, n_dep/dt
        rr.log("fps/vio_clean", rr.Scalars(fps_vio))
        rr.log("fps/depth_dotted", rr.Scalars(fps_dep))
        print(f"[{int(now-t0)}s] VIO(clean)={fps_vio:.0f} fps  "
              f"DEPTH(dotted)={fps_dep:.0f} fps", flush=True)
        n_vio = n_dep = 0; tlast = now

pipe.stop()
print(f"done. last VIO={fps_vio:.0f} DEPTH={fps_dep:.0f}")
