#!/usr/bin/env python3
"""Capture D455 depth @ 848x480x60 with the IR emitter ON, measure real fps,
colorize, and report coverage/range. Night-friendly (active IR stereo)."""
import time
import numpy as np
import cv2
import pyrealsense2 as rs

W, H, FPS = 848, 480, 60
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
cfg.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)
prof = pipe.start(cfg)

dev = prof.get_device()
ds = dev.first_depth_sensor()
if ds.supports(rs.option.emitter_enabled):
    ds.set_option(rs.option.emitter_enabled, 1)          # projector ON
if ds.supports(rs.option.laser_power):
    ds.set_option(rs.option.laser_power, ds.get_option_range(rs.option.laser_power).max)
depth_scale = ds.get_depth_scale()
print(f"emitter ON, laser={ds.get_option(rs.option.laser_power)}, depth_scale={depth_scale} m/unit")

# warm-up (auto-exposure settle)
for _ in range(30):
    pipe.wait_for_frames()

# measure real fps over ~1.5 s
t0, n = time.time(), 0
depth_img = ir_img = None
while time.time() - t0 < 1.5:
    fr = pipe.wait_for_frames()
    d = fr.get_depth_frame(); ir = fr.get_infrared_frame(1)
    if not d or not ir:
        continue
    depth_img = np.asanyarray(d.get_data())
    ir_img = np.asanyarray(ir.get_data())
    n += 1
fps = n / (time.time() - t0)
pipe.stop()

dm = depth_img.astype(np.float32) * depth_scale            # metres
valid = dm > 0
cov = 100.0 * valid.mean()
rng = dm[valid]
print(f"REAL fps={fps:.1f}  coverage={cov:.1f}%  "
      f"range={rng.min():.2f}..{np.percentile(rng,99):.2f} m (median {np.median(rng):.2f})")

# colorize: near=warm, far=cool, black=no-data
dclip = np.clip(dm, 0.2, 6.0)
norm = ((dclip - 0.2) / (6.0 - 0.2) * 255).astype(np.uint8)
col = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
col[~valid] = (0, 0, 0)
cv2.putText(col, f"D455 depth 848x480@{fps:.0f}fps  cov {cov:.0f}%  emitter ON",
            (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
cv2.imwrite("/tmp/d455_depth.png", col)
cv2.imwrite("/tmp/d455_ir.png", cv2.cvtColor(ir_img, cv2.COLOR_GRAY2BGR))
print("wrote /tmp/d455_depth.png  /tmp/d455_ir.png")
