#!/usr/bin/env python3
"""90 fps test + emitter ON/OFF comparison WITH room light on.
Measures sustained fps and shows the clean (emitter-off) frame is now usable."""
import time
import numpy as np
import cv2
import pyrealsense2 as rs

W, H, FPS = 848, 480, 90


def run(emit, measure=False):
    p = rs.pipeline(); c = rs.config()
    c.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    c.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)
    pr = p.start(c); s = pr.get_device().first_depth_sensor()
    if s.supports(rs.option.emitter_on_off):
        s.set_option(rs.option.emitter_on_off, 0)
    s.set_option(rs.option.emitter_enabled, emit)
    if emit:
        s.set_option(rs.option.laser_power, s.get_option_range(rs.option.laser_power).max)
    scale = s.get_depth_scale()
    for _ in range(50):
        p.wait_for_frames()
    ir = dm = None; n = 0; t0 = time.time()
    dur = 1.5 if measure else 0.05
    while time.time() - t0 < dur:
        f = p.wait_for_frames()
        ir = np.asanyarray(f.get_infrared_frame(1).get_data())
        dm = np.asanyarray(f.get_depth_frame().get_data()).astype(np.float32) * scale
        n += 1
    fps = n / (time.time() - t0)
    p.stop()
    return ir, dm, fps


def corners(g):
    c = cv2.goodFeaturesToTrack(g, 2000, 0.02, 7)
    return 0 if c is None else len(c)


for emit, tag, fn in [(1, "EMITTER ON", "on"), (0, "EMITTER OFF", "off")]:
    ir, dm, fps = run(emit, measure=True)
    v = dm > 0
    use = "depth (nvblox)" if emit else "VIO/reloc (OKVIS/XFeat)"
    col = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
    cv2.putText(col, f"{tag} -> {use}  @ {fps:.0f} fps", (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(col, f"corners={corners(ir)}  brightness={ir.mean():.0f}/255  "
                f"depth_cov={100*v.mean():.0f}%", (10, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.imwrite(f"/tmp/lit_{fn}.png", col)
    print(f"{tag}: REAL fps={fps:.1f}  corners={corners(ir)}  "
          f"brightness={ir.mean():.0f}/255  depth_cov={100*v.mean():.0f}%")
print("wrote /tmp/lit_on.png /tmp/lit_off.png")
