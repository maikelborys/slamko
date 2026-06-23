#!/usr/bin/env python3
"""Two STEADY passes (no metadata ambiguity): emitter ON vs OFF, at night.
Show the real tradeoff: IR texture + depth coverage + image brightness."""
import numpy as np
import cv2
import pyrealsense2 as rs

W, H, FPS = 848, 480, 60


def grab(emit):
    p = rs.pipeline(); c = rs.config()
    c.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    c.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)
    pr = p.start(c); s = pr.get_device().first_depth_sensor()
    if s.supports(rs.option.emitter_on_off):
        s.set_option(rs.option.emitter_on_off, 0)         # NO alternating
    s.set_option(rs.option.emitter_enabled, emit)         # steady 0 or 1
    if emit and s.supports(rs.option.laser_power):
        s.set_option(rs.option.laser_power, s.get_option_range(rs.option.laser_power).max)
    scale = s.get_depth_scale()
    for _ in range(50):                                   # settle exposure + emitter
        p.wait_for_frames()
    f = p.wait_for_frames()
    ir = np.asanyarray(f.get_infrared_frame(1).get_data())
    dm = np.asanyarray(f.get_depth_frame().get_data()).astype(np.float32) * scale
    p.stop()
    return ir, dm


def corners(g):
    c = cv2.goodFeaturesToTrack(g, 2000, 0.02, 7)
    return 0 if c is None else len(c)


for emit, tag, fname in [(1, "EMITTER ON", "on"), (0, "EMITTER OFF", "off")]:
    ir, dm = grab(emit)
    v = dm > 0
    nc = corners(ir)
    bright = ir.mean()
    use = "depth (nvblox)" if emit else "VIO/reloc (OKVIS/XFeat)"
    col = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
    cv2.putText(col, f"{tag} -> {use}", (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(col, f"corners={nc}  brightness={bright:.0f}/255  depth_cov={100*v.mean():.0f}%",
                (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.imwrite(f"/tmp/emit_{fname}.png", col)
    # depth colorized
    norm = (np.clip(dm, 0.2, 6.0) - 0.2) / 5.8
    dcol = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    dcol[~v] = 0
    cv2.putText(dcol, f"DEPTH ({tag.lower()})  cov {100*v.mean():.0f}%", (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(f"/tmp/depth_{fname}.png", dcol)
    print(f"{tag}: corners={nc}  brightness={bright:.0f}  depth_cov={100*v.mean():.0f}%")
print("wrote /tmp/emit_on.png /tmp/emit_off.png /tmp/depth_on.png /tmp/depth_off.png")
