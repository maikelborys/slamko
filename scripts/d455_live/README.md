# D455 live-depth scripts (v2a groundwork)

Hands-on RealSense D455 live-depth tooling validated 2026-06-23 (night, then
lit room). These feed **v2a** (live local nvblox costmap). Full findings +
rationale: [`../../docs/V2A_LIVE_DEPTH_D455_01.md`](../../docs/V2A_LIVE_DEPTH_D455_01.md).

## One-time venv (NOT system Python — PEP668)
```bash
python3 -m venv /tmp/rsenv
/tmp/rsenv/bin/pip install pyrealsense2 numpy opencv-python-headless rerun-sdk==0.33
```
The D455 is **exclusive** — only one process can open it. Free it before each run
(`pkill -f <script>`); check holders with `pgrep -af rsenv/bin/python3`.

## Scripts
| script | what it shows |
|---|---|
| `depth_shot.py` | one depth frame @ 848x480x60, emitter on → colorized PNG + coverage/range |
| `emitter_on_off_compare.py` | two STEADY passes (emitter on vs off): IR texture, brightness, depth coverage |
| `test_90fps_lit.py` | 90 fps sustained check + lit on/off comparison |
| `live_depth_rerun.py [secs]` | live depth+IR+3D-cloud → Rerun (throttled 12 Hz push, ~7k pts) |
| `alternating_split_rerun.py [secs]` | **the star:** 90 fps `emitter_on_off` split into `vio_eye` (clean, 45 fps) + `depth_eye` (dotted, 45 fps) by image dot-energy → Rerun, with per-channel FPS scalars |

## Viewer — use the NATIVE Rerun window, not web
The web viewer (`rerun --serve-web`, browser) **chokes** on sustained live streams
(freezes after a few seconds — proxy bandwidth). The **native desktop viewer** is
smooth. Launch it FIRST, then run a script (scripts `connect_grpc` to `:9876`):
```bash
DISPLAY=:0 /tmp/rrviewer/bin/rerun --port 9876 &      # native window, listens :9876
/tmp/rsenv/bin/python3 alternating_split_rerun.py 600 # streams into it
```
Native viewer is real-time live: do NOT press "play" (that replays from frame 0).
Data uses the default `log_time` timeline → the viewer auto-follows latest.
