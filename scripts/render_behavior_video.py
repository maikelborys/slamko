#!/usr/bin/env python3
"""Composited slamko BEHAVIOR video — what the system is doing, frame by frame.
Three panels into one mp4 (no rviz; offline, reproducible, LLM-readable):
  LEFT   camera color frame + FAST corners (green) + state text
  RIGHT  top-down 2D Atlas: submaps colored by id, GRAY=not-yet-sealed, anchors,
         edges (gray=odom, orange-dash=SOFT/DR, green=HARD weld), fused trail, live pose
  BOTTOM event ticker: sealed submap / LOOP CLOSED / reloc, revealed at their bag-time

Usage: render_behavior_video.py --bag <dir> --run-dir <results/run/x> --out <mp4> [--fps 6]
Sync: fused.tum is in bag-time; log wall-time is linearly mapped onto the fused span."""
import argparse, os, re, sys, csv, math
import numpy as np
import cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_neverlost import load_smap
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

ap = argparse.ArgumentParser()
ap.add_argument("--bag", required=True)
ap.add_argument("--run-dir", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--fps", type=int, default=6)
ap.add_argument("--color-topic", default="/camera/camera/color/image_raw")
a = ap.parse_args()
MAP_DIR = os.path.join(a.run_dir, "map")
LOG = os.path.join(a.run_dir, "launch.log")

# ---- trajectory (bag-time) ----
def load_tum(p):
    if not os.path.exists(p): return np.zeros((0, 8))
    rows = [list(map(float, l.split())) for l in open(p) if l.strip() and not l.startswith("#")]
    return np.array(rows) if rows else np.zeros((0, 8))
traj = load_tum(os.path.join(a.run_dir, "fused.tum"))
if not len(traj):
    print("no fused.tum"); sys.exit(1)
t0, t1 = traj[0, 0], traj[-1, 0]
print(f"traj: {len(traj)} poses, bag-time {t0:.1f}..{t1:.1f} ({t1-t0:.1f}s)")

# ---- submaps + edges ----
ids = [int(x) for x in open(os.path.join(MAP_DIR, "submaps.manifest"))]
subs, cents = {}, {}
for i in ids:
    p = load_smap(os.path.join(MAP_DIR, f"submap_{i}.smap"))
    subs[i] = p[::max(1, len(p) // 4000)] if len(p) else p
    cents[i] = p.mean(axis=0) if len(p) else np.zeros(3)
edges = []
ecsv = os.path.join(MAP_DIR, "anchor_edges.csv")
if os.path.exists(ecsv):
    for rr in csv.DictReader(open(ecsv)):
        f_, t_, ty = int(rr["from"]), int(rr["to"]), int(rr["type"])
        if f_ in cents and t_ in cents and f_ != t_:
            edges.append((f_, t_, ty))

# ---- events from log, wall-time -> bag-time (linear over the INFO span) ----
walls = []
ev = []  # (wall, kind, text, submap_id_or_-1)
for line in open(LOG, errors="ignore"):
    m = re.search(r"\[(\d{10}\.\d+)\]", line)
    w = float(m.group(1)) if m else None
    if w: walls.append(w)
    if "sealed submap" in line:
        mm = re.search(r"sealed submap (\d+).*?(\d+)->(\d+) lm", line)
        sid = int(mm.group(1)) if mm else -1
        ev.append((w, "SEAL", f"sealed submap {sid}" + (f" ({mm.group(2)}->{mm.group(3)} lm)" if mm else ""), sid))
    elif "LOOP CLOSED" in line:
        mm = re.search(r"LOOP CLOSED: kf (\d+) -> submap (\d+).*inliers=(\d+)", line)
        if mm:
            ev.append((w, "LOOP", f"LOOP CLOSED kf{mm.group(1)} -> sm{mm.group(2)} ({mm.group(3)} inl)", int(mm.group(2))))
    elif "loss" in line.lower() and ("seg" in line.lower() or "stale" in line.lower() or "branch" in line.lower()):
        ev.append((w, "LOSS", line.strip()[-70:], -1))
wall0, wall1 = (min(walls), max(walls)) if walls else (0, 1)
def wall2bag(w):
    if w is None or wall1 == wall0: return t0
    return t0 + (w - wall0) / (wall1 - wall0) * (t1 - t0)
ev = [(wall2bag(w), k, tx, sid) for (w, k, tx, sid) in ev if w]
ev.sort()
seal_time = {}
for (bt, k, tx, sid) in ev:
    if k == "SEAL" and sid >= 0 and sid not in seal_time:
        seal_time[sid] = bt
print(f"events: {len(ev)}  ({sum(k=='SEAL' for _,k,_,_ in ev)} seal, {sum(k=='LOOP' for _,k,_,_ in ev)} loop)")

# ---- world->pixel transform for the map panel ----
allp = np.vstack([subs[i][:, :2] for i in ids if len(subs[i])]) if any(len(subs[i]) for i in ids) else traj[:, 1:3]
xmin, ymin = allp.min(0) - 0.5; xmax, ymax = allp.max(0) + 0.5
MW, MH = 560, 760
sc = min((MW - 40) / (xmax - xmin), (MH - 40) / (ymax - ymin))
def w2p(x, y):
    px = int(20 + (x - xmin) * sc); py = int(MH - 20 - (y - ymin) * sc)
    return px, py
TAB = [(int(c[2]*255), int(c[1]*255), int(c[0]*255)) for c in
       __import__("matplotlib.cm", fromlist=["tab20"]).tab20(np.linspace(0, 1, 20))]

# ---- preload color frames (bag-time) ----
r = SequentialReader(); r.open(StorageOptions(uri=a.bag, storage_id='mcap'), ConverterOptions('cdr', 'cdr'))
r.set_filter(StorageFilter(topics=[a.color_topic]))
cframes = []  # (bag_t, msg-bytes)
while r.has_next():
    _, d, t = r.read_next(); cframes.append((t / 1e9, d))
del r
cft = np.array([c[0] for c in cframes])
# color frame stamps are absolute epoch; fused.tum is also epoch -> align by nearest
print(f"color frames: {len(cframes)} ({cft[0]:.1f}..{cft[-1]:.1f})")

ECOL = {0: ((136,136,136), 1, False), 1: ((14,127,255), 2, True), 2: ((44,160,44), 2, False)}

def draw_map(bt, pose_xy):
    img = np.full((MH, MW, 3), 28, np.uint8)
    sealed = [i for i in ids if seal_time.get(i, t0) <= bt]
    # submap points: gray if not sealed yet, colored if sealed
    for n, i in enumerate(ids):
        if not len(subs[i]): continue
        col = TAB[n % 20] if i in sealed else (70, 70, 70)
        for (x, y) in subs[i][:, :2]:
            px, py = w2p(x, y)
            if 0 <= px < MW and 0 <= py < MH: img[py, px] = col
    # edges that exist among sealed submaps
    for (f_, t_, ty) in edges:
        if f_ not in sealed or t_ not in sealed: continue
        col, w, dash = ECOL[ty]
        p1, p2 = w2p(*cents[f_][:2]), w2p(*cents[t_][:2])
        if dash:
            for s in range(0, 100, 14):
                x = p1[0] + (p2[0]-p1[0])*s/100; y = p1[1] + (p2[1]-p1[1])*s/100
                x2 = p1[0] + (p2[0]-p1[0])*min(100,s+7)/100; y2 = p1[1] + (p2[1]-p1[1])*min(100,s+7)/100
                cv2.line(img, (int(x),int(y)), (int(x2),int(y2)), col, w)
        else:
            cv2.line(img, p1, p2, col, w)
    # anchors
    for i in sealed:
        px, py = w2p(*cents[i][:2])
        cv2.drawMarker(img, (px, py), (255,255,255), cv2.MARKER_DIAMOND, 12, 2)
        cv2.putText(img, str(i), (px-4, py+4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1)
    # fused trail up to bt
    tr = traj[traj[:, 0] <= bt]
    for k in range(1, len(tr)):
        cv2.line(img, w2p(tr[k-1,1], tr[k-1,2]), w2p(tr[k,1], tr[k,2]), (0,200,255), 1)
    if pose_xy is not None:
        cv2.circle(img, w2p(*pose_xy), 5, (0,0,255), -1)
    cv2.putText(img, f"Atlas: {len(sealed)}/{len(ids)} submaps  t={bt-t0:5.1f}s", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.putText(img, "gray=unsealed  diamond=anchor  orange-dash=SOFT  green=HARD", (10, MH-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180,180,180), 1)
    return img

fast = cv2.FastFeatureDetector_create(threshold=22)
CW, CH = 560, 420
def draw_cam(bt):
    k = int(np.argmin(np.abs(cft - bt)))
    m = deserialize_message(cframes[k][1], Image)
    img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
    bgr = cv2.cvtColor(cv2.resize(img, (CW, CH)), cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    kps = fast.detect(gray, None)
    for kp in kps[:600]:
        cv2.circle(bgr, (int(kp.pt[0]), int(kp.pt[1])), 2, (0,255,0), -1)
    cv2.rectangle(bgr, (0,0), (CW,24), (0,0,0), -1)
    cv2.putText(bgr, f"camera + {len(kps)} corners   t={bt-t0:5.1f}s", (6,17),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
    return bgr

# ---- compose ----
H = CH + 120; W = CW + MW
vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), float(a.fps), (W, H))
n_out = int((t1 - t0) * a.fps)
for f in range(n_out):
    bt = t0 + (f / a.fps)
    tr = traj[traj[:, 0] <= bt]
    pose = (tr[-1, 1], tr[-1, 2]) if len(tr) else None
    frame = np.full((H, W, 3), 18, np.uint8)
    frame[0:CH, 0:CW] = draw_cam(bt)
    mp = draw_map(bt, pose)
    frame[0:min(MH,H), CW:CW+MW] = mp[0:min(MH,H)]
    # event ticker (last 4 events <= bt)
    past = [e for e in ev if e[0] <= bt][-4:]
    cv2.rectangle(frame, (0, CH), (CW, H), (0,0,0), -1)
    cv2.putText(frame, "EVENTS:", (6, CH+18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    for j, (bte, kind, tx, sid) in enumerate(past):
        col = {"SEAL":(200,200,0), "LOOP":(0,255,0), "LOSS":(0,128,255)}.get(kind, (200,200,200))
        cv2.putText(frame, f"{bte-t0:5.1f}s {tx}", (10, CH+38+j*20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)
    vw.write(frame)
    if f % 60 == 0: print(f"  frame {f}/{n_out}")
vw.release()
print(f"wrote {a.out}  ({n_out} frames @ {a.fps}fps)")
