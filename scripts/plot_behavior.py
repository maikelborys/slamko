#!/usr/bin/env python3
"""Interactive, REWINDABLE behavior viz (offline, Plotly) — the debugging standard
until system robustness is locked (no rviz/Foxglove/Pangolin infra needed).
Top-down 2D Atlas with a TIME SLIDER + play: scrub to any bag-time and see the
trajectory grown so far, the submaps sealed so far (colored; unsealed hidden),
the inter-submap edges (gray=odom, orange=SOFT/DR, green=HARD weld), the live
pose, and the latest event (seal / LOOP CLOSED / loss) in the title.

Usage: plot_behavior.py --run-dir results/run/x --out results/run/x/behavior.html [--step 1.0]
Sync: fused.tum is bag-time; log wall-time linearly mapped onto the fused span."""
import argparse, os, re, sys, csv
import numpy as np
import plotly.graph_objects as go
import matplotlib.cm as cm
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_neverlost import load_smap

ap = argparse.ArgumentParser()
ap.add_argument("--run-dir", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--step", type=float, default=1.0, help="bag-seconds per animation frame")
a = ap.parse_args()
MAP, LOG = os.path.join(a.run_dir, "map"), os.path.join(a.run_dir, "launch.log")

def load_tum(p):
    rows = [list(map(float, l.split())) for l in open(p) if l.strip() and not l.startswith("#")]
    return np.array(rows) if rows else np.zeros((0, 8))
traj = load_tum(os.path.join(a.run_dir, "fused.tum"))
t0, t1 = traj[0, 0], traj[-1, 0]

ids = [int(x) for x in open(os.path.join(MAP, "submaps.manifest"))]
subs, cents = {}, {}
for i in ids:
    p = load_smap(os.path.join(MAP, f"submap_{i}.smap"))
    subs[i] = p[::max(1, len(p) // 800)] if len(p) else p
    cents[i] = p.mean(axis=0) if len(p) else np.zeros(3)

edges = []
ec = os.path.join(MAP, "anchor_edges.csv")
if os.path.exists(ec):
    for rr in csv.DictReader(open(ec)):
        f_, t_, ty = int(rr["from"]), int(rr["to"]), int(rr["type"])
        if f_ in cents and t_ in cents and f_ != t_: edges.append((f_, t_, ty))

# events + wall->bag
walls, ev = [], []
for line in open(LOG, errors="ignore"):
    m = re.search(r"\[(\d{10}\.\d+)\]", line); w = float(m.group(1)) if m else None
    if w: walls.append(w)
    if "sealed submap" in line:
        mm = re.search(r"sealed submap (\d+)", line); ev.append((w, "SEAL", line.strip()[-60:], int(mm.group(1)) if mm else -1))
    elif "LOOP CLOSED" in line:
        mm = re.search(r"LOOP CLOSED: kf \d+ -> submap (\d+).*inliers=(\d+)", line)
        if mm: ev.append((w, "LOOP", f"LOOP CLOSED -> sm{mm.group(1)} ({mm.group(2)} inl)", int(mm.group(1))))
    elif re.search(r"\b(branch|dead.?reckon|soft edge|tracking lost|stale)\b", line, re.I):
        ev.append((w, "LOSS", line.strip()[-60:], -1))
w0, w1 = (min(walls), max(walls)) if walls else (0, 1)
b = lambda w: t0 if (w is None or w1 == w0) else t0 + (w - w0) / (w1 - w0) * (t1 - t0)
ev = sorted([(b(w), k, tx, sid) for (w, k, tx, sid) in ev if w])
seal_t = {}
for (bt, k, tx, sid) in ev:
    if k == "SEAL" and sid >= 0: seal_t.setdefault(sid, bt)

colors = cm.tab20(np.linspace(0, 1, 20))
rgb = lambda c: f"rgb({int(c[0]*255)},{int(c[1]*255)},{int(c[2]*255)})"
ESTYLE = {0: ("#888888", "solid"), 1: ("#ff7f0e", "dash"), 2: ("#2ca02c", "solid")}

# fixed traces: per-submap (N) + 3 edge-type + anchors + traj + pose
def at(t):
    sealed = [i for i in ids if seal_t.get(i, t0) <= t]
    data = []
    for n, i in enumerate(ids):
        p = subs[i]
        vis = (i in sealed) and len(p)
        data.append(go.Scatter(x=p[:, 0] if vis else [], y=p[:, 1] if vis else [],
                    mode="markers", marker=dict(size=2.5, color=rgb(colors[n % 20])),
                    name=f"sm{i}", showlegend=False, hoverinfo="name"))
    for ty in (0, 1, 2):
        xs, ys = [], []
        for (f_, t_, ety) in edges:
            if ety == ty and f_ in sealed and t_ in sealed:
                xs += [cents[f_][0], cents[t_][0], None]; ys += [cents[f_][1], cents[t_][1], None]
        col, dash = ESTYLE[ty]
        data.append(go.Scatter(x=xs, y=ys, mode="lines", line=dict(color=col, width=3, dash=dash),
                    name={0: "odom", 1: "SOFT", 2: "HARD"}[ty]))
    data.append(go.Scatter(x=[cents[i][0] for i in sealed], y=[cents[i][1] for i in sealed],
                mode="markers+text", marker=dict(size=11, color="white", symbol="diamond"),
                text=[str(i) for i in sealed], textposition="middle center",
                textfont=dict(size=8, color="black"), name="anchors", showlegend=False))
    tr = traj[traj[:, 0] <= t]
    data.append(go.Scatter(x=tr[:, 1], y=tr[:, 2], mode="lines", line=dict(color="#ffd700", width=1.5),
                name="trail", showlegend=False))
    px = ([tr[-1, 1]], [tr[-1, 2]]) if len(tr) else ([], [])
    data.append(go.Scatter(x=px[0], y=px[1], mode="markers", marker=dict(size=12, color="red"),
                name="pose", showlegend=False))
    return data

times = list(np.arange(t0, t1 + 1e-6, a.step))
def title_at(t):
    past = [e for e in ev if e[0] <= t]
    last = past[-1] if past else None
    sealed = sum(1 for i in ids if seal_t.get(i, t0) <= t)
    nl = sum(1 for e in past if e[1] == "LOOP")
    s = f"t={t-t0:5.1f}s | {sealed}/{len(ids)} submaps | {nl} loop-closes"
    if last: s += f" | last: {last[2]}"
    return s

frames = [go.Frame(data=at(t), name=f"{i}", layout=go.Layout(title=title_at(t))) for i, t in enumerate(times)]
fig = go.Figure(data=at(times[0]), frames=frames)
xr = [traj[:, 1].min() - 1, traj[:, 1].max() + 1]; yr = [traj[:, 2].min() - 1, traj[:, 2].max() + 1]
fig.update_layout(
    template="plotly_dark", title=title_at(times[0]),
    xaxis=dict(title="x [m]", range=xr, scaleanchor="y", scaleratio=1),
    yaxis=dict(title="y [m]", range=yr),
    updatemenus=[dict(type="buttons", x=0.0, y=1.12, buttons=[
        dict(label="▶ play", method="animate", args=[None, dict(frame=dict(duration=120, redraw=True), fromcurrent=True)]),
        dict(label="⏸ pause", method="animate", args=[[None], dict(mode="immediate", frame=dict(duration=0))])])],
    sliders=[dict(active=0, y=0, x=0.1, len=0.9, currentvalue=dict(prefix="frame "),
        steps=[dict(method="animate", label=f"{times[i]-t0:.0f}s",
                    args=[[f"{i}"], dict(mode="immediate", frame=dict(duration=0, redraw=True))])
               for i in range(len(times))])])
fig.write_html(a.out)
print(f"wrote {a.out}  ({len(ids)} submaps, {len(times)} frames, {len(ev)} events, "
      f"loops={sum(k=='LOOP' for _,k,_,_ in ev)})")
