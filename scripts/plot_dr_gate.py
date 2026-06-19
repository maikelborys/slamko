#!/usr/bin/env python3
"""R0.1 DR-gate visualization — does OKVIS's across-gap motion agree with an
INDEPENDENT dead-reckoning estimate (gyro-integrated rotation + last-velocity
coasting)? Big disagreement => OKVIS's IMU-bridge is suspect => R0 gate fires.

Aggregates every results/**/dr_gate.csv and plots disagreement vs gap length:
  - d_rot (deg): gyro-vs-OKVIS rotation disagreement — the TRUSTWORTHY channel
    (gyro is accurate over short gaps). This is what the R0 gate should key on.
  - d_trans (m): coast-vs-OKVIS translation disagreement — crude (constant-velocity
    breaks on a curving path), shown for context, NOT a gate signal on its own.

Usage:  python3 scripts/plot_dr_gate.py results/r01  [out.html]
Writes <out.html> + a <out.png> screenshot next to it.
"""
import sys, glob, csv, os
import plotly.graph_objects as go
from plotly.subplots import make_subplots

root = sys.argv[1] if len(sys.argv) > 1 else "results/r01"
out_html = sys.argv[2] if len(sys.argv) > 2 else os.path.join(root, "dr_gate.html")

rows = []
for f in sorted(glob.glob(os.path.join(root, "**", "dr_gate.csv"), recursive=True)):
    run = os.path.basename(os.path.dirname(f))
    with open(f) as fh:
        for r in csv.DictReader(fh):
            r["run"] = run
            rows.append(r)

if not rows:
    print(f"no dr_gate.csv rows under {root}")
    sys.exit(1)

gap = [float(r["gap_s"]) for r in rows]
drot = [float(r["d_rot_deg"]) for r in rows]
dtr = [float(r["d_trans_m"]) for r in rows]
labels = [f'{r["run"]} @t={float(r["rel_t"]):.0f}s' for r in rows]

# Proposed gate threshold (rotation): gyro drift on a MEMS IMU is ~1-2 deg/s of
# integration; a TRUSTWORTHY OKVIS bridge stays within that band. Flag above ~3x.
ROT_GATE = 15.0  # deg — provisional; tighten once a broken-bridge negative exists

fig = make_subplots(rows=1, cols=2, subplot_titles=(
    "Rotation disagreement (gyro vs OKVIS) — the gate channel",
    "Translation disagreement (coast vs OKVIS) — context only"))
fig.add_trace(go.Scatter(x=gap, y=drot, mode="markers+text", text=labels,
              textposition="top center", marker=dict(size=12, color="crimson"),
              name="d_rot"), row=1, col=1)
fig.add_hline(y=ROT_GATE, line_dash="dash", line_color="black",
              annotation_text=f"provisional gate {ROT_GATE:.0f}deg", row=1, col=1)
fig.add_trace(go.Scatter(x=gap, y=dtr, mode="markers+text", text=labels,
              textposition="top center", marker=dict(size=12, color="darkorange"),
              name="d_trans"), row=1, col=2)
fig.update_xaxes(title_text="gap length [s]", row=1, col=1)
fig.update_xaxes(title_text="gap length [s]", row=1, col=2)
fig.update_yaxes(title_text="d_rot [deg]", row=1, col=1)
fig.update_yaxes(title_text="d_trans [m]", row=1, col=2)
fig.update_layout(title_text="R0.1 DR-gate: OKVIS across-gap motion vs independent DR",
                  showlegend=False, width=1100, height=520)
fig.write_html(out_html)
print(f"wrote {out_html}  ({len(rows)} gaps)")
try:
    png = out_html.replace(".html", ".png")
    fig.write_image(png, scale=2)
    print(f"wrote {png}")
except Exception as e:
    print(f"(no PNG — kaleido missing? {e})")
