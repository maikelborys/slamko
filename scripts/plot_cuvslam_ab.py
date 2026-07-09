#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Maikel Borys
#
# Plotly visuals for the cuVSLAM provider A/B (RESEARCH_CUVSLAM_OPENSOURCE_01):
#   1. escaleras_ab_3d.html  — OKVIS vs cuVSLAM-stereo vs cuVSLAM-Inertial, Sim3-aligned
#      into the OKVIS frame (rotatable; the stairs under-scale is visible by eye).
#   2. wall_health_timeline.html — the trusted-health story on the blank-wall bag:
#      speed (teleports) vs inliers vs info-condition vs covariance trace, SUSPECT shaded.
# Usage: ~/.venvs/cuvslam/bin/python scripts/plot_cuvslam_ab.py [out_dir]

import os
import sys

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    '~/coding/slamko/results/cuvslam_v16/viz')
os.makedirs(OUT, exist_ok=True)


def load(p):
    d = np.loadtxt(p)
    return d[:, 0], d[:, 1:4]


def umeyama(est, gt):
    mu_e, mu_g = est.mean(0), gt.mean(0)
    ec, gc = est - mu_e, gt - mu_g
    U, D, Vt = np.linalg.svd(gc.T @ ec / len(est))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / max((ec ** 2).sum() / len(est), 1e-12)
    return s, R, mu_g - s * R @ mu_e


def assoc(ta, tb, tol=0.02):
    j = 0
    pairs = []
    for i, te in enumerate(ta):
        while j + 1 < len(tb) and abs(tb[j + 1] - te) <= abs(tb[j] - te):
            j += 1
        if abs(tb[j] - te) <= tol:
            pairs.append((i, j))
    return pairs


# ---------------------------------------------------------------- 1. Escaleras 3D
def escaleras_3d():
    to, Po = load(os.path.expanduser('~/coding/slamko/results/ab_okvis_esc/fused.tum'))
    series = [('OKVIS VIO (referencia)', to, Po, '#2C7FB8', None)]
    for name, path, color in [
            ('cuVSLAM stereo-only (sub-escala 6.6%)', '/tmp/slamko_pa_cuvslam_esc/provider.tum', '#D95F02'),
            ('cuVSLAM Inertial (1.7% — GREEN)', '/tmp/slamko_pa_cuvslam_esc_vio/provider.tum', '#1B9E77')]:
        if not os.path.exists(path):
            continue
        t, P = load(path)
        pr = assoc(t, to)
        s, R, tr = umeyama(P[[p[0] for p in pr]], Po[[p[1] for p in pr]])
        series.append((name, t, (s * (R @ P.T)).T + tr, color, s))

    fig = go.Figure()
    for name, t, P, color, s in series:
        P = P[::4]  # downsample for size
        fig.add_trace(go.Scatter3d(
            x=P[:, 0], y=P[:, 1], z=P[:, 2], mode='lines',
            line=dict(color=color, width=4), name=name))
    fig.add_trace(go.Scatter3d(x=[series[0][2][0, 0]], y=[series[0][2][0, 1]],
                               z=[series[0][2][0, 2]], mode='markers',
                               marker=dict(size=6, color='black'), name='inicio'))
    fig.update_layout(
        title='Escaleras A/B — 2 pisos, 64 m (Sim3-aligned al frame OKVIS)',
        scene=dict(aspectmode='data', xaxis_title='x [m]', yaxis_title='y [m]',
                   zaxis_title='z [m]'),
        legend=dict(x=0.02, y=0.98), margin=dict(l=0, r=0, t=40, b=0))
    out = os.path.join(OUT, 'escaleras_ab_3d.html')
    fig.write_html(out, include_plotlyjs='cdn')
    print(out)


# ------------------------------------------------------- 2. Wall health timeline
def wall_health():
    import csv
    d = os.path.expanduser(
        '~/coding/slamko/results/cuvslam_v16/CASA1_wall_Stereo60_RGB30_BNO_trim_stereo')
    rows = [r for r in csv.DictReader(open(f'{d}/cov_trace.csv'))]
    t = np.array([float(r['t']) for r in rows])
    t0 = t[0]
    t = t - t0
    trace = np.array([float(r['trace']) if r['trace'] else np.nan for r in rows])
    inl = np.array([float(r['inliers']) if r['inliers'] else np.nan for r in rows])
    cond = np.array([float(r['info_condition']) if r['info_condition'] else np.nan
                     for r in rows])
    traj = np.loadtxt(f'{d}/traj.tum')
    tt, P = traj[:, 0] - t0, traj[:, 1:4]
    v = np.linalg.norm(np.diff(P, axis=0), axis=1) / np.maximum(np.diff(tt), 1e-6)

    fig = make_subplots(
        rows=4, cols=1, shared_xaxes=True, vertical_spacing=0.04,
        subplot_titles=('velocidad |Δp|/Δt — los teleports que vo_state NUNCA marcó',
                        'inliers PnP (fork trusted-health) — gate ≥ 10',
                        'condición de la matriz de información — gate ≤ 1e6',
                        'traza de covarianza publicada (inflada ×25 en SUSPECT)'))
    fig.add_trace(go.Scatter(x=tt[1:], y=np.maximum(v, 1e-3), mode='lines',
                             line=dict(color='#D95F02', width=1), name='v [m/s]'), 1, 1)
    fig.add_hline(y=5.0, line_dash='dash', line_color='red', row=1, col=1,
                  annotation_text='gate físico 5 m/s')
    fig.add_trace(go.Scatter(x=t, y=inl, mode='lines',
                             line=dict(color='#1B9E77', width=1), name='inliers'), 2, 1)
    fig.add_hline(y=10, line_dash='dash', line_color='red', row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=cond, mode='lines',
                             line=dict(color='#7570B3', width=1), name='cond(H)'), 3, 1)
    fig.add_hline(y=1e6, line_dash='dash', line_color='red', row=3, col=1)
    fig.add_trace(go.Scatter(x=t, y=trace, mode='lines',
                             line=dict(color='#E7298A', width=1), name='tr(cov)'), 4, 1)
    fig.update_yaxes(type='log', row=1, col=1)
    fig.update_yaxes(type='log', row=3, col=1)
    fig.update_yaxes(type='log', row=4, col=1)
    fig.update_xaxes(title_text='t [s] del bag', row=4, col=1)
    fig.update_layout(
        title='CASA1_wall (pared en blanco + sacudidas) — la evidencia interna separa '
              'los teleports que el flag de éxito dejó pasar',
        height=900, showlegend=False, margin=dict(l=60, r=20, t=80, b=40))
    out = os.path.join(OUT, 'wall_health_timeline.html')
    fig.write_html(out, include_plotlyjs='cdn')
    print(out)


escaleras_3d()
wall_health()
