#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Maikel Borys
#
# Report for the Atlas multi-session EuRoC test (atlas_multisession_euroc.sh):
# per-session ATE (own Sim3, sanity) + the ORB-SLAM3 metric — ONE joint Sim3 from the
# merged multi-session map to the concatenated GT (all MH GTs share the Leica frame).
# A session is WELDED if its residual under the JOINT transform is coherent
# (< weld_thresh RMSE); DANGLING sessions are reported honestly, never force-fitted.
# Also emits a rotatable plotly: sessions colored, GT in gray.
#   ~/.venvs/cuvslam/bin/python atlas_multisession_report.py <out_root>

import glob
import os
import sys

import numpy as np

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    '~/coding/slamko/results/atlas_ms')
EUROC = '/mnt/data/datasets/euroc'
WELD_RMSE_M = 0.5


def load_tum(p):
    d = np.loadtxt(p)
    return d[:, 0], d[:, 1:4]


def umeyama(est, gt, with_scale=True):
    mu_e, mu_g = est.mean(0), gt.mean(0)
    ec, gc = est - mu_e, gt - mu_g
    U, D, Vt = np.linalg.svd(gc.T @ ec / len(est))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = (np.trace(np.diag(D) @ S) / max((ec ** 2).sum() / len(est), 1e-12)
         if with_scale else 1.0)
    t = mu_g - s * R @ mu_e
    return s, R, t


def apply(T, P):
    s, R, t = T
    return (s * (R @ P.T)).T + t


def assoc(ta, tb, tol=0.02):
    j = 0
    out = []
    for i, te in enumerate(ta):
        while j + 1 < len(tb) and abs(tb[j + 1] - te) <= abs(tb[j] - te):
            j += 1
        if abs(tb[j] - te) <= tol:
            out.append((i, j))
    return out


sessions = sorted(glob.glob(os.path.join(ROOT, 's[0-9]_*')))
data = []
for sdir in sessions:
    seq = os.path.basename(sdir).split('_', 1)[1]
    gtum = os.path.join(sdir, 'graph.tum')
    gt = os.path.join(EUROC, seq, 'mav0', 'state_groundtruth_estimate0', 'data_tum.txt')
    if not (os.path.exists(gtum) and os.path.getsize(gtum) and os.path.exists(gt)):
        print(f'{seq}: MISSING graph.tum — skipped')
        continue
    te, Pe = load_tum(gtum)
    tg, Pg = load_tum(gt)
    pairs = assoc(te, tg)
    ie = [p[0] for p in pairs]
    ig = [p[1] for p in pairs]
    T = umeyama(Pe[ie], Pg[ig])
    err = np.linalg.norm(apply(T, Pe[ie]) - Pg[ig], axis=1)
    ate = float(np.sqrt((err ** 2).mean()))
    data.append(dict(seq=seq, te=te, Pe=Pe, ie=ie, ig=ig, Pg=Pg, own_ate=ate))
    print(f'{seq}: {len(Pe)} graph kf, own-Sim3 ATE {ate*100:.1f} cm '
          f'(scale {T[0]:.4f})')

# ---- ORB-SLAM3 metric: ONE joint Sim3 over ALL sessions' matched pairs
E = np.vstack([d['Pe'][d['ie']] for d in data])
G = np.vstack([d['Pg'][d['ig']] for d in data])
TJ = umeyama(E, G)
print(f'\nJOINT single Sim3 (all sessions together): scale={TJ[0]:.4f}')
joint_res = []
welded = []
for d in data:
    err = np.linalg.norm(apply(TJ, d['Pe'][d['ie']]) - d['Pg'][d['ig']], axis=1)
    rmse = float(np.sqrt((err ** 2).mean()))
    d['joint_rmse'] = rmse
    tag = 'WELDED' if rmse < WELD_RMSE_M else 'DANGLING'
    welded.append(rmse < WELD_RMSE_M)
    print(f'  {d["seq"]}: joint-frame RMSE {rmse*100:.1f} cm -> {tag}')
    joint_res.append(err)
if sum(welded) >= 2:
    wa = np.concatenate([e for e, w in zip(joint_res, welded) if w])
    print(f'\nMULTI-SESSION ATE over the {sum(welded)} welded sessions: '
          f'{np.sqrt((wa**2).mean())*100:.1f} cm RMSE / max {wa.max()*100:.1f} cm '
          f'(ORB-SLAM3 Atlas MH multi-session reference: ~3-8 cm)')
else:
    print('\n< 2 sessions welded — no multi-session map formed (honest dangling).')

# ---- plotly
import plotly.graph_objects as go
colors = ['#1B9E77', '#D95F02', '#7570B3', '#E7298A', '#66A61E']
fig = go.Figure()
for k, d in enumerate(data):
    P = apply(TJ, d['Pe'])[::3]
    fig.add_trace(go.Scatter3d(x=P[:, 0], y=P[:, 1], z=P[:, 2], mode='lines',
                               line=dict(color=colors[k % 5], width=4),
                               name=f'{d["seq"]} ({d["joint_rmse"]*100:.0f} cm '
                                    f'{"WELD" if d["joint_rmse"]<WELD_RMSE_M else "DANGLE"})'))
    Pg = d['Pg'][d['ig']][::6]
    fig.add_trace(go.Scatter3d(x=Pg[:, 0], y=Pg[:, 1], z=Pg[:, 2], mode='lines',
                               line=dict(color='gray', width=1), showlegend=(k == 0),
                               name='GT (Leica)'))
fig.update_layout(title='Atlas multi-session EuRoC MH — mapa fusionado en el frame GT '
                        '(un solo Sim3 conjunto)',
                  scene=dict(aspectmode='data'), margin=dict(l=0, r=0, t=40, b=0))
out = os.path.join(ROOT, 'atlas_multisession_3d.html')
fig.write_html(out, include_plotlyjs='cdn')
print('\nplotly ->', out)
