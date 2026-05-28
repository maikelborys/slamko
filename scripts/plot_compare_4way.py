#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Multi-trajectory Plotly comparator. Plots GT + N estimates in the SAME 3D scene,
# each translated so its first frame is at origin (no Sim3 alignment to GT — for
# magistrale1 GT is room-only, so a Sim3 fit is degenerate). Shows where each
# system's loop closure lands at the end of the traversal (the honest metric for
# this dataset: end-displacement vs GT's 25 cm).

import argparse
import sys
import numpy as np
import plotly.graph_objects as go

sys.path.insert(0, '/home/maikel/coding/slamko/scripts')
from plot_neverlost import load_tum


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--title', default='Trajectory comparison')
    # Each --traj is "name:tum_path:color"
    ap.add_argument('--traj', action='append', required=True,
                    help='name:path:hexcolor (repeatable)')
    a = ap.parse_args()

    fig = go.Figure()

    print(f'{"name":<15} {"frames":>7} {"span_s":>8} {"end_disp_m":>11} {"end_pose":>30}')
    for spec in a.traj:
        name, path, color = spec.split(':', 2)
        try:
            te, xe = load_tum(path)
        except Exception as e:
            print(f'  skip {name}: {e}')
            continue
        # Translate so frame 0 == origin (no rotation alignment — keeps each
        # system's own gravity / forward axis, the honest visual).
        xe = xe - xe[0]
        end_disp = np.linalg.norm(xe[-1] - xe[0])
        print(f'{name:<15} {len(xe):>7} {te[-1]-te[0]:>8.1f} {end_disp:>11.3f}  '
              f'[{xe[-1,0]:>7.2f},{xe[-1,1]:>7.2f},{xe[-1,2]:>7.2f}]')
        fig.add_trace(go.Scatter3d(
            x=xe[:, 0], y=xe[:, 1], z=xe[:, 2], mode='lines',
            name=f'{name} (end Δ={end_disp:.2f} m, {len(xe)} pts)',
            line=dict(color=color, width=3)))
        # Marker at end pose
        fig.add_trace(go.Scatter3d(
            x=[xe[-1, 0]], y=[xe[-1, 1]], z=[xe[-1, 2]], mode='markers',
            marker=dict(color=color, size=8, symbol='x'),
            name=f'{name} end', showlegend=False))

    fig.update_layout(
        title=a.title,
        scene=dict(aspectmode='data', xaxis_title='x [m]', yaxis_title='y [m]',
                   zaxis_title='z [m]'),
        margin=dict(l=0, r=0, b=0, t=40), showlegend=True)
    fig.write_html(a.out)
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
