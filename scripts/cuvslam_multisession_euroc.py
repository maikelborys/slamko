#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Maikel Borys
#
# THE DECISIVE EXPERIMENT (RESEARCH_ATLAS_MULTISESSION_01 follow-up): can cuVSLAM's
# NATIVE multi-session machinery (Slam + SaveMap + LocalizeInMap + continue-mapping,
# v16 "SLAM jump after map load" fixed) pass the ORB-SLAM3 EuRoC MH multi-session
# test where slamko's sparse-descriptor welds hit the viewpoint wall (1 m coherence)?
# Offline via PyCuVSLAM — no ROS. Chain: MH01 (map) -> save -> MH03 (localize with a
# GT-derived guess + continue in the SAME map) -> save -> MH05 (same).
# Metric: ONE joint Sim3 of all sessions' slam poses vs the concatenated Leica GT.
#
#   LD_LIBRARY_PATH=~/.venvs/cuvslam/lib/python3.12/site-packages/cuvslam \
#     ~/.venvs/cuvslam/bin/python scripts/cuvslam_multisession_euroc.py
#
# Guess pose: cuVSLAM world-1 = cam0 pose at MH01's first frame, so the MH0X start
# guess in that frame is T_cam0(t0_MH01)^-1 * T_cam0(t0_MH0X), computed from GT
# (T_gt_body * T_BS_cam0) — same basis on both sides, no OpenCV/ROS conversion needed.

import os
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.expanduser('~/coding/cuVSLAM_src/examples/euroc'))
import cuvslam  # noqa: E402
from dataset_utils import get_rig, load_frame, prepare_frame_metadata_euroc  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

EUROC = '/mnt/data/datasets/euroc'
SEQS = [s for s in os.environ.get('MS_SEQS', 'MH_01_easy MH_03_medium MH_05_difficult').split()]
OUT = os.path.expanduser('~/coding/slamko/results/cuvslam_native_ms')
os.makedirs(OUT, exist_ok=True)
MAP_DIR = os.path.join(OUT, 'map_db')


def gt_first_cam0_pose(seq):
    """cam0 pose (4x4, Leica frame) at the sequence's first GT timestamp."""
    root = f'{EUROC}/{seq}/mav0'
    g = np.loadtxt(f'{root}/state_groundtruth_estimate0/data_tum.txt')
    t0, p, q = g[0, 0], g[0, 1:4], g[0, 4:8]  # x y z qx qy qz qw
    T_wb = np.eye(4)
    T_wb[:3, :3] = Rotation.from_quat(q).as_matrix()
    T_wb[:3, 3] = p
    T_bc = np.array(yaml.safe_load(open(f'{root}/cam0/sensor.yaml'))['T_BS']['data']).reshape(4, 4)
    return t0, T_wb @ T_bc


def make_guess_fn(seq, T_cam0_ref):
    """GT-derived cam0-pose guess in the MH01-start (map) frame, at any time t."""
    root = f'{EUROC}/{seq}/mav0'
    g = np.loadtxt(f'{root}/state_groundtruth_estimate0/data_tum.txt')
    T_bc = np.array(yaml.safe_load(open(f'{root}/cam0/sensor.yaml'))['T_BS']['data']).reshape(4, 4)
    Tref_inv = np.linalg.inv(T_cam0_ref)

    def fn(t_s):
        i = int(np.argmin(np.abs(g[:, 0] - t_s)))
        if abs(g[i, 0] - t_s) > 0.05:
            return None
        T_wb = np.eye(4)
        T_wb[:3, :3] = Rotation.from_quat(g[i, 4:8]).as_matrix()
        T_wb[:3, 3] = g[i, 1:4]
        return Tref_inv @ T_wb @ T_bc
    return fn


def pose_from_T(T):
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    return cuvslam.Pose(rotation=q, translation=T[:3, 3])


def run_session(seq, tracker_holder, localize_from=None):
    seq_dir = f'{EUROC}/{seq}/mav0'
    rig = get_rig(seq_dir)
    odom_cfg = cuvslam.Tracker.OdometryConfig(
        async_sba=False, rectified_stereo_camera=False,
        odometry_mode=cuvslam.Tracker.OdometryMode.Multicamera)
    slam_cfg = cuvslam.Tracker.SlamConfig()
    slam_cfg.sync_mode = True  # deterministic: everything on the caller thread
    slam_cfg.enable_reading_internals = True
    slam_cfg.max_map_size = 5000
    tracker = cuvslam.Tracker(rig, odom_cfg, slam_cfg)
    tracker_holder['t'] = tracker

    frames = [f for f in prepare_frame_metadata_euroc(
        seq_dir, odom_cfg.odometry_mode) if f['type'] == 'stereo']

    # LIVE-style flow: TRACK continuously; while not yet localized, attempt
    # localize_in_map every ~300 frames with the CURRENT image (so the tail/odometry
    # state matches the localization stitch — localizing at t while the tail starts
    # elsewhere mis-binds the graph: measured 3.2 m on MH05).
    st = cuvslam.Tracker.SlamLocalizationSettings(
        horizontal_search_radius=4.0, vertical_search_radius=2.0,
        horizontal_step=0.5, vertical_step=0.5, angular_step_rads=0.25)
    prior_map, guess_fn = localize_from if localize_from else (None, None)
    localized = localize_from is None
    n_lost = 0
    for fi, fm in enumerate(frames):
        images = [load_frame(p) for p in fm['images_paths']]
        est, _ = tracker.track(int(fm['timestamp']), images)
        if est.world_from_rig is None:
            n_lost += 1
        if not localized and fi % 300 == 0:
            T_guess = guess_fn(fm['timestamp'] * 1e-9)
            if T_guess is None:
                continue
            result = {}

            def fin(pose, err):
                result['pose'] = pose
                result['err'] = err
            tracker.localize_in_map(prior_map, int(fm['timestamp']),
                                    pose_from_T(T_guess), images, st,
                                    lambda: None, fin)
            for _ in range(600):
                if 'pose' in result or result.get('err'):
                    break
                time.sleep(0.5)
            localized = result.get('pose') is not None
            if localized:
                try:
                    pg0 = tracker.get_pose_graph()
                    tracker_holder['prior_max_id'] = max(n.id for n in pg0.nodes) if pg0 and pg0.nodes else -1
                except Exception:
                    tracker_holder['prior_max_id'] = -1
            print(f'[{seq}] localize@frame {fi}: '
                  f'{"OK" if localized else "FAIL"} err={result.get("err", "")!r}',
                  flush=True)
    if not localized:
        print(f'[{seq}] NEVER localized -> honest dangling island', flush=True)
        return None, False
    poses = tracker.get_all_slam_poses(0)
    try:
        m = tracker.get_slam_metrics()
        print(f'[{seq}] METRICS: {m}', flush=True)
        pg = tracker.get_pose_graph()
        if pg is not None and pg.nodes:
            boundary = tracker_holder.get('prev_final_max_id', -1)
            tracker_holder['prev_final_max_id'] = max(n.id for n in pg.nodes)
            cross = sum(1 for e in pg.edges
                        if (e.node_from <= boundary) != (e.node_to <= boundary))
            print(f'[{seq}] POSEGRAPH: {len(pg.nodes)} nodes {len(pg.edges)} edges | '
                  f'prior_boundary={boundary} CROSS-SESSION edges={cross}', flush=True)
    except Exception as ex:
        print(f'[{seq}] metrics err: {ex}', flush=True)
    print(f'[{seq}] tracked {len(frames)} frames (lost {n_lost}), '
          f'slam graph poses: {len(poses)}', flush=True)

    # TUM dump of the CURRENT full graph (includes prior sessions after a merge).
    with open(os.path.join(OUT, f'after_{seq}.tum'), 'w') as f:
        for ps in poses:
            p, q = ps.pose.translation, ps.pose.rotation
            f.write(f'{ps.timestamp_ns * 1e-9:.9f} {p[0]} {p[1]} {p[2]} '
                    f'{q[0]} {q[1]} {q[2]} {q[3]}\n')

    saved = {}
    tracker.save_map(MAP_DIR, lambda okk: saved.update(ok=okk))
    for _ in range(600):
        if 'ok' in saved:
            break
        time.sleep(0.5)
    print(f'[{seq}] save_map -> {saved.get("ok")}', flush=True)
    return poses, bool(saved.get('ok'))


def main():
    cuvslam.set_verbosity(0)
    cuvslam.warm_up_gpu()
    t0_ref, T_cam0_ref = gt_first_cam0_pose(SEQS[0])

    holder = {}
    run_session(SEQS[0], holder)

    for seq in SEQS[1:]:
        guess_fn = make_guess_fn(seq, T_cam0_ref)
        run_session(seq, holder, localize_from=(MAP_DIR, guess_fn))

    print('DONE ->', OUT, flush=True)


if __name__ == '__main__':
    main()
