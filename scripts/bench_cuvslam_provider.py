#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Maikel Borys
#
# bench_cuvslam_provider.py — provider-level benchmark of cuVSLAM v16+ (open-source
# PyCuVSLAM) against the slamko immortal-provider criteria. This is the R1 harness of
# docs/RESEARCH_CUVSLAM_OPENSOURCE_01.md: it measures the FOUR channels slamko cares
# about in a provider, offline and deterministically (no ROS, no GPU contention races):
#
#   1. accuracy   — ATE RMSE after Umeyama Sim(3) alignment vs GT (EuRoC only), so the
#                   numbers are apples-to-apples with the old binary baseline
#                   (~/coding/cuvslam/CLAUDE.md: stereo 0.023/0.040/0.057 on MH_01/03/05).
#   2. never-jump — teleport events: per-frame velocity above a physical bound
#                   (default 5 m/s, >> handheld/robot speeds; same gate as the old
#                   detect_tracking_loss.py so casa numbers are comparable: 10 brutal /
#                   17 wall on the closed 4.4 binary).
#   3. trust      — covariance regime census. The source study showed the exposed 6x6 is
#                   contaminated by Identity fallbacks (still frames, multicam_pnp.cpp:219)
#                   and Zero-info inversions (empty map). We classify every frame:
#                   zero / identity / constant / varying, and compute translation-block
#                   NEES vs GT where GT exists — the §7-A calibration prerequisite.
#   4. realtime   — track() wall-time p50/p99 + effective fps (the OKVIS 31-fps-ceiling
#                   motive, memory slamko-okvis-realtime-ceiling).
#
# Runs the NVIDIA example loaders at RUNTIME from ~/coding/cuVSLAM_src (sys.path import —
# never vendored: NVIDIA Community License code must not enter this Apache-2.0 repo).
#
# Usage (venv ~/.venvs/cuvslam; the wrapper exports the bundled-lib LD_LIBRARY_PATH —
# REQUIRED or the ROS isaac_ros 4.4 libcuvslam.so shadows the wheel and import fails):
#   scripts/bench_cuvslam_provider.sh euroc --seq MH_03_medium --mode stereo
#   scripts/bench_cuvslam_provider.sh euroc --seq MH_03_medium --mode vio
#   scripts/bench_cuvslam_provider.sh bag --bag /mnt/data/bags/bno_ab/CASA1_brutal1_..._trim
# Outputs <out>/<name>/: traj.tum, metrics.json, cov_trace.csv — and prints the scorecard.

import argparse
import csv
import json
import os
import sys
import time

import numpy as np

CUVSLAM_SRC = os.path.expanduser('~/coding/cuVSLAM_src')
sys.path.insert(0, os.path.join(CUVSLAM_SRC, 'examples', 'euroc'))

import cuvslam  # noqa: E402
from dataset_utils import get_rig, load_frame, prepare_frame_metadata_euroc  # noqa: E402

EUROC_ROOT = '/mnt/data/datasets/euroc'
DEFAULT_OUT = os.path.expanduser('~/coding/slamko/results/cuvslam_v16')
TELEPORT_MPS = 5.0  # same physical gate as the legacy detect_tracking_loss.py


# ---------------------------------------------------------------- trajectory metrics

def umeyama_sim3(est: np.ndarray, gt: np.ndarray):
    """Umeyama alignment with scale (Sim3), est/gt: Nx3. Returns aligned est."""
    mu_e, mu_g = est.mean(0), gt.mean(0)
    ec, gc = est - mu_e, gt - mu_g
    cov = gc.T @ ec / len(est)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_e = (ec ** 2).sum() / len(est)
    s = np.trace(np.diag(D) @ S) / var_e if var_e > 0 else 1.0
    t = mu_g - s * R @ mu_e
    return (s * (R @ est.T)).T + t, s, R, t


def associate(t_est, t_gt, max_dt=0.02):
    """Nearest-timestamp association. Returns index pairs (i_est, i_gt)."""
    j = 0
    pairs = []
    for i, te in enumerate(t_est):
        while j + 1 < len(t_gt) and abs(t_gt[j + 1] - te) <= abs(t_gt[j] - te):
            j += 1
        if abs(t_gt[j] - te) <= max_dt:
            pairs.append((i, j))
    return pairs


def load_tum(path):
    d = np.loadtxt(path)
    return d[:, 0], d[:, 1:4]


# ---------------------------------------------------------------- covariance census

def classify_cov(cov: np.ndarray):
    """One frame's 6x6 (rotation-first, tangent-space) -> regime label."""
    if cov is None:
        return 'none'
    c = np.asarray(cov, dtype=np.float64).reshape(6, 6)
    if np.abs(c).max() < 1e-12:
        return 'zero'
    if np.allclose(c, np.eye(6), atol=1e-9):
        return 'identity'
    return 'varying'


# ---------------------------------------------------------------- runners

def make_tracker(rig, mode: str, async_sba: bool, health: bool = False):
    m = (cuvslam.Tracker.OdometryMode.Inertial if mode == 'vio'
         else cuvslam.Tracker.OdometryMode.Multicamera)
    cfg = cuvslam.Tracker.OdometryConfig(
        async_sba=async_sba,
        enable_observations_export=health,  # stat/state export needed for pnp_health
        enable_final_landmarks_export=False,
        rectified_stereo_camera=False,
        odometry_mode=m)
    return cuvslam.Tracker(rig, cfg)


def grab_health(tracker):
    """slamko/trusted-health fork only (branch slamko/trusted-health of cuVSLAM_src);
    returns None on the stock wheel."""
    try:
        h = tracker.odom.get_state().pnp_health
        return {'obs': h.observations, 'inliers': h.inliers,
                'mean_residual': h.mean_residual, 'final_cost': h.final_cost,
                'info_condition': h.info_condition}
    except Exception:
        return None


def run_frames(tracker, frames, mode: str, imu_scale: float = 1.0,
               health: bool = False):
    """Common tracking loop. frames = iterable of dicts (type stereo/imu).
    Returns per-frame records."""
    recs = []
    n_fail = 0
    for fm in frames:
        if fm['type'] == 'imu':
            m = cuvslam.ImuMeasurement()
            m.timestamp_ns = int(fm['timestamp'])
            m.linear_accelerations = np.asarray(fm['accel']) * imu_scale
            m.angular_velocities = np.asarray(fm['gyro'])
            tracker.register_imu_measurement(0, m)
            continue
        if 'images_paths' in fm:
            images = [load_frame(p) for p in fm['images_paths']]
        else:
            images = fm['images']
        t0 = time.perf_counter()
        est, _ = tracker.track(int(fm['timestamp']), images)
        dt_track = time.perf_counter() - t0
        h = grab_health(tracker) if health else None
        if est.world_from_rig is None:
            n_fail += 1
            recs.append({'t': fm['timestamp'] * 1e-9, 'ok': False, 'health': h,
                         'track_s': dt_track, 'pos': None, 'quat': None, 'cov': None})
            continue
        p = est.world_from_rig.pose
        recs.append({'t': fm['timestamp'] * 1e-9, 'ok': True, 'track_s': dt_track,
                     'pos': np.asarray(p.translation, dtype=np.float64),
                     'quat': np.asarray(p.rotation, dtype=np.float64),
                     'cov': np.asarray(est.world_from_rig.covariance, dtype=np.float64),
                     'health': h})
    return recs, n_fail


def euroc_frames(seq_dir: str, mode: str):
    m = (cuvslam.Tracker.OdometryMode.Inertial if mode == 'vio'
         else cuvslam.Tracker.OdometryMode.Multicamera)
    return prepare_frame_metadata_euroc(seq_dir, m)


def bag_frames(bag_path: str, left_topic: str, right_topic: str, imu_topic: str,
               mode: str):
    """Stream synced stereo (+optional IMU) from a rosbag2. Pairs left/right by
    identical header stamp (D455 IR frames share the stamp)."""
    from rosbags.highlevel import AnyReader
    from pathlib import Path
    want = {left_topic, right_topic} | ({imu_topic} if mode == 'vio' else set())
    pend = {}
    with AnyReader([Path(bag_path)]) as reader:
        conns = [c for c in reader.connections if c.topic in want]
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == imu_topic:
                ts = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                yield {'type': 'imu', 'timestamp': ts,
                       'accel': [msg.linear_acceleration.x, msg.linear_acceleration.y,
                                 msg.linear_acceleration.z],
                       'gyro': [msg.angular_velocity.x, msg.angular_velocity.y,
                                msg.angular_velocity.z]}
                continue
            ts = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            side = 0 if conn.topic == left_topic else 1
            if ts in pend:
                pair = pend.pop(ts)
                pair[side] = img
                if pair[0] is not None and pair[1] is not None:
                    yield {'type': 'stereo', 'timestamp': ts, 'images': [pair[0], pair[1]]}
            else:
                pair = [None, None]
                pair[side] = img
                pend[ts] = pair


def d455_rig_from_bag(bag_path: str, left_info: str, right_info: str):
    """Build the stereo rig from the bag's own camera_info (pinhole rectified IR)."""
    from rosbags.highlevel import AnyReader
    from pathlib import Path
    infos = {}
    with AnyReader([Path(bag_path)]) as reader:
        conns = [c for c in reader.connections if c.topic in (left_info, right_info)]
        for conn, _, raw in reader.messages(connections=conns):
            if conn.topic not in infos:
                infos[conn.topic] = reader.deserialize(raw, conn.msgtype)
            if len(infos) == 2:
                break
    li, ri = infos[left_info], infos[right_info]
    baseline = -ri.p[3] / ri.p[0]  # P[3] = -fx*B on the right camera

    def cam(info, tx=0.0):
        c = cuvslam.Camera()
        c.focal = [info.k[0], info.k[4]]
        c.principal = [info.k[2], info.k[5]]
        c.size = [info.width, info.height]
        c.distortion = cuvslam.Distortion(cuvslam.Distortion.Model.Pinhole, [])
        c.rig_from_camera = cuvslam.Pose(rotation=[0, 0, 0, 1], translation=[tx, 0, 0])
        return c

    rig = cuvslam.Rig()
    rig.cameras = [cam(li), cam(ri, baseline)]
    rig.imus = []
    return rig, baseline


# ---------------------------------------------------------------- metrics + report

def analyze(recs, gt_tum=None, name='run'):
    ok = [r for r in recs if r['ok']]
    out = {'name': name, 'frames': len(recs), 'tracked': len(ok),
           'lost_frames': len(recs) - len(ok)}
    if len(ok) < 10:
        out['error'] = 'too few tracked frames'
        return out

    # realtime channel
    ts = np.array([r['track_s'] for r in recs])
    out['track_ms_p50'] = round(float(np.percentile(ts, 50)) * 1e3, 3)
    out['track_ms_p99'] = round(float(np.percentile(ts, 99)) * 1e3, 3)
    out['fps_compute_bound'] = round(1.0 / max(ts.mean(), 1e-9), 1)

    # never-jump channel
    t = np.array([r['t'] for r in ok])
    P = np.stack([r['pos'] for r in ok])
    dt = np.diff(t)
    v = np.linalg.norm(np.diff(P, axis=0), axis=1) / np.maximum(dt, 1e-6)
    tele_idx = np.where(v > TELEPORT_MPS)[0]
    out['teleport_events'] = int(len(tele_idx))
    out['max_speed_mps'] = round(float(v.max()), 2)
    out['teleport_times'] = [round(float(t[i + 1]), 3) for i in tele_idx[:20]]

    # trust channel — covariance census
    regimes = {}
    traces = []
    for r in ok:
        lab = classify_cov(r['cov'])
        regimes[lab] = regimes.get(lab, 0) + 1
        if lab == 'varying':
            traces.append(float(np.trace(r['cov'].reshape(6, 6))))
    out['cov_regimes'] = regimes
    if traces:
        out['cov_trace_p50'] = float(np.percentile(traces, 50))
        out['cov_trace_p95'] = float(np.percentile(traces, 95))

    # trusted-health channel (fork only): does the internal evidence separate the
    # teleport frames the success flag misses?
    hs = [r for r in ok if r.get('health')]
    if hs:
        tele_t = set(round(float(t[i + 1]), 4) for i in tele_idx)
        def _split(key):
            a = [r['health'][key] for r in hs if round(r['t'], 4) in tele_t]
            b = [r['health'][key] for r in hs if round(r['t'], 4) not in tele_t]
            return (float(np.median(a)) if a else None,
                    float(np.median(b)) if b else None)
        inl_t, inl_n = _split('inliers')
        cond_t, cond_n = _split('info_condition')
        res_t, res_n = _split('mean_residual')
        out['health'] = {
            'inliers_median_teleport': inl_t, 'inliers_median_normal': inl_n,
            'info_condition_median_teleport': cond_t, 'info_condition_median_normal': cond_n,
            'mean_residual_median_teleport': res_t, 'mean_residual_median_normal': res_n,
        }

    # accuracy channel (GT only)
    if gt_tum and os.path.exists(gt_tum):
        tg, Pg = load_tum(gt_tum)
        pairs = associate(t, tg)
        if len(pairs) > 10:
            ie = [p[0] for p in pairs]
            ig = [p[1] for p in pairs]
            aligned, s, R, tr = umeyama_sim3(P[ie], Pg[ig])
            err = np.linalg.norm(aligned - Pg[ig], axis=1)
            out['ate_rmse_m'] = round(float(np.sqrt((err ** 2).mean())), 4)
            out['ate_max_m'] = round(float(err.max()), 4)
            out['sim3_scale'] = round(float(s), 4)
            # translation-block NEES on the aligned error (order: rot-first -> trans = 3:6)
            nees = []
            for k, (i, j) in enumerate(pairs):
                cov = ok[i]['cov']
                if cov is None or classify_cov(cov) != 'varying':
                    continue
                Ct = cov.reshape(6, 6)[3:6, 3:6] * (s ** 2)
                e = aligned[k] - Pg[j]
                try:
                    nees.append(float(e @ np.linalg.solve(Ct, e)))
                except np.linalg.LinAlgError:
                    continue
            if nees:
                out['nees_trans_median'] = round(float(np.median(nees)), 2)
                out['nees_trans_expected'] = 3.0
                out['nees_frames'] = len(nees)
    return out


def save_run(recs, metrics, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'traj.tum'), 'w') as f:
        for r in recs:
            if r['ok']:
                q = r['quat']
                f.write(f"{r['t']:.9f} {r['pos'][0]} {r['pos'][1]} {r['pos'][2]} "
                        f"{q[0]} {q[1]} {q[2]} {q[3]}\n")
    with open(os.path.join(out_dir, 'cov_trace.csv'), 'w') as f:
        w = csv.writer(f)
        w.writerow(['t', 'regime', 'trace', 'obs', 'inliers', 'mean_residual',
                    'final_cost', 'info_condition'])
        for r in recs:
            if r['ok']:
                lab = classify_cov(r['cov'])
                tr = float(np.trace(r['cov'].reshape(6, 6))) if lab == 'varying' else ''
                h = r.get('health') or {}
                w.writerow([f"{r['t']:.6f}", lab, tr, h.get('obs', ''),
                            h.get('inliers', ''), h.get('mean_residual', ''),
                            h.get('final_cost', ''), h.get('info_condition', '')])
    with open(os.path.join(out_dir, 'metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=2)
    print(json.dumps(metrics, indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)
    e = sub.add_parser('euroc')
    e.add_argument('--seq', required=True)
    e.add_argument('--mode', choices=['stereo', 'vio'], default='stereo')
    e.add_argument('--sync-sba', action='store_true',
                   help='deterministic SBA (default: async, like production)')
    e.add_argument('--health', action='store_true',
                   help='capture pnp_health (slamko/trusted-health fork only)')
    e.add_argument('--out', default=DEFAULT_OUT)
    b = sub.add_parser('bag')
    b.add_argument('--bag', required=True)
    b.add_argument('--mode', choices=['stereo', 'vio'], default='stereo')
    b.add_argument('--left', default='/camera/camera/infra1/image_rect_raw')
    b.add_argument('--right', default='/camera/camera/infra2/image_rect_raw')
    b.add_argument('--left-info', default='/camera/camera/infra1/camera_info')
    b.add_argument('--right-info', default='/camera/camera/infra2/camera_info')
    b.add_argument('--imu', default='/camera/camera/imu')
    b.add_argument('--imu-scale', type=float, default=1.0,
                   help='accel scale fix (bno_ab bags: 0.5 per the doubled-accel gotcha)')
    b.add_argument('--health', action='store_true',
                   help='capture pnp_health (slamko/trusted-health fork only)')
    b.add_argument('--out', default=DEFAULT_OUT)
    args = ap.parse_args()

    cuvslam.set_verbosity(0)
    cuvslam.warm_up_gpu()

    if args.cmd == 'euroc':
        seq_dir = os.path.join(EUROC_ROOT, args.seq, 'mav0')
        rig = get_rig(seq_dir)
        tracker = make_tracker(rig, args.mode, async_sba=not args.sync_sba,
                               health=args.health)
        frames = euroc_frames(seq_dir, args.mode)
        recs, _ = run_frames(tracker, frames, args.mode, health=args.health)
        gt = os.path.join(seq_dir, 'state_groundtruth_estimate0', 'data_tum.txt')
        name = f"{args.seq}_{args.mode}"
        metrics = analyze(recs, gt_tum=gt, name=name)
    else:
        rig, baseline = d455_rig_from_bag(args.bag, args.left_info, args.right_info)
        print(f"D455 rig from bag: baseline={baseline:.4f} m")
        tracker = make_tracker(rig, args.mode, async_sba=True, health=args.health)
        frames = bag_frames(args.bag, args.left, args.right, args.imu, args.mode)
        recs, _ = run_frames(tracker, frames, args.mode, imu_scale=args.imu_scale,
                             health=args.health)
        name = os.path.basename(args.bag.rstrip('/')) + f"_{args.mode}"
        metrics = analyze(recs, name=name)

    save_run(recs, metrics, os.path.join(args.out, name))


if __name__ == '__main__':
    main()
