#!/usr/bin/env python3
# P-B step 1 (MASTER_PLAN §8): EigenPlaces recall diagnostic DIRECTLY on dataset
# frames — no VIO in the loop. Motivated the hard way: the perkfvpr_mag1_diag
# smap archive starts at t=777.6 (the deprecated VIO ate 133 s initializing), so
# the start room never entered that map and the start<->end bridge was
# unmeasurable from smaps. Frames + GT room windows are the artifact-free
# substrate: DB = frames with t<=db_tmax (start room per GT), queries = frames
# with t>=query_tmin (the return), distractors = the corridor trek between.
#
# Metrics (mirror slamko_loop/tools/vpr_recall_diag): R@1/5/10 of the first
# DB hit ranked against DB+distractors, margins, rank percentiles, per-query
# CSV. Absolute cosines are LOW by domain gap (~0.25-0.35, PLAN_VPR_RELOC) —
# only the ranking matters.
#
#   python3 scripts/vpr_recall_frames.py --seq /mnt/data/datasets/tumvi_rect/magistrale1 \
#     --db-tmax 1520500707.2 --query-tmin 1520501357.8 --stride 10 \
#     --csv results/pb/mag1_recall_frames.csv

import argparse
import csv
import os
import sys

import numpy as np

MEAN = np.array([0.485, 0.456, 0.406], np.float32).reshape(3, 1, 1)
STD = np.array([0.229, 0.224, 0.225], np.float32).reshape(3, 1, 1)


def load_frames(seq, stride):
    import cv2  # noqa: local import keeps --help fast
    data = os.path.join(seq, 'mav0', 'cam0', 'data.csv')
    rows = []
    with open(data) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            ts, fn = line.strip().split(',')
            rows.append((float(ts) * 1e-9, os.path.join(seq, 'mav0', 'cam0', 'data', fn)))
    return rows[::stride]


def describe(frames, onnx_path):
    import cv2
    import onnxruntime as ort
    sess = ort.InferenceSession(onnx_path, providers=ort.get_available_providers())
    iname = sess.get_inputs()[0].name
    descs = np.zeros((len(frames), 512), np.float32)
    for i, (_, path) in enumerate(frames):
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f'cannot read {path}')
        img = cv2.resize(img, (512, 512), interpolation=cv2.INTER_AREA)
        x = img.astype(np.float32) / 255.0
        x = np.repeat(x[None, :, :], 3, axis=0)
        x = (x - MEAN) / STD
        d = sess.run(None, {iname: x[None]})[0].reshape(-1).astype(np.float32)
        d /= (np.linalg.norm(d) + 1e-12)
        descs[i] = d
        if (i + 1) % 200 == 0:
            print(f'  {i + 1}/{len(frames)} frames described', flush=True)
    return descs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seq', required=True)
    ap.add_argument('--db-tmax', type=float, required=True)
    ap.add_argument('--query-tmin', type=float, required=True)
    ap.add_argument('--stride', type=int, default=10, help='use every Nth frame (~2 Hz at 20 fps)')
    ap.add_argument('--onnx', default=os.path.join(os.path.dirname(__file__), '..',
                    'slamko_vio', 'models', 'eigenplaces.onnx'))
    ap.add_argument('--csv', default='')
    ap.add_argument('--cache', default='', help='npz cache for descriptors')
    a = ap.parse_args()

    frames = load_frames(a.seq, a.stride)
    ts = np.array([t for t, _ in frames])
    print(f'{len(frames)} frames (stride {a.stride}), t {ts[0]:.1f}..{ts[-1]:.1f}')

    if a.cache and os.path.exists(a.cache):
        descs = np.load(a.cache)['descs']
        assert len(descs) == len(frames), 'cache/stride mismatch'
        print(f'descriptors from cache {a.cache}')
    else:
        descs = describe(frames, a.onnx)
        if a.cache:
            np.savez_compressed(a.cache, descs=descs, ts=ts)

    db = ts <= a.db_tmax
    qy = ts >= a.query_tmin
    mid = ~db & ~qy
    print(f'DB(start room)={db.sum()} | distractors={mid.sum()} | queries(return)={qy.sum()}')
    if db.sum() == 0 or qy.sum() == 0:
        print('FAIL: empty DB or query set')
        return 1

    D, M, Q = descs[db], descs[mid], descs[qy]
    tdb, tmid, tq = ts[db], ts[mid], ts[qy]
    cd = Q @ D.T                       # queries x DB cosines
    cm = Q @ M.T if M.size else np.zeros((len(Q), 0), np.float32)

    best_db = cd.max(1)
    best_db_i = cd.argmax(1)
    best_mid = cm.max(1) if cm.size else np.full(len(Q), -1.0)
    best_mid_i = cm.argmax(1) if cm.size else np.zeros(len(Q), int)
    rank = 1 + (cm > best_db[:, None]).sum(1)

    n = len(Q)
    for k in (1, 5, 10):
        print(f'R@{k:<3} = {(rank <= k).mean():.3f}  ({(rank <= k).sum()}/{n})')
    print(f'mean margin (best_db - best_distractor) = {(best_db - best_mid).mean():+.4f}')
    sr = np.sort(rank)
    print(f'rank of first start-room hit: median={sr[n // 2]} p90={sr[(n * 9) // 10]} worst={sr[-1]}')
    print(f'cosines: best_db median {np.median(best_db):.3f} | best_distractor median {np.median(best_mid):.3f}')

    if a.csv:
        os.makedirs(os.path.dirname(a.csv), exist_ok=True)
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'best_db_cos', 'best_db_t', 'best_mid_cos', 'best_mid_t',
                        'margin', 'rank_first_db'])
            for i in range(n):
                w.writerow([f'{tq[i]:.6f}', f'{best_db[i]:.4f}', f'{tdb[best_db_i[i]]:.6f}',
                            f'{best_mid[i]:.4f}',
                            f'{tmid[best_mid_i[i]]:.6f}' if cm.size else '',
                            f'{best_db[i] - best_mid[i]:.4f}', rank[i]])
        print(f'per-query CSV -> {a.csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
