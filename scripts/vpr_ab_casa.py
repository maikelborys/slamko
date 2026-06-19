#!/usr/bin/env python3
"""VPR model A/B on REAL casa frames — does a stronger global descriptor (SALAD / CosPlace)
separate the blind-spot revisit pairs better than EigenPlaces (close the cosine cliff we
measured)? Methodology: GROUND TRUTH from OKVIS poses — two frames are SAME-PLACE if their
camera positions are within `--same-m` (a true revisit) and DIFFERENT-PLACE if > `--diff-m`.
A good VPR model gives high cosine to same-place pairs and low to different-place; the metric
is the SEPARATION (same-median - diff-median) and the same-place cosine floor (recall).

Stage 1 (this run): extract frames from the mcap + pair them by pose, cache to npz.
Stage 2: describe with each model, report separation. Run with --models once frames are cached.

  python3 scripts/vpr_ab_casa.py --bag <mcap_dir> --poses <provider.tum> --stride 25 \
      --cache /tmp/vpr_frames.npz [--models eigenplaces,cosplace,salad]
"""
import argparse, os, sys
import numpy as np


def load_poses(tum):
    P = []
    for ln in open(tum):
        if ln.startswith('#') or not ln.strip():
            continue
        v = ln.split()
        if len(v) >= 8:
            P.append([float(x) for x in v[:8]])  # t x y z qx qy qz qw
    P = np.array(P)
    return P[:, 0], P[:, 1:4]  # ts, xyz


def extract_frames(bag, topic, stride, max_frames):
    from rosbags.highlevel import AnyReader
    from pathlib import Path
    import cv2
    frames_t, imgs = [], []
    with AnyReader([Path(bag)]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            sys.exit(f'topic {topic} not in bag; have: {sorted(set(c.topic for c in reader.connections))[:8]}')
        i = 0
        for conn, ts, raw in reader.messages(connections=conns):
            if i % stride == 0:
                m = reader.deserialize(raw, conn.msgtype)
                h, w = m.height, m.width
                img = np.frombuffer(m.data, np.uint8).reshape(h, w, -1)[:, :, 0]
                img = cv2.resize(img, (224, 224), interpolation=cv2.INTER_AREA)
                frames_t.append(ts * 1e-9)
                imgs.append(img)
                if len(imgs) >= max_frames:
                    break
            i += 1
    return np.array(frames_t), np.stack(imgs)


def pose_at(ts_pose, xyz, t):
    j = np.clip(np.searchsorted(ts_pose, t), 0, len(ts_pose) - 1)
    return xyz[j]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--poses', required=True)
    ap.add_argument('--topic', default='/camera/camera/infra1/image_rect_raw')
    ap.add_argument('--stride', type=int, default=25)
    ap.add_argument('--max-frames', type=int, default=400)
    ap.add_argument('--same-m', type=float, default=0.6)
    ap.add_argument('--diff-m', type=float, default=3.0)
    ap.add_argument('--min-dt', type=float, default=8.0, help='same-place pairs must be >=this apart in time (real revisit)')
    ap.add_argument('--cache', default='/tmp/vpr_frames.npz')
    ap.add_argument('--models', default='', help='comma list: eigenplaces,cosplace,salad')
    ap.add_argument('--onnx', default=os.path.join(os.path.dirname(__file__), '..', 'slamko_vio', 'models', 'eigenplaces.onnx'))
    a = ap.parse_args()

    if os.path.exists(a.cache):
        z = np.load(a.cache)
        ft, imgs = z['ft'], z['imgs']
        print(f'frames from cache {a.cache}: {len(ft)}')
    else:
        ts_pose, xyz = load_poses(a.poses)
        ft, imgs = extract_frames(a.bag, a.topic, a.stride, a.max_frames)
        np.savez_compressed(a.cache, ft=ft, imgs=imgs)
        print(f'extracted {len(ft)} frames @stride {a.stride}, cached -> {a.cache}')

    # GT pairs from poses
    ts_pose, xyz = load_poses(a.poses)
    pos = np.array([pose_at(ts_pose, xyz, t) for t in ft])
    n = len(ft)
    D = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)  # n x n metric dist
    dt = np.abs(ft[:, None] - ft[None, :])
    same = (D <= a.same_m) & (dt >= a.min_dt)   # true revisit (close in space, far in time)
    diff = D >= a.diff_m
    iu = np.triu_indices(n, 1)
    n_same, n_diff = int(same[iu].sum()), int(diff[iu].sum())
    print(f'GT pairs: same-place(revisit)={n_same}  different-place={n_diff}  (of {n*(n-1)//2})')
    if n_same < 5:
        print('WARN: too few revisit pairs — widen --same-m or lower --min-dt')

    if not a.models:
        print('stage 1 done (frames+pairs cached). add --models eigenplaces,cosplace,salad to A/B')
        return 0

    for name in a.models.split(','):
        name = name.strip()
        try:
            descs = describe(name, imgs, a.onnx)
        except Exception as e:
            print(f'{name:12s}: FAILED — {str(e)[:160]}')
            continue
        C = descs @ descs.T
        cs = C[iu][same[iu]]
        cd = C[iu][diff[iu]]
        sep = np.median(cs) - np.median(cd)
        # recall proxy: of same-place pairs, fraction whose cosine beats the 95th pct of different-place
        thr = np.percentile(cd, 95)
        recall = (cs > thr).mean()
        print(f'{name:12s}: same-cos med={np.median(cs):.3f} (min {cs.min():.3f}) | '
              f'diff-cos med={np.median(cd):.3f} p95={thr:.3f} | SEP={sep:+.3f} | '
              f'recall@diff-p95={recall:.2f}')
    return 0


def describe(name, imgs, onnx):
    MEAN = np.array([0.485, 0.456, 0.406], np.float32).reshape(3, 1, 1)
    STD = np.array([0.229, 0.224, 0.225], np.float32).reshape(3, 1, 1)
    if name == 'eigenplaces':
        import onnxruntime as ort, cv2
        sess = ort.InferenceSession(onnx, providers=ort.get_available_providers())
        iname = sess.get_inputs()[0].name
        out = []
        for im in imgs:
            x = cv2.resize(im, (512, 512), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0
            x = np.repeat(x[None], 3, 0); x = (x - MEAN) / STD
            d = sess.run(None, {iname: x[None]})[0].reshape(-1)
            out.append(d / (np.linalg.norm(d) + 1e-12))
        return np.array(out, np.float32)
    # torch models
    import torch
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    if name == 'cosplace':
        m = torch.hub.load('gmberton/cosplace', 'get_trained_model', backbone='ResNet18', fc_output_dim=512, trust_repo=True)
    elif name == 'salad':
        m = torch.hub.load('serizba/salad', 'dinov2_salad', trust_repo=True)
    elif name == 'eigenplaces-torch':
        m = torch.hub.load('gmberton/eigenplaces', 'get_trained_model', backbone='ResNet50', fc_output_dim=2048, trust_repo=True)
    else:
        raise ValueError(f'unknown model {name}')
    m = m.eval().to(dev)
    mean = torch.tensor([0.485, 0.456, 0.406], device=dev).view(1, 3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225], device=dev).view(1, 3, 1, 1)
    out = []
    with torch.no_grad():
        for im in imgs:
            x = torch.from_numpy(im).float().to(dev) / 255.0
            x = x[None, None].repeat(1, 3, 1, 1)
            x = (x - mean) / std
            d = m(x).reshape(-1).cpu().numpy()
            out.append(d / (np.linalg.norm(d) + 1e-12))
    return np.array(out, np.float32)


if __name__ == '__main__':
    sys.exit(main())
