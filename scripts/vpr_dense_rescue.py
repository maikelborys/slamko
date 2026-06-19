#!/usr/bin/env python3
"""Does a DENSE matcher (LoFTR) RESCUE the blind-spot revisit pairs that the global VPR
descriptor (EigenPlaces) fails? Methodology that follows the VPR A/B: pick same-place pairs
(OKVIS-pose GT) sorted by EigenPlaces cosine; on the WORST (blind spots, low cosine) run LoFTR
+ RANSAC and count geometric inliers. If low-cosine same-place pairs still get many LoFTR
inliers, the dense matcher recovers them -> the cascade (global-retrieve, dense-verify-borderline)
is justified. Controls: good same-place pairs (should also pass) + different-place (should fail).

  /tmp/vprvenv/bin/python scripts/vpr_dense_rescue.py --bag <mcap> --poses <provider.tum>
"""
import argparse, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from vpr_ab_casa import load_poses, pose_at, describe  # reuse


def extract_full(bag, topic, stride, max_frames, H=480, W=640):
    from rosbags.highlevel import AnyReader
    from pathlib import Path
    import cv2
    ft, imgs = [], []
    with AnyReader([Path(bag)]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        i = 0
        for conn, ts, raw in reader.messages(connections=conns):
            if i % stride == 0:
                m = reader.deserialize(raw, conn.msgtype)
                img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, 0]
                imgs.append(cv2.resize(img, (W, H)))
                ft.append(ts * 1e-9)
                if len(imgs) >= max_frames:
                    break
            i += 1
    return np.array(ft), np.stack(imgs)


def loftr_inliers(matcher, ia, ib, dev, conf=0.5):
    import torch, cv2
    ta = torch.from_numpy(ia).float()[None, None].to(dev) / 255.0
    tb = torch.from_numpy(ib).float()[None, None].to(dev) / 255.0
    with torch.no_grad():
        out = matcher({'image0': ta, 'image1': tb})
    k0 = out['keypoints0'].cpu().numpy(); k1 = out['keypoints1'].cpu().numpy()
    c = out['confidence'].cpu().numpy()
    m = c >= conf
    k0, k1 = k0[m], k1[m]
    if len(k0) < 8:
        return len(k0), 0
    F, mask = cv2.findFundamentalMat(k0, k1, cv2.FM_RANSAC, 1.5, 0.99)
    inl = int(mask.sum()) if mask is not None else 0
    return len(k0), inl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--poses', required=True)
    ap.add_argument('--topic', default='/camera/camera/infra1/image_rect_raw')
    ap.add_argument('--stride', type=int, default=25)
    ap.add_argument('--max-frames', type=int, default=300)
    ap.add_argument('--same-m', type=float, default=0.6)
    ap.add_argument('--min-dt', type=float, default=8.0)
    ap.add_argument('--cache', default='/tmp/vpr_full_frames.npz')
    ap.add_argument('--onnx', default=os.path.join(os.path.dirname(__file__), '..', 'slamko_vio', 'models', 'eigenplaces.onnx'))
    a = ap.parse_args()

    if os.path.exists(a.cache):
        z = np.load(a.cache); ft, imgs = z['ft'], z['imgs']
    else:
        ft, imgs = extract_full(a.bag, a.topic, a.stride, a.max_frames)
        np.savez_compressed(a.cache, ft=ft, imgs=imgs)
    print(f'{len(ft)} full-res frames')

    ts_pose, xyz = load_poses(a.poses)
    pos = np.array([pose_at(ts_pose, xyz, t) for t in ft])
    n = len(ft)
    D = np.linalg.norm(pos[:, None] - pos[None], axis=2)
    dt = np.abs(ft[:, None] - ft[None])
    eig = describe('eigenplaces', imgs, a.onnx)  # uses internal 512 resize
    C = eig @ eig.T

    iu = np.triu_indices(n, 1)
    same = (D[iu] <= a.same_m) & (dt[iu] >= a.min_dt)
    diff = D[iu] >= 3.0
    pairs = list(zip(iu[0][same], iu[1][same], C[iu][same]))
    pairs.sort(key=lambda p: p[2])  # by EigenPlaces cosine, worst first
    dpairs = list(zip(iu[0][diff], iu[1][diff], C[iu][diff]))

    import torch, kornia as K
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    matcher = K.feature.LoFTR(pretrained='indoor').eval().to(dev)

    print('\n=== BLIND-SPOT same-place pairs (lowest EigenPlaces cosine) — does LoFTR rescue? ===')
    print(f"{'eig_cos':>8} {'loftr_matches':>14} {'ransac_inliers':>15}  verdict")
    rescued = 0; tested = pairs[:15]
    for i, j, cos in tested:
        nm, inl = loftr_inliers(matcher, imgs[i], imgs[j], dev)
        ok = inl >= 30
        rescued += ok
        print(f'{cos:8.3f} {nm:14d} {inl:15d}  {"RESCUED" if ok else "no"}')
    print(f'\nblind-spot rescue: {rescued}/{len(tested)} low-cosine revisits recovered by LoFTR (>=30 inliers)')

    print('\n=== controls ===')
    gi, gj, gc = pairs[-1]; nm, inl = loftr_inliers(matcher, imgs[gi], imgs[gj], dev)
    print(f'good same-place (cos {gc:.3f}): LoFTR {inl} inliers (expect high)')
    di, dj, dc = dpairs[len(dpairs)//2]; nm, inl = loftr_inliers(matcher, imgs[di], imgs[dj], dev)
    print(f'different-place (cos {dc:.3f}): LoFTR {inl} inliers (expect ~0 = no false rescue)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
