#!/usr/bin/env python3
"""Produce per-keyframe depth (.skdf) for slamko_tsdf from a recorded D455 bag.

For each slamko KEYFRAME (id + timestamp read from the .smap archive) we grab the
nearest rectified IR stereo pair (infra1=left/cam0, infra2=right/cam1) from the
bag, run OpenCV StereoSGBM → disparity → metric depth (depth = fx*baseline/disp),
and write a `kf_<id>.skdf` file (the SKDF format in depth_io.hpp). The export
driver then re-integrates these at the keyframes' CORRECTED poses (the bend).

SGBM (CPU) is the v0 depth source — robust, no TRT engine to chase; HITNet/ESS is
the quality upgrade behind the same .skdf contract. The IR pair is already
rectified (image_rect_raw, zero distortion) so disparity→depth is direct.

  make_depth_skdf.py --archive <map_dir> --bag <bag_dir> --out <skdf_dir> \
      [--fx 426.1532] [--baseline 0.0950564] [--max-depth 8.0] [--tol 0.02]

Defaults match config rsD455_map848 (848x480 casa bags). Uses system ROS python
(cv2 + rosbag2_py); NO pip (PEP668).
"""
import argparse
import glob
import os
import struct
import sys

import numpy as np
import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

# T_SC0 (cam0=left IR in IMU/body frame) from rsD455_map848 okvis2.yaml: identity
# rotation, t = (-0.03022, 0.0074, 0.01602). slamko body = OKVIS IMU frame.
T_BODY_CAM = np.array([
    [1.0, 0.0, 0.0, -0.0302200001],
    [0.0, 1.0, 0.0,  0.0074000000],
    [0.0, 0.0, 1.0,  0.0160200000],
    [0.0, 0.0, 0.0,  1.0],
], dtype=np.float64)


def kf_timestamps(archive_dir):
    """Read (kf_id, timestamp[s]) for every keyframe across the .smap archive.
    .smap layout (submap_io.hpp): magic(4) + id(u64) + anchor 7d(56) + nk(u64) +
    nk * [id u64 + ts double + pose 7d] (=72 B each)."""
    kfs = []
    for path in sorted(glob.glob(os.path.join(archive_dir, "submap_*.smap"))):
        with open(path, "rb") as fh:
            d = fh.read()
        if d[:4] not in (b"SMP1", b"SMP2", b"SMP3", b"SMP4", b"SMP5", b"SMP6"):
            print(f"  skip {path}: bad magic {d[:4]!r}")
            continue
        off = 4 + 8 + 56                       # magic + submap id + anchor
        (nk,) = struct.unpack_from("<Q", d, off); off += 8
        for _ in range(nk):
            kid, ts = struct.unpack_from("<Qd", d, off)
            off += 72
            kfs.append((kid, ts))
    return kfs


def collect_stereo(bag_dir, want_ts, tol):
    """Single pass: keep the infra1/infra2 frame closest to each wanted ts
    (within tol). Returns {kf_index: (left_img, right_img)} keyed by want_ts idx."""
    want = np.asarray(want_ts, dtype=np.float64)
    best = {}  # (cam, idx) -> (dt, img)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    topics = {"/camera/camera/infra1/image_rect_raw": 0,
              "/camera/camera/infra2/image_rect_raw": 1}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        cam = topics[topic]
        m = deserialize_message(data, Image)
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        i = int(np.argmin(np.abs(want - ts)))
        dt = abs(want[i] - ts)
        if dt > tol:
            continue
        key = (cam, i)
        if key not in best or dt < best[key][0]:
            img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width)
            best[key] = (dt, img.copy())
    pairs = {}
    n = len(want_ts)
    for i in range(n):
        l = best.get((0, i))
        r = best.get((1, i))
        if l is not None and r is not None:
            pairs[i] = (l[1], r[1])
    return pairs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--archive", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fx", type=float, default=426.1532)
    ap.add_argument("--fy", type=float, default=426.1532)
    ap.add_argument("--cx", type=float, default=423.6672)
    ap.add_argument("--cy", type=float, default=240.5506)
    ap.add_argument("--baseline", type=float, default=0.0950564)
    ap.add_argument("--min-depth", type=float, default=0.3)
    ap.add_argument("--max-depth", type=float, default=5.0)
    ap.add_argument("--tol", type=float, default=0.02)
    ap.add_argument("--min-conf", type=float, default=110.0,
                    help="WLS confidence [0-255] below which depth is dropped")
    ap.add_argument("--engine", choices=["sgbm", "hitnet"], default="sgbm")
    ap.add_argument("--hitnet-engine", default=os.path.expanduser(
        "~/coding/hitnet/engines/hitnet_eth3d_480x640_fp16.engine"),
        help="HITNet TRT engine (run this script with the FFS venv python for GPU)")
    ap.add_argument("--model-w", type=int, default=640)
    ap.add_argument("--model-h", type=int, default=480)
    ap.add_argument("--stride", type=int, default=1, help="use every Nth keyframe")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    kfs = kf_timestamps(a.archive)
    if not kfs:
        sys.exit(f"no keyframes in {a.archive}")
    print(f"{len(kfs)} keyframes from archive")

    pairs = collect_stereo(a.bag, [ts for _, ts in kfs], a.tol)
    print(f"matched stereo for {len(pairs)}/{len(kfs)} keyframes (tol {a.tol}s)")

    left_m = right_m = wls = None
    if a.engine == "sgbm":
        # max disparity for the closest depth we trust → numDisparities (mult 16).
        max_disp = int(np.ceil(a.fx * a.baseline / a.min_depth))
        num_disp = int(np.ceil(max_disp / 16.0)) * 16
        bs = 5
        left_m = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=num_disp, blockSize=bs,
            P1=8 * bs * bs, P2=32 * bs * bs, disp12MaxDiff=1, uniquenessRatio=12,
            speckleWindowSize=200, speckleRange=1,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        # WLS edge-aware filter (left+right matcher + confidence) — kills the SGBM
        # speckle/streak noise that floods the TSDF with floating geometry.
        right_m = cv2.ximgproc.createRightMatcher(left_m)
        wls = cv2.ximgproc.createDisparityWLSFilter(left_m)
        wls.setLambda(8000.0)
        wls.setSigmaColor(1.5)

    # HITNet (learned stereo, GPU TRT): denser than SGBM (fills textureless walls)
    # but can hallucinate. Replicates the proven d455_hitnet_adapter path — resize
    # src→model (640x480), TRT, disp@model → depth (fx_model=fx*model_w/src_w),
    # resize depth back to src → FULL src intrinsics (same as SGBM). Run this script
    # with the FFS venv python (torch+TRT): ~/coding/FFS/.venv/bin/python ...
    runner = None
    if a.engine == "hitnet":
        import sys
        sys.path.insert(0, os.path.expanduser(
            "~/coding/FFS/Fast-FoundationStereo/scripts"))
        import torch
        from run_demo_single_trt import SingleEngineTrtRunner
        runner = SingleEngineTrtRunner(a.hitnet_engine)
        mw, mh = a.model_w, a.model_h
        fx_model = a.fx * mw / 848.0
        runner({"input": torch.zeros((1, 2, mh, mw),
                                     dtype=torch.float32, device="cuda")})  # warmup
        print(f"HITNet TRT ready (fx_model={fx_model:.1f})", flush=True)
    cx_out = a.cx  # depth is resized back to src res → src intrinsics either way

    written = 0
    for i, (kid, _ts) in enumerate(kfs):
        if i not in pairs or (i % a.stride):
            continue
        left, right = pairs[i]

        if a.engine == "hitnet":
            l = cv2.resize(left, (mw, mh))
            r = cv2.resize(right, (mw, mh))
            comb = np.stack([l, r], axis=0).astype(np.float32) / 255.0
            t = torch.from_numpy(comb).unsqueeze(0).cuda()
            out = runner({"input": t})
            disp = out["reference_output_disparity"].squeeze().detach().cpu().numpy()
            dm = np.zeros_like(disp, dtype=np.float32)
            v = disp > 0.1
            dm[v] = fx_model * a.baseline / disp[v]
            depth = cv2.resize(dm, (left.shape[1], left.shape[0]),
                               interpolation=cv2.INTER_NEAREST)
        else:
            dl = left_m.compute(left, right)
            dr = right_m.compute(right, left)
            filt = wls.filter(dl, left, disparity_map_right=dr).astype(np.float32) / 16.0
            conf = wls.getConfidenceMap()
            depth = np.zeros_like(filt)
            valid = (filt > 0.5) & (conf >= a.min_conf)
            depth[valid] = a.fx * a.baseline / filt[valid]

        depth[(depth < a.min_depth) | (depth > a.max_depth)] = 0.0
        h, w = depth.shape

        path = os.path.join(a.out, f"kf_{kid}.skdf")
        with open(path, "wb") as o:
            o.write(b"SKDF")
            o.write(struct.pack("<i", 1))
            o.write(struct.pack("<Q", kid))
            o.write(struct.pack("<ii", w, h))
            o.write(struct.pack("<4d", a.fx, a.fy, cx_out, a.cy))
            o.write(struct.pack("<16d", *T_BODY_CAM.reshape(-1)))
            o.write(depth.astype("<f4").tobytes())
        written += 1
        if a.engine == "hitnet" and written % 20 == 0:
            print(f"  ...{written} frames", flush=True)
    print(f"wrote {written} .skdf depth files -> {a.out}")


if __name__ == "__main__":
    main()
