#!/usr/bin/env python3
"""Per-keyframe depth (.skdf) for slamko_tsdf from the bag's REAL hardware depth
(NOT recomputed stereo). For each slamko keyframe (id+ts from the .smap archive)
grab the /camera/camera/depth frame within tol with the MOST valid pixels — the
emitter-ON frame adjacent to the clean-IR keyframe (the flashing recording gives
good depth on emitter-ON, poor on emitter-OFF). uint16 mm -> float32 m. The export
driver re-integrates these at the keyframes' CORRECTED poses (the bend).

  make_depth_skdf_frombag.py --archive <map_dir> --bag <bag_dir> --out <skdf_dir>
Uses system ROS python (rosbag2_py + numpy); NO pip (PEP668).
"""
import argparse, glob, os, struct, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

T_BODY_CAM = np.array([
    [1.0, 0.0, 0.0, -0.0302200001],
    [0.0, 1.0, 0.0,  0.0074000000],
    [0.0, 0.0, 1.0,  0.0160200000],
    [0.0, 0.0, 0.0,  1.0]], dtype=np.float64)


def kf_timestamps(archive_dir):
    kfs = []
    for path in sorted(glob.glob(os.path.join(archive_dir, "submap_*.smap"))):
        d = open(path, "rb").read()
        if d[:4] not in (b"SMP1", b"SMP2", b"SMP3", b"SMP4", b"SMP5", b"SMP6"):
            continue
        off = 4 + 8 + 56
        (nk,) = struct.unpack_from("<Q", d, off); off += 8
        for _ in range(nk):
            kid, ts = struct.unpack_from("<Qd", d, off); off += 72
            kfs.append((kid, ts))
    return kfs


def collect_depth(bag_dir, want_ts, tol):
    """For each wanted ts keep the depth frame within tol with the MOST valid px."""
    want = np.asarray(want_ts, dtype=np.float64)
    best = {}  # idx -> (n_valid, H, W, depth_u16)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=["/camera/camera/depth/image_rect_raw"]))
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, Image)
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        i = int(np.argmin(np.abs(want - ts)))
        if abs(want[i] - ts) > tol:
            continue
        z = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
        nv = int((z > 0).sum())
        if i not in best or nv > best[i][0]:
            best[i] = (nv, m.height, m.width, z.copy())
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--archive", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fx", type=float, default=426.1532)
    ap.add_argument("--fy", type=float, default=426.1532)
    ap.add_argument("--cx", type=float, default=423.6672)
    ap.add_argument("--cy", type=float, default=240.5506)
    ap.add_argument("--min-depth", type=float, default=0.3)
    ap.add_argument("--max-depth", type=float, default=5.0)
    ap.add_argument("--tol", type=float, default=0.025)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    kfs = kf_timestamps(a.archive)
    if not kfs:
        sys.exit(f"no keyframes in {a.archive}")
    print(f"{len(kfs)} keyframes from archive")

    best = collect_depth(a.bag, [ts for _, ts in kfs], a.tol)
    print(f"matched HW depth for {len(best)}/{len(kfs)} keyframes (tol {a.tol}s)")

    written = 0
    for i, (kid, _ts) in enumerate(kfs):
        if i not in best:
            continue
        _, h, w, z = best[i]
        depth = z.astype(np.float32) * 0.001          # mm -> m
        depth[(depth < a.min_depth) | (depth > a.max_depth)] = 0.0
        with open(os.path.join(a.out, f"kf_{kid}.skdf"), "wb") as o:
            o.write(b"SKDF")
            o.write(struct.pack("<i", 1))
            o.write(struct.pack("<Q", kid))
            o.write(struct.pack("<ii", w, h))
            o.write(struct.pack("<4d", a.fx, a.fy, a.cx, a.cy))
            o.write(struct.pack("<16d", *T_BODY_CAM.reshape(-1)))
            o.write(depth.astype("<f4").tobytes())
        written += 1
    print(f"wrote {written} .skdf (REAL HW depth) -> {a.out}")


if __name__ == "__main__":
    main()
