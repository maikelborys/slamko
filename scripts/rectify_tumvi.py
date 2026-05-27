#!/usr/bin/env python3
"""
rectify_tumvi.py — TUM VI equidistant-fisheye → pinhole rectifier (offline preprocessor).

WHY THIS EXISTS
---------------
slamko_vio is a **rectified-pinhole** tracker (KLT + stereo + PnP project with
u = fx·X/Z + cx, no fisheye model). TUM VI ships **equidistant fisheye** 512×512 with
4 distortion coeffs [k1,k2,k3,k4] — feeding it raw breaks the geometry. This script
undistorts the fisheye stereo pair into a virtual **pinhole** stereo pair (row-aligned,
horizontal epipolar, baseline along +x) and writes a drop-in EuRoC `mav0/`, so the
existing `euroc_player` + `vio_euroc.launch.py` replay it unchanged.

THE "DOUBLE RECTIFY" IS A NO-OP (the load-bearing trick)
--------------------------------------------------------
`euroc_player` ALSO rectifies internally — but with cv2.stereoRectify (pinhole+radtan),
which is wrong for fisheye. So we pre-rectify here with cv2.fisheye.* and write the
output `mav0` with **zero distortion** + **identical K on both cams** + a **pure-x
baseline**. The player's second pinhole-rectify on that pair then collapses to R1=R2=I
and an identity remap (K==P, D=0) — an EXACT no-op, no double-warp. The only thing the
player still needs from us correctly is each rectified cam's `T_BS` (cam→body), which we
write into the rectified sensor.yaml exactly as the player would compute it internally:
    T_BS_rect = T_BS_raw · diag(R_rect^T, 1)        (R_rect = fisheye rect rotation)

GRAVITY / FRAME NOTE
--------------------
slamko_vio CALIBRATES gravity direction from the accel mean in the visual-world frame
(it assumes no fixed body axis), and the player rebases IMU values + cam TFs by the SAME
rotation — so TUM VI's z-up body frame (vs EuRoC's x-up) is a harmless relabel. We copy
imu0/ and mocap0/ verbatim.

USAGE
-----
    python3 scripts/rectify_tumvi.py \
        --seq /mnt/data/datasets/tumvi/dataset-room1_512_16 \
        --out /mnt/data/datasets/tumvi_rect/room1 \
        [--balance 0.0] [--fov-scale 1.0] [--size 512] [--limit N] [--debug-only]

Outputs <out>/mav0/{cam0,cam1,imu0,mocap0} + <out>/gt.tum (mocap0 → TUM, for ATE) +
<out>/rectify_debug.png (epipolar-line check) and prints the new pinhole intrinsics +
baseline so they can be dropped into the run command.
"""
from __future__ import annotations

import argparse
import csv
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_cam(path: Path):
    with open(path) as f:
        y = yaml.safe_load(f)
    T_BS = np.array(y["T_BS"]["data"], dtype=np.float64).reshape(4, 4)
    fx, fy, cx, cy = y["intrinsics"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    D = np.array(y["distortion_coefficients"], dtype=np.float64).reshape(4, 1)
    w, h = y["resolution"]
    return dict(T_BS=T_BS, K=K, D=D, w=int(w), h=int(h))


def write_cam_yaml(path: Path, K: np.ndarray, T_BS: np.ndarray, w: int, h: int):
    """A rectified pinhole sensor.yaml the player can consume. distortion=0 so the
    player's internal pinhole-rectify is an identity remap (see header)."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    lines = [
        "# Rectified by scripts/rectify_tumvi.py (TUM VI fisheye -> pinhole).",
        "sensor_type: camera",
        "comment: TUM VI rectified pinhole (equidistant fisheye undistorted offline)",
        "",
        "T_BS:",
        "  cols: 4",
        "  rows: 4",
        "  data: [" + ", ".join(f"{v!r}" for v in T_BS.flatten()) + "]",
        "",
        "rate_hz: 20",
        f"resolution: [{w}, {h}]",
        "camera_model: pinhole",
        f"intrinsics: [{fx!r}, {fy!r}, {cx!r}, {cy!r}]",
        "distortion_model: radtan",
        "distortion_coefficients: [0.0, 0.0, 0.0, 0.0]",
        "",
    ]
    path.write_text("\n".join(lines))


def remap_dir(src_cam: Path, dst_cam: Path, map_x, map_y, limit: int | None, clahe):
    """Remap every PNG in src_cam/data → dst_cam/data, copy data.csv verbatim.
    Optional CLAHE applied BEFORE remap (on the raw fisheye) so contrast is
    normalized over the original neighbourhoods, not the stretched ones."""
    (dst_cam / "data").mkdir(parents=True, exist_ok=True)
    shutil.copy(src_cam / "data.csv", dst_cam / "data.csv")
    rows = []
    with open(src_cam / "data.csv") as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            rows.append(row[1].strip())
    if limit:
        rows = rows[:limit]
    n = len(rows)
    for i, fn in enumerate(rows):
        img = cv2.imread(str(src_cam / "data" / fn), cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"  WARN: missing {fn}", file=sys.stderr)
            continue
        if clahe is not None:
            img = clahe.apply(img)
        rect = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)
        cv2.imwrite(str(dst_cam / "data" / fn), rect)
        if i % 500 == 0:
            print(f"  {dst_cam.parent.name}/{dst_cam.name}: {i}/{n}", flush=True)
    return n


def mocap_to_tum(mocap_csv: Path, out_tum: Path):
    """mocap0/data.csv (#ts_ns, px,py,pz, qw,qx,qy,qz) → TUM (t[s] tx ty tz qx qy qz qw)."""
    n = 0
    with open(mocap_csv) as f, open(out_tum, "w") as g:
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            ts = int(row[0]) / 1e9
            px, py, pz = row[1], row[2], row[3]
            qw, qx, qy, qz = row[4], row[5], row[6], row[7]
            g.write(f"{ts:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}\n")
            n += 1
    return n


def save_debug(seq: Path, dst: Path, map_x_l, map_y_l, map_x_r, map_y_r, clahe):
    """Pick the first synced frame, save raw|rect side-by-side with horizontal
    epipolar lines so a human can confirm rows align + lines are straight."""
    def first_png(cam):
        with open(seq / "mav0" / cam / "data.csv") as f:
            for row in csv.reader(f):
                if row and not row[0].startswith("#"):
                    return seq / "mav0" / cam / "data" / row[1].strip()
        return None
    l = cv2.imread(str(first_png("cam0")), cv2.IMREAD_GRAYSCALE)
    r = cv2.imread(str(first_png("cam1")), cv2.IMREAD_GRAYSCALE)
    if clahe is not None:
        l = clahe.apply(l); r = clahe.apply(r)
    rl = cv2.remap(l, map_x_l, map_y_l, cv2.INTER_LINEAR)
    rr = cv2.remap(r, map_x_r, map_y_r, cv2.INTER_LINEAR)
    raw = np.hstack([l, r])
    rect = np.hstack([rl, rr])
    # raw (source size) and rect (new size) widths can differ — match for vstack.
    if raw.shape[1] != rect.shape[1]:
        w = max(raw.shape[1], rect.shape[1])
        raw = cv2.resize(raw, (w, int(raw.shape[0] * w / raw.shape[1])))
        rect = cv2.resize(rect, (w, int(rect.shape[0] * w / rect.shape[1])))
    canvas = np.vstack([raw, rect])
    canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    for y in range(0, canvas.shape[0], 32):
        cv2.line(canvas, (0, y), (canvas.shape[1], y), (0, 255, 0), 1)
    cv2.putText(canvas, "raw fisheye (top)  |  rectified pinhole (bottom)",
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.imwrite(str(dst / "rectify_debug.png"), canvas)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", required=True, help="TUM VI dataset dir (contains mav0/)")
    ap.add_argument("--out", required=True, help="output rectified EuRoC dir")
    ap.add_argument("--balance", type=float, default=0.0,
                    help="fisheye rect balance 0..1 (0=tight/no-black, 1=full-FOV/black corners)")
    ap.add_argument("--fov-scale", type=float, default=1.0,
                    help="fisheye rect fov_scale (>1 widens FOV, more edge stretch)")
    ap.add_argument("--width",  type=int, default=752,
                    help="rectified width  (default 752 = XFeat-native, NO in-net resize)")
    ap.add_argument("--height", type=int, default=480,
                    help="rectified height (default 480 = XFeat-native)")
    ap.add_argument("--clahe", type=float, default=2.0,
                    help="CLAHE clipLimit for dark TUM VI frames (0 disables; default 2.0)")
    ap.add_argument("--limit", type=int, default=0, help="only process first N frames (debug)")
    ap.add_argument("--debug-only", action="store_true",
                    help="only write rectify_debug.png + print intrinsics, skip full remap")
    args = ap.parse_args()
    clahe = cv2.createCLAHE(clipLimit=args.clahe, tileGridSize=(8, 8)) if args.clahe > 0 else None

    seq = Path(args.seq).expanduser().resolve()
    out = Path(args.out).expanduser().resolve()
    limit = args.limit or None

    c0 = load_cam(seq / "mav0" / "cam0" / "sensor.yaml")
    c1 = load_cam(seq / "mav0" / "cam1" / "sensor.yaml")
    size = (c0["w"], c0["h"])
    new_size = (args.width, args.height)

    # cam0 -> cam1 transform (X_cam1 = R·X_cam0 + t), matching euroc_player.
    T_c0_c1 = np.linalg.inv(c1["T_BS"]) @ c0["T_BS"]
    R = T_c0_c1[:3, :3]
    t = T_c0_c1[:3, 3]

    R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
        c0["K"], c0["D"], c1["K"], c1["D"], size, R, t,
        flags=cv2.fisheye.CALIB_ZERO_DISPARITY,
        newImageSize=new_size, balance=args.balance, fov_scale=args.fov_scale)

    map_x_l, map_y_l = cv2.fisheye.initUndistortRectifyMap(
        c0["K"], c0["D"], R1, P1, new_size, cv2.CV_16SC2)
    map_x_r, map_y_r = cv2.fisheye.initUndistortRectifyMap(
        c1["K"], c1["D"], R2, P2, new_size, cv2.CV_16SC2)

    # New pinhole intrinsics (must be identical on both cams for the player's
    # second rectify to be identity). P1/P2 share fx,fy,cx,cy by ZERO_DISPARITY.
    Kl = P1[:3, :3].copy()
    Kr = P2[:3, :3].copy()
    fx = Kl[0, 0]
    baseline = -P2[0, 3] / fx  # P2[0,3] = -fx·baseline
    print(f"\n  rectified pinhole K (left):  fx={Kl[0,0]:.4f} fy={Kl[1,1]:.4f} "
          f"cx={Kl[0,2]:.4f} cy={Kl[1,2]:.4f}")
    print(f"  rectified pinhole K (right): fx={Kr[0,0]:.4f} fy={Kr[1,1]:.4f} "
          f"cx={Kr[0,2]:.4f} cy={Kr[1,2]:.4f}")
    print(f"  baseline = {baseline*1000:.2f} mm   size = {new_size}")
    print(f"  balance={args.balance} fov_scale={args.fov_scale} clahe={args.clahe}")
    if not np.allclose(Kl, Kr, atol=1e-6):
        print("  WARN: left/right K differ — player's identity-rectify assumption breaks!",
              file=sys.stderr)

    # Rectified cam-in-body: T_BS_rect = T_BS_raw · diag(R_rect^T, 1).
    def rect_TBS(T_raw, Rrect):
        M = np.eye(4)
        M[:3, :3] = Rrect.T
        return T_raw @ M
    T_BS_l = rect_TBS(c0["T_BS"], R1)
    T_BS_r = rect_TBS(c1["T_BS"], R2)

    (out / "mav0").mkdir(parents=True, exist_ok=True)
    save_debug(seq, out, map_x_l, map_y_l, map_x_r, map_y_r, clahe)
    print(f"  wrote {out/'rectify_debug.png'}")
    if args.debug_only:
        return

    for cam in ("cam0", "cam1"):
        (out / "mav0" / cam).mkdir(parents=True, exist_ok=True)
    write_cam_yaml(out / "mav0" / "cam0" / "sensor.yaml", Kl, T_BS_l, *new_size)
    write_cam_yaml(out / "mav0" / "cam1" / "sensor.yaml", Kr, T_BS_r, *new_size)

    print("  remapping cam0 ...")
    n0 = remap_dir(seq / "mav0" / "cam0", out / "mav0" / "cam0", map_x_l, map_y_l, limit, clahe)
    print("  remapping cam1 ...")
    n1 = remap_dir(seq / "mav0" / "cam1", out / "mav0" / "cam1", map_x_r, map_y_r, limit, clahe)

    # Copy imu0 + mocap0 verbatim (frames unchanged, see header gravity note).
    for sub in ("imu0", "mocap0"):
        src = seq / "mav0" / sub
        if src.is_dir():
            shutil.copytree(src, out / "mav0" / sub, dirs_exist_ok=True)
    gt_n = 0
    mocap_csv = seq / "mav0" / "mocap0" / "data.csv"
    if mocap_csv.is_file():
        gt_n = mocap_to_tum(mocap_csv, out / "gt.tum")

    print(f"\nDONE: {n0} left + {n1} right rectified frames → {out}")
    print(f"      GT: {gt_n} mocap poses → {out/'gt.tum'}")
    print(f"\nRun:\n  ros2 launch slamko_vio vio_euroc.launch.py seq:={out} \\")
    print(f"    image_width:={new_size[0]} image_height:={new_size[1]} \\")
    print(f"    feature_source:=xfeat rate:=1.0 \\")
    print(f"    pose_dump_path:=/tmp/{out.name}.tum landmark_dump_path:=/tmp/{out.name}_lm.csv")


if __name__ == "__main__":
    main()
