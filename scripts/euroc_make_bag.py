#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Maikel Borys
#
# Write a rosbag2 from a EuRoC dataset dir with the EXACT topic surface of the
# existing /mnt/data/euroc_bags/mh_XX_okvis bags (/euroc/cam0|1/image_raw +
# camera_info + /euroc/imu0, stamps = sensor ns). Needed because only MH01/03/05
# were ever bagged and the Atlas multi-session test needs all five.
#   ~/.venvs/cuvslam/bin/python euroc_make_bag.py MH_02_easy /mnt/data/euroc_bags/mh_02_okvis

import csv
import os
import sys

import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

seq, out = sys.argv[1], sys.argv[2]
root = f'/mnt/data/datasets/euroc/{seq}/mav0'
ts = get_typestore(Stores.ROS2_JAZZY)
Image = ts.types['sensor_msgs/msg/Image']
CameraInfo = ts.types['sensor_msgs/msg/CameraInfo']
Imu = ts.types['sensor_msgs/msg/Imu']
Header = ts.types['std_msgs/msg/Header']
Time = ts.types['builtin_interfaces/msg/Time']
Quaternion = ts.types['geometry_msgs/msg/Quaternion']
Vector3 = ts.types['geometry_msgs/msg/Vector3']
RegionOfInterest = ts.types['sensor_msgs/msg/RegionOfInterest']

import yaml
from PIL import Image as PILImage


def cam_yaml(cam):
    with open(f'{root}/{cam}/sensor.yaml') as f:
        return yaml.safe_load(f)


def caminfo(cfg, stamp, frame):
    fu, fv, cu, cv = cfg['intrinsics']
    k1, k2, p1, p2 = cfg['distortion_coefficients']
    return CameraInfo(
        header=Header(stamp=stamp, frame_id=frame),
        height=cfg['resolution'][1], width=cfg['resolution'][0],
        distortion_model='plumb_bob',
        d=np.array([k1, k2, p1, p2, 0.0]),
        k=np.array([fu, 0, cu, 0, fv, cv, 0, 0, 1], dtype=np.float64),
        r=np.eye(3, dtype=np.float64).flatten(),
        p=np.array([fu, 0, cu, 0, 0, fv, cv, 0, 0, 0, 1, 0], dtype=np.float64),
        binning_x=0, binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))


def st(ns):
    return Time(sec=int(ns // 10**9), nanosec=int(ns % 10**9))


def read_cam_csv(cam):
    rows = []
    with open(f'{root}/{cam}/data.csv') as f:
        next(f)
        for r in csv.reader(f):
            if r:
                rows.append((int(r[0]), r[1]))
    return rows


cfg0, cfg1 = cam_yaml('cam0'), cam_yaml('cam1')
cams = {0: read_cam_csv('cam0'), 1: read_cam_csv('cam1')}
imu_rows = []
with open(f'{root}/imu0/data.csv') as f:
    next(f)
    for r in csv.reader(f):
        if r:
            imu_rows.append((int(r[0]), [float(x) for x in r[1:7]]))

events = ([(t, 'c0', fn) for t, fn in cams[0]] + [(t, 'c1', fn) for t, fn in cams[1]]
          + [(t, 'imu', v) for t, v in imu_rows])
events.sort(key=lambda e: e[0])
print(f'{seq}: {len(cams[0])}+{len(cams[1])} images, {len(imu_rows)} imu')

os.path.exists(out) and sys.exit(f'{out} exists')
with Writer(out) as w:
    conns = {
        'c0i': w.add_connection('/euroc/cam0/image_raw', Image.__msgtype__, typestore=ts),
        'c0c': w.add_connection('/euroc/cam0/camera_info', CameraInfo.__msgtype__, typestore=ts),
        'c1i': w.add_connection('/euroc/cam1/image_raw', Image.__msgtype__, typestore=ts),
        'c1c': w.add_connection('/euroc/cam1/camera_info', CameraInfo.__msgtype__, typestore=ts),
        'imu': w.add_connection('/euroc/imu0', Imu.__msgtype__, typestore=ts),
    }
    zc = np.zeros(9)
    for t, kind, payload in events:
        stamp = st(t)
        if kind == 'imu':
            g, a = payload[0:3], payload[3:6]
            m = Imu(header=Header(stamp=stamp, frame_id='imu0'),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    orientation_covariance=zc,
                    angular_velocity=Vector3(x=g[0], y=g[1], z=g[2]),
                    angular_velocity_covariance=zc,
                    linear_acceleration=Vector3(x=a[0], y=a[1], z=a[2]),
                    linear_acceleration_covariance=zc)
            w.write(conns['imu'], t, ts.serialize_cdr(m, Imu.__msgtype__))
        else:
            cam = 0 if kind == 'c0' else 1
            arr = np.array(PILImage.open(f'{root}/cam{cam}/data/{payload}'))
            frame = f'cam{cam}'
            m = Image(header=Header(stamp=stamp, frame_id=frame),
                      height=arr.shape[0], width=arr.shape[1], encoding='mono8',
                      is_bigendian=0, step=arr.shape[1], data=arr.flatten())
            w.write(conns[f'c{cam}i'], t, ts.serialize_cdr(m, Image.__msgtype__))
            cfg = cfg0 if cam == 0 else cfg1
            ci = caminfo(cfg, stamp, frame)
            w.write(conns[f'c{cam}c'], t, ts.serialize_cdr(ci, CameraInfo.__msgtype__))
print('wrote', out)
