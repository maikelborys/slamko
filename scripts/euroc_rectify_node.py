#!/usr/bin/env python3
"""Live stereo-rectify node for EuRoC -> feeds slamko's stereo-XFeat landmark path.

EuRoC bags carry RAW radtan images; OKVIS undistorts them INTERNALLY with its own
calib, so the provider chain needs no rectification. slamko's stereo-landmark /
reloc path, however, assumes ROW-ALIGNED rectified pinhole stereo. This node
rectifies the raw pair ON THE FLY (no bag re-recording) and publishes a SECOND
stream that only slamko consumes — OKVIS keeps eating /euroc/cam{0,1}/image_raw.

  python3 euroc_rectify_node.py --seq MH_03_medium

Publishes (752x480, distortion zeroed, OpenCV stereoRectify P0/P1):
  /euroc/cam0/image_rect          /euroc/cam0/camera_info_rect
  /euroc/cam1/image_rect          /euroc/cam1/camera_info_rect
slamko reads baseline from right P[3] = -fx*baseline (matches onInfo).
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def load_cam(p):
    d = yaml.safe_load(open(p))
    K = np.eye(3)
    fu, fv, cu, cv_ = d["intrinsics"]
    K[0, 0], K[1, 1], K[0, 2], K[1, 2] = fu, fv, cu, cv_
    D = np.array(d["distortion_coefficients"])
    T_BS = np.array(d["T_BS"]["data"]).reshape(4, 4)
    return K, D, T_BS


class Rectify(Node):
    def __init__(self, seq):
        super().__init__("euroc_rectify_node")
        base = Path("/mnt/data/datasets/euroc") / seq / "mav0"
        K0, D0, T_B_C0 = load_cam(base / "cam0/sensor.yaml")
        K1, D1, T_B_C1 = load_cam(base / "cam1/sensor.yaml")
        T_C1_C0 = np.linalg.inv(T_B_C1) @ T_B_C0
        R, t = T_C1_C0[:3, :3], T_C1_C0[:3, 3]
        self.size = (752, 480)
        R0, R1, P0, P1, _, _, _ = cv2.stereoRectify(
            K0, D0, K1, D1, self.size, R, t,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self.m0 = cv2.initUndistortRectifyMap(K0, D0, R0, P0, self.size, cv2.CV_32FC1)
        self.m1 = cv2.initUndistortRectifyMap(K1, D1, R1, P1, self.size, cv2.CV_32FC1)
        self.P0, self.P1 = P0, P1
        self.bridge = CvBridge()
        self.get_logger().info(
            f"rectified: fx={P0[0,0]:.2f} baseline={-P1[0,3]/P1[0,0]:.4f} m")

        # Bags publish images BEST_EFFORT; match it or we get nothing.
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub0 = self.create_publisher(Image, "/euroc/cam0/image_rect", qos)
        self.pub1 = self.create_publisher(Image, "/euroc/cam1/image_rect", qos)
        self.pubi0 = self.create_publisher(CameraInfo, "/euroc/cam0/camera_info_rect", qos)
        self.pubi1 = self.create_publisher(CameraInfo, "/euroc/cam1/camera_info_rect", qos)
        self.create_subscription(Image, "/euroc/cam0/image_raw",
                                 lambda m: self.cb(m, 0), qos)
        self.create_subscription(Image, "/euroc/cam1/image_raw",
                                 lambda m: self.cb(m, 1), qos)

    def info(self, hdr, P):
        ci = CameraInfo()
        ci.header = hdr
        ci.width, ci.height = self.size
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0] * 5
        ci.k = [P[0, 0], 0.0, P[0, 2], 0.0, P[1, 1], P[1, 2], 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = list(P.reshape(-1))
        return ci

    def cb(self, msg, cam):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        m = self.m0 if cam == 0 else self.m1
        rect = cv2.remap(img, m[0], m[1], cv2.INTER_LINEAR)
        out = self.bridge.cv2_to_imgmsg(rect, encoding="mono8")
        out.header = msg.header
        (self.pub0 if cam == 0 else self.pub1).publish(out)
        (self.pubi0 if cam == 0 else self.pubi1).publish(
            self.info(msg.header, self.P0 if cam == 0 else self.P1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", default="MH_03_medium")
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = Rectify(a.seq)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
