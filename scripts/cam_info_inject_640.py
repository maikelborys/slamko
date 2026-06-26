#!/usr/bin/env python3
"""Inject static CameraInfo the recorded bag is missing — 640x480 rsD455 intrinsics,
left + right (right P encodes the stereo baseline). slamko's relocalizer needs both."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import qos_profile_sensor_data

FX = FY = 385.950073
CX, CY = 319.698578, 240.498688
BASELINE = 0.0950564
W, H = 640, 480


def make(right):
    m = CameraInfo()
    m.width, m.height = W, H
    m.distortion_model = 'plumb_bob'
    m.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    m.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    tx = -FX * BASELINE if right else 0.0
    m.p = [FX, 0.0, CX, tx, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
    m.header.frame_id = 'camera_infra%d_optical_frame' % (2 if right else 1)
    return m


class CI(Node):
    def __init__(self):
        super().__init__('cam_info_inject')
        sd = qos_profile_sensor_data
        self.p1 = self.create_publisher(CameraInfo, '/camera/camera/infra1/camera_info', sd)
        self.p2 = self.create_publisher(CameraInfo, '/camera/camera/infra2/camera_info', sd)
        self.m1, self.m2 = make(False), make(True)
        self.create_timer(0.1, self.tick)

    def tick(self):
        t = self.get_clock().now().to_msg()
        self.m1.header.stamp = t; self.m2.header.stamp = t
        self.p1.publish(self.m1); self.p2.publish(self.m2)


def main():
    rclpy.init(); rclpy.spin(CI())


if __name__ == '__main__':
    main()
