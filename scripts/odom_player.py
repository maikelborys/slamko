#!/usr/bin/env python3
# Image-driven odometry replayer for two-pass mapping (pass 2).
#
# Replaying the recorded odometry bag alongside the image bag with two
# independent `ros2 bag play` processes skews their timelines by the pass-1
# pre-roll (recorder starts ~10-20 s before OKVIS's first message) — the fusion
# node's image ring buffer never overlaps. This player removes pacing entirely:
# it loads ALL odometry messages from the mcap up front and re-publishes each
# one as soon as an incoming IMAGE's header stamp passes it. Header-stamp-driven
# -> perfectly synced with whatever rate the image bag plays at.
#
#   ros2 run ... | python3 scripts/odom_player.py <odom_bag_dir> [topic_out]
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message
import rosbag2_py


def load_odom(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    msgs = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if 'odometry' in topic:
            m = deserialize_message(data, Odometry)
            msgs.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m))
    msgs.sort(key=lambda x: x[0])
    return msgs


class OdomPlayer(Node):
    def __init__(self, bag_dir, topic_out):
        super().__init__('odom_player')
        self.msgs = load_odom(bag_dir)
        self.i = 0
        self.pub = self.create_publisher(Odometry, topic_out, 100)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            Image, '/camera/camera/infra1/image_rect_raw', self.on_image, qos)
        self.get_logger().info(
            f'{len(self.msgs)} odometry msgs loaded from {bag_dir}; image-driven')

    def on_image(self, img):
        t = img.header.stamp.sec + img.header.stamp.nanosec * 1e-9
        # publish everything up to the image stamp (+ tiny lead so the KF's own
        # odometry, which OKVIS stamps AT the frame time, goes out too)
        while self.i < len(self.msgs) and self.msgs[self.i][0] <= t + 0.001:
            self.pub.publish(self.msgs[self.i][1])
            self.i += 1
        if self.i >= len(self.msgs):
            self.get_logger().info('all odometry published', once=True)


def main():
    rclpy.init()
    n = OdomPlayer(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else '/okvis/okvis_odometry')
    rclpy.spin(n)


if __name__ == '__main__':
    main()
