#!/usr/bin/env python3
"""Experiment 7-A recorder — capture cuVSLAM (Isaac ROS visual_slam) odometry pose +
6x6 covariance and input-frame rate, for NEES calibration + fps measurement.

The Isaac node already converts cuVSLAM's native rotation-first/tangent-space covariance
into ROS nav_msgs order [x,y,z,rx,ry,rz] in the body frame (FromcuVSLAMCovariance), so
pose.covariance here is directly usable. We ALSO count input frames vs published odometry
to measure dropped frames (the real-time-fps question, slamko-okvis-realtime-ceiling).

Usage: python3 cuvslam_record_odom.py --out <dir> --image-topic <left img topic>
       [--odom-topic /visual_slam/tracking/odometry]
Writes <dir>/odom.csv (t,x,y,z,qx,qy,qz,qw,cov0..cov35,recv_wall) and prints an fps summary.
"""
import argparse, time, csv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, out_dir, odom_topic, image_topic):
        super().__init__('cuvslam_recorder')
        self.f = open(f'{out_dir}/odom.csv', 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
                        + [f'cov{i}' for i in range(36)] + ['recv_wall'])
        self.n_odom = 0
        self.n_img = 0
        self.t_first = None
        self.t_last = None
        self.wall_first = None
        self.wall_last = None
        self.img_first = None
        self.img_last = None
        be = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 50)
        self.create_subscription(Image, image_topic, self.on_img, be)
        self.get_logger().info(f'recording {odom_topic} + counting {image_topic} -> {out_dir}/odom.csv')

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.w.writerow([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                        + list(m.pose.covariance) + [time.time()])
        self.n_odom += 1
        wnow = time.time()
        if self.t_first is None:
            self.t_first = t
            self.wall_first = wnow
        self.t_last = t
        self.wall_last = wnow

    def on_img(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.n_img += 1
        if self.img_first is None:
            self.img_first = t
        self.img_last = t

    def summary(self):
        span_o = (self.t_last - self.t_first) if self.t_first else 0.0
        span_i = (self.img_last - self.img_first) if self.img_first else 0.0
        # WALL-CLOCK throughput is the real-time question (header stamps are bag-time
        # and read constant regardless of playback rate). recv_wall is in odom.csv.
        wall = (self.wall_last - self.wall_first) if self.wall_first else 0.0
        print('\n===== cuVSLAM run summary =====')
        print(f'WALL-CLOCK out : {self.n_odom} poses over {wall:.2f}s wall -> {self.n_odom/max(1e-3,wall):.1f} fps REAL throughput')
        print(f'input frames   : {self.n_img}  over {span_i:.2f}s  -> {self.n_img/max(1e-3,span_i):.1f} fps in (bag-time)')
        print(f'odometry poses : {self.n_odom}  over {span_o:.2f}s  -> {self.n_odom/max(1e-3,span_o):.1f} fps out (bag-time)')
        dropped = self.n_img - self.n_odom
        print(f'dropped/skipped: {dropped}  ({100.0*dropped/max(1,self.n_img):.1f}% of input)')
        self.f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--odom-topic', default='/visual_slam/tracking/odometry')
    ap.add_argument('--image-topic', default='/euroc/left/image_rect_raw')
    a = ap.parse_args()
    rclpy.init()
    n = Recorder(a.out, a.odom_topic, a.image_topic)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.summary()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
