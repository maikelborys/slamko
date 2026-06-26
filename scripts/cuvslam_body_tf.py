#!/usr/bin/env python3
"""cuVSLAM body-frame adapter — the ONE seam fix for the OKVIS->cuVSLAM provider swap.

cuVSLAM (Isaac visual_slam) publishes odometry of `camera_link` (REP-103: x-fwd, z-up).
But slamko places BOTH the sparse XFeat landmarks AND the volumetric depth using extrinsics
(`body_t_cam_xyz`, `depth_extrinsic_xyz`) tuned for OKVIS's body = camera_imu_optical_frame
(IMU, identity rotation to the IR optical frame). Feeding cuVSLAM's camera_link pose directly
makes slamko apply an identity-rotation extrinsic to a frame that is actually rotated by the
optical rpy(-pi/2,0,-pi/2) -> landmarks/depth land ~90 deg wrong -> a silently BENT map
(STACK_MAP_01.md §7, the #1 risk).

Fix: re-express the pose in camera_imu_optical_frame so slamko's OKVIS extrinsics are correct.
  T_odom_imu = T_odom_cameralink * T_cameralink_imu     (T_cameralink_imu = fixed, from URDF/TF)
The fixed transform is looked up from TF (robot_state_publisher of the real D455 URDF) so we
don't re-derive the optical rotation by hand. Covariance + body-twist are rotated by the same R.

  in:  /visual_slam/tracking/odometry   (child = camera_link)
  out: /cuvslam/odometry_body           (child = camera_imu_optical_frame)  -> slamko odom_topic
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

BODY = 'camera_imu_optical_frame'
CL = 'camera_link'


def quat_to_T(q, t):
    T = np.eye(4)
    T[:3, :3] = R.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


class BodyTf(Node):
    def __init__(self):
        super().__init__('cuvslam_body_tf')
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        self.T_cl_imu = None          # fixed camera_link -> imu_optical
        self.R_cl_imu = None
        self.pub = self.create_publisher(Odometry, '/cuvslam/odometry_body', 50)
        self.sub = self.create_subscription(Odometry, '/visual_slam/tracking/odometry',
                                            self.cb, 50)
        self.warned = False

    def lookup(self):
        try:
            tf = self.buf.lookup_transform(CL, BODY, rclpy.time.Time())  # T_cl_imu
            q = tf.transform.rotation
            t = tf.transform.translation
            self.T_cl_imu = quat_to_T([q.x, q.y, q.z, q.w], [t.x, t.y, t.z])
            self.R_cl_imu = self.T_cl_imu[:3, :3]
            self.get_logger().info(f'got fixed {CL}->{BODY}: t={self.T_cl_imu[:3,3].round(4)} '
                                   f'rpy={R.from_matrix(self.R_cl_imu).as_euler("xyz",degrees=True).round(1)}')
            return True
        except Exception as e:
            if not self.warned:
                self.get_logger().warn(f'waiting for TF {CL}->{BODY} (robot_state_publisher): {e}')
                self.warned = True
            return False

    def cb(self, m):
        if self.T_cl_imu is None and not self.lookup():
            return
        p, q = m.pose.pose.position, m.pose.pose.orientation
        T_odom_cl = quat_to_T([q.x, q.y, q.z, q.w], [p.x, p.y, p.z])
        T_odom_imu = T_odom_cl @ self.T_cl_imu
        Ri = self.R_cl_imu

        out = Odometry()
        out.header = m.header
        out.child_frame_id = BODY
        qi = R.from_matrix(T_odom_imu[:3, :3]).as_quat()
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = T_odom_imu[:3, 3]
        out.pose.pose.orientation.x, out.pose.pose.orientation.y, \
            out.pose.pose.orientation.z, out.pose.pose.orientation.w = qi
        # covariance: rotate the trans/rot 3x3 blocks by R (block-diag adjoint; ignores the
        # small lever-arm coupling). Trace-preserving -> slamko's quality multiplier intact.
        C = np.array(m.pose.covariance).reshape(6, 6)
        Cn = C.copy()
        Cn[:3, :3] = Ri @ C[:3, :3] @ Ri.T
        Cn[3:, 3:] = Ri @ C[3:, 3:] @ Ri.T
        out.pose.covariance = Cn.flatten().tolist()
        # body twist (velocity in child frame) -> rotate into the new body axes.
        v = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        w = np.array([m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z])
        vi, wi = Ri.T @ v, Ri.T @ w
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = vi
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = wi
        Tw = np.array(m.twist.covariance).reshape(6, 6)
        Twn = Tw.copy()
        Twn[:3, :3] = Ri.T @ Tw[:3, :3] @ Ri
        Twn[3:, 3:] = Ri.T @ Tw[3:, 3:] @ Ri
        out.twist.covariance = Twn.flatten().tolist()
        self.pub.publish(out)


def main():
    rclpy.init()
    n = BodyTf()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
