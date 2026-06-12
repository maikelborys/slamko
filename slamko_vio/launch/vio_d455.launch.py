#!/usr/bin/env python3
# slamko_vio on an Intel RealSense D455 — stereo IR (infra1/infra2) + IMU.
#
# Works for BOTH:
#   • LIVE camera: first bring up the driver with the emitter OFF (the IR dot
#     pattern corrupts KLT/stereo), e.g.
#       ros2 launch realsense2_camera rs_launch.py \
#         enable_infra1:=true enable_infra2:=true enable_gyro:=true enable_accel:=true \
#         unite_imu_method:=2 depth_module.emitter_enabled:=0 \
#         depth_module.infra_profile:=848x480x60 enable_sync:=true
#     then: ros2 launch slamko_vio vio_d455.launch.py
#   • BAG replay (validation, no hardware):
#       ros2 launch slamko_vio vio_d455.launch.py bag_path:=/mnt/data/d455_bags/casa2
#
# Calibration: K + baseline come from the bag/driver's infra1/infra2 camera_info
# (D455 848x480 IR is already rectified, D=0). The cam→imu extrinsic T_BS is NOT
# in the D455 TF tree the way slamko expects, so we publish a static
# euroc_imu → camera_infra1_optical_frame from the OKVIS rsD455 T_SC (cam0):
#   translation (-0.03022, 0.0074, 0.01602) m, identity rotation.
# IMU is BEST_EFFORT on the D455 → imu_best_effort:=true (a reliable subscriber
# would receive zero IMU and drop every frame).
#
# Validated offline on casa2 (real D455 IR house walk): tracks at inlier ~0.67,
# gravity init clean (|a_mean|=8.94 ⇒ the D455 s_a≈1.0948 accel scale; |g| locked
# to 9.81), ~6 m trajectory, no divergence.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CAM = '/camera/camera'
INFRA1 = CAM + '/infra1/image_rect_raw'
INFRA2 = CAM + '/infra2/image_rect_raw'
INFO1  = CAM + '/infra1/camera_info'
INFO2  = CAM + '/infra2/camera_info'
IMU    = CAM + '/imu'
CAM_FRAME = 'camera_infra1_optical_frame'   # = infra1 camera_info.frame_id


def generate_launch_description():
    feature_source   = LaunchConfiguration('feature_source')
    estimate_gravity = LaunchConfiguration('estimate_gravity')
    bag_path         = LaunchConfiguration('bag_path')
    rate             = LaunchConfiguration('rate')

    # cam0 (infra1) → imu (= body 'euroc_imu'): OKVIS rsD455 T_SC cam0, identity rot.
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='d455_imu_to_cam',
        arguments=['--x', '-0.03022', '--y', '0.0074', '--z', '0.01602',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'euroc_imu', '--child-frame-id', CAM_FRAME])

    vio = Node(
        package='slamko_vio', executable='slamko_vio_node', name='slamko_vio_node',
        output='screen',
        parameters=[{
            'image_width':   848,
            'image_height':  480,
            'enable_imu':    True,
            'imu_best_effort': True,          # D455 IMU is BEST_EFFORT
            'imu_rate_hz':   200.0,
            'estimate_gravity': ParameterValue(estimate_gravity, value_type=bool),
            'feature_source': feature_source,
            'publish_tf':    True,
        }],
        remappings=[
            ('left/image_rect_raw',  INFRA1),
            ('right/image_rect_raw', INFRA2),
            ('left/camera_info',     INFO1),
            ('right/camera_info',    INFO2),
            ('imu',                  IMU),
        ])

    # Optional bag replay (validation). bag_path='' → live mode (driver runs separately).
    play = GroupAction(
        condition=IfCondition(PythonExpression(["'", bag_path, "' != ''"])),
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--rate', rate],
            output='screen')])

    return LaunchDescription([
        DeclareLaunchArgument('feature_source',   default_value='shitomasi'),
        DeclareLaunchArgument('estimate_gravity', default_value='true'),
        DeclareLaunchArgument('bag_path', default_value='',
            description="D455 bag to replay (raw /camera/camera topics). Empty = live driver."),
        DeclareLaunchArgument('rate', default_value='1.0'),
        static_tf,
        vio,
        play,
    ])
