#!/usr/bin/env python3
# P-E first light: klt_vo as the odometry provider (instead of OKVIS2-X) on a
# D455 casa bag + slamko's provider_fusion_node — the SAME loose contract,
# different provider, proving the pluggability the rebuild promised.
#
#   ros2 launch slamko_ros pa_kltvo_bag.launch.py \
#     bag_path:=/mnt/data/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO \
#     out_dir:=/tmp/slamko_kltvo
#
# Provider specifics (vs the OKVIS launch):
# - odometry topic /klt_vo/odometry; the pose is the CAMERA pose T_WC
#   (klt_vo_node.cpp:767), so the fusion node runs body_T_cam = IDENTITY.
# - klt_vo subscribes the IMU RELIABLE -> the bag's BEST_EFFORT IMU needs the
#   same QoS override as OKVIS (casa2_qos_override.yaml).
# - klt_vo workspace: ~/ros2_ws (source its setup before launching).
# - casa1/BNO bags are 640x480 (the launch default); casa2 is 848x480.

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

KLT_LAUNCH = os.path.expanduser(
    '~/coding/klt_vo/klt_vo_node/launch/klt_vo_d455.launch.py')
QOS = os.path.expanduser('~/coding/d455_setup/casa2_qos_override.yaml')


def setup(context):
    out_dir = LaunchConfiguration('out_dir').perform(context)
    os.makedirs(out_dir, exist_ok=True)
    vpr_on = LaunchConfiguration('vpr').perform(context).lower() == 'true'
    prior = LaunchConfiguration('prior_map_dir').perform(context)

    klt = IncludeLaunchDescription(AnyLaunchDescriptionSource(KLT_LAUNCH))

    fusion = Node(
        package='slamko_ros', executable='provider_fusion_node',
        name='provider_fusion_node', output='screen',
        parameters=[{
            'odom_topic': '/klt_vo/odometry',
            'body_t_cam_xyz': [0.0, 0.0, 0.0],  # klt_vo pose IS the camera
            'traj_fused_path':    out_dir + '/fused.tum',
            'traj_provider_path': out_dir + '/provider.tum',
            'traj_global_path':   out_dir + '/global.tum',
            'image_topic': '/camera/camera/infra1/image_rect_raw' if vpr_on else '',
            'map_dir': (out_dir + '/map') if vpr_on else '',
            'prior_map_dir': prior,
        }])

    play = TimerAction(
        period=15.0,  # klt_vo + our TRT engines warm up (engines cached)
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play',
                 LaunchConfiguration('bag_path').perform(context),
                 '--rate', LaunchConfiguration('rate').perform(context),
                 '--qos-profile-overrides-path', QOS,
                 '--remap', '/tf_static:=/_bag_tf_static_unused'],
            output='screen')])

    return [klt, fusion, play]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_path',
            default_value='/mnt/data/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO'),
        DeclareLaunchArgument('out_dir', default_value='/tmp/slamko_kltvo'),
        DeclareLaunchArgument('rate', default_value='0.5'),
        DeclareLaunchArgument('vpr', default_value='true'),
        DeclareLaunchArgument('prior_map_dir', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),  # accepted, unused
        OpaqueFunction(function=setup),
    ])
