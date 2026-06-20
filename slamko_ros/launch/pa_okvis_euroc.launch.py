#!/usr/bin/env python3
# EuRoC validation gate: OKVIS2-X (pure VIO) on a EuRoC Machine Hall bag +
# slamko's provider_fusion_node, with ground-truth ATE and optional VIO/VIO+IMU
# BLACKOUT injection (force_loss window -> slamko coasts on DR + seals/branches).
#
#   ros2 launch slamko_ros pa_okvis_euroc.launch.py \
#     bag_path:=/mnt/data/euroc_bags/mh_03_okvis seq:=MH_03_medium \
#     out_dir:=/tmp/slamko_euroc_mh03
#
# Reuses the PROVEN ~/coding/euroc_publisher/launch/okvis_euroc_bag.launch.py
# (config euroc/okvis2.yaml, --clock + use_sim_time, remaps /euroc/cam{0,1} +
# /euroc/imu0 -> /okvis/...). OKVIS undistorts EuRoC's raw radtan images
# INTERNALLY, so the PROVIDER chain (P-A) needs NO rectification. The VPR /
# stereo-landmark path (vpr:=true) needs rectified row-aligned stereo -> use a
# rectified bag for that (Stage 2); leave vpr:=false here for the ATE gate.
#
# EuRoC is OKVIS-stable (3.22 cm ATE MH_03) -> run at rate:=1.0 (the rate<=0.5
# rule is a D455+VPR GPU-contention workaround, not an EuRoC limit).
#
# Blackout modes (force_loss_start/end, bag-relative seconds):
#   imu_topic:=''          -> VIO-only blackout (pure constant-velocity coast)
#   imu_topic:=/euroc/imu0 -> VIO+IMU blackout (gyro-aided DR-gate)
#
# ATE: scripts/euroc_ate_pair.sh <out_dir> <seq>  (Umeyama SE3 vs GT).

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

EUROC_LAUNCH = os.path.expanduser(
    '~/coding/euroc_publisher/launch/okvis_euroc_bag.launch.py')


def setup(context):
    out_dir = LaunchConfiguration('out_dir').perform(context)
    os.makedirs(out_dir + '/okvis', exist_ok=True)
    seq = LaunchConfiguration('seq').perform(context)
    seq_dir = '/mnt/data/datasets/euroc/' + seq

    okvis = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(EUROC_LAUNCH),
        launch_arguments={
            'bag_path': LaunchConfiguration('bag_path'),
            'seq_dir':  seq_dir,
            'csv_path': out_dir + '/okvis/',
            'rate':     LaunchConfiguration('rate'),
            'rviz':     LaunchConfiguration('rviz'),
        }.items())

    # 'none' sentinel = VIO-only blackout (empty launch-arg values are malformed).
    imu_t = LaunchConfiguration('imu_topic').perform(context)
    if imu_t in ('none', 'None', ''):
        imu_t = ''
    vpr_on = LaunchConfiguration('vpr').perform(context).lower() == 'true'
    fusion = Node(
        package='slamko_ros', executable='provider_fusion_node',
        name='provider_fusion_node', output='screen',
        parameters=[{
            'use_sim_time': True,   # the bag plays with --clock
            'odom_topic': '/okvis/okvis_odometry',
            'traj_fused_path':    out_dir + '/fused.tum',
            'traj_provider_path': out_dir + '/provider.tum',
            # EuRoC raw cam0 is 752x480 = XFeat-native (no crop). Only used when vpr:=true.
            'image_topic': '/euroc/cam0/image_raw' if vpr_on else '',
            'right_image_topic': '/euroc/cam1/image_raw',
            'right_info_topic': '/euroc/cam1/camera_info',
            'info_topic': '/euroc/cam0/camera_info',
            'map_dir': (out_dir + '/map') if vpr_on else '',
            'prior_map_dir': LaunchConfiguration('prior_map_dir').perform(context),
            'traj_global_path': out_dir + '/global.tum',
            'traj_graph_path':  out_dir + '/graph.tum',
            'force_loss_start': float(LaunchConfiguration('force_loss_start').perform(context)),
            'force_loss_end':   float(LaunchConfiguration('force_loss_end').perform(context)),
            'imu_topic':    imu_t,
            'dr_gate_path': out_dir + '/dr_gate.csv',
            'viz': LaunchConfiguration('viz').perform(context).lower() == 'true',
            'viz_endpoint': LaunchConfiguration('viz_endpoint').perform(context),
        }])

    return [okvis, fusion]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_path',
            default_value='/mnt/data/euroc_bags/mh_03_okvis'),
        DeclareLaunchArgument('seq', default_value='MH_03_medium',
            description='EuRoC seq name under /mnt/data/datasets/euroc/ (for GT + ATE).'),
        DeclareLaunchArgument('out_dir', default_value='/tmp/slamko_euroc'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('vpr', default_value='false',
            description='Stereo XFeat landmarks + reloc (needs a RECTIFIED bag; Stage 2).'),
        DeclareLaunchArgument('prior_map_dir', default_value='',
            description='Prior smap dir for cross-session relocalization.'),
        DeclareLaunchArgument('imu_topic', default_value='/euroc/imu0',
            description="'' = VIO-only blackout coast; /euroc/imu0 = VIO+IMU DR-gate."),
        DeclareLaunchArgument('viz', default_value='false'),
        DeclareLaunchArgument('viz_endpoint', default_value=''),
        DeclareLaunchArgument('force_loss_start', default_value='-1.0',
            description='Blackout: drop odom from this bag-relative time [s] (-1 = off).'),
        DeclareLaunchArgument('force_loss_end', default_value='-1.0',
            description='Blackout: ... until this bag-relative time [s].'),
        OpaqueFunction(function=setup),
    ])
