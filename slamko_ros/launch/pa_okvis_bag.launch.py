#!/usr/bin/env python3
# P-A live gate (MASTER_PLAN §8): OKVIS2-X as pure VIO odometry on a real D455
# bag + slamko's provider_fusion_node consuming /okvis/okvis_odometry. With no
# global constraints the fused trajectory must track the provider exactly —
# scripts/bench_pa.sh runs this and compares the two TUM dumps.
#
#   ros2 launch slamko_ros pa_okvis_bag.launch.py \
#     bag_path:=/mnt/data/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO \
#     out_dir:=/tmp/slamko_pa
#
# Reuses the PROVEN ~/coding/d455_setup/okvis_d455_casa2_odom.launch.py
# (config rsD455_odom848: do_loop_closures=false, enable_submapping=false; QoS
# override so the bag's BEST_EFFORT IMU reaches OKVIS's reliable subscriber;
# 10 s bag delay for GPU warm-up). The CASA1_* bags carry the same topics as
# casa2 (/camera/camera/infra1|2/image_rect_raw, /camera/camera/imu).
#
# Safety: nothing here publishes /map or fights the production TF tree — the
# fusion node uses slamko_map/slamko_odom/slamko_base frames by default.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

OKVIS_LAUNCH = os.path.expanduser('~/coding/d455_setup/okvis_d455_casa2_odom.launch.py')


def setup(context):
    out_dir = LaunchConfiguration('out_dir').perform(context)
    os.makedirs(out_dir, exist_ok=True)

    okvis = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(OKVIS_LAUNCH),
        launch_arguments={
            'bag_path': LaunchConfiguration('bag_path'),
            'rate':     LaunchConfiguration('rate'),
            'imu_rate': LaunchConfiguration('imu_rate'),
            'csv_path': out_dir + '/okvis/',
            'rviz':     LaunchConfiguration('rviz'),
        }.items())

    # image_topic:='' keeps the P-A behavior; vpr:=true turns on the P-B path
    # (KF images -> EigenPlaces -> sealed VPR submaps in <out_dir>/map).
    vpr_on = LaunchConfiguration('vpr').perform(context).lower() == 'true'
    fusion = Node(
        package='slamko_ros', executable='provider_fusion_node',
        name='provider_fusion_node', output='screen',
        parameters=[{
            'odom_topic': '/okvis/okvis_odometry',
            'traj_fused_path':    out_dir + '/fused.tum',
            'traj_provider_path': out_dir + '/provider.tum',
            'image_topic': '/camera/camera/infra1/image_rect_raw' if vpr_on else '',
            'map_dir': (out_dir + '/map') if vpr_on else '',
            'prior_map_dir': LaunchConfiguration('prior_map_dir').perform(context),
            'traj_global_path': out_dir + '/global.tum',
        }])

    return [okvis, fusion]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_path',
            default_value='/mnt/data/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO'),
        DeclareLaunchArgument('out_dir', default_value='/tmp/slamko_pa'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('imu_rate', default_value='0.0',
            description='OKVIS IMU-propagated odometry rate (0 = per-frame ~26 Hz).'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('vpr', default_value='false',
            description='Capture KF images -> EigenPlaces -> sealed VPR submaps (P-B).'),
        DeclareLaunchArgument('prior_map_dir', default_value='',
            description='Prior smap dir for cross-session relocalization (re-anchor).'),
        OpaqueFunction(function=setup),
    ])
