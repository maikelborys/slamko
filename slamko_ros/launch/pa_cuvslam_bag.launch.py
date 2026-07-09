#!/usr/bin/env python3
# P-A gate for the cuVSLAM provider (mirror of pa_okvis_bag.launch.py): the
# slamko_vio cuvslam_provider_node as pure odometry on a real D455 bag +
# provider_fusion_node consuming /cuvslam/odometry. With no global constraints
# the fused trajectory must track the provider exactly.
#
#   ros2 launch slamko_ros pa_cuvslam_bag.launch.py \
#     bag_path:=/mnt/data/bags/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO \
#     out_dir:=/tmp/slamko_pa_cuvslam
#
# cuVSLAM needs no config file: the rig is built from the bag's own camera_info
# (any resolution — the 848-vs-640 OKVIS config trap does not exist here).
# GPU warm-up is seconds, not the OKVIS 10-20 s; bag_delay default 6 s covers it
# (raise to 20 if vpr:=true — the TRT reloc engines warm up in the fusion node).
#
# Safety: nothing here publishes /map or fights the production TF tree.
# First place to look when it misbehaves: the cuvslam_provider "rig ready" line
# (missing = camera_info never arrived), then /cuvslam/health suspect ratio.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    out_dir = LaunchConfiguration('out_dir').perform(context)
    os.makedirs(out_dir, exist_ok=True)
    vpr_on = LaunchConfiguration('vpr').perform(context).lower() == 'true'
    delay = float(LaunchConfiguration('bag_delay').perform(context))

    provider = Node(
        package='slamko_vio', executable='cuvslam_provider_node',
        name='cuvslam_provider', output='screen',
        parameters=[{
            'image_best_effort': True,
            'async_sba': True,
            'min_inliers': int(LaunchConfiguration('min_inliers').perform(context)),
            'max_info_condition': float(LaunchConfiguration('max_info_condition').perform(context)),
            'cov_scale': float(LaunchConfiguration('cov_scale').perform(context)),
            'suspect_cov_mult': float(LaunchConfiguration('suspect_cov_mult').perform(context)),
            'use_imu': LaunchConfiguration('use_imu').perform(context).lower() == 'true',
            'imu_scale': float(LaunchConfiguration('imu_scale').perform(context)),
        }])

    fusion = Node(
        package='slamko_ros', executable='provider_fusion_node',
        name='provider_fusion_node', output='screen',
        parameters=[{
            'odom_topic': '/cuvslam/odometry',
            'traj_fused_path':    out_dir + '/fused.tum',
            'traj_provider_path': out_dir + '/provider.tum',
            'image_topic': '/camera/camera/infra1/image_rect_raw' if vpr_on else '',
            'map_dir': (out_dir + '/map') if vpr_on else '',
            'prior_map_dir': LaunchConfiguration('prior_map_dir').perform(context),
            'traj_global_path': out_dir + '/global.tum',
            'traj_graph_path':  out_dir + '/graph.tum',
            'imu_topic':    '/camera/camera/imu',
            'dr_gate_path': out_dir + '/dr_gate.csv',
            'atlas_break_on_loss': True,
            'atlas_break_on_quality': True,
            'quality_soft_bridge': True,
            'mag_topic': '/bno055/mag',
            'compass_yaw_prior':
                LaunchConfiguration('compass_yaw_prior').perform(context).lower() == 'true',
        }])

    bag = TimerAction(period=delay, actions=[ExecuteProcess(
        cmd=['ros2', 'bag', 'play',
             LaunchConfiguration('bag_path').perform(context),
             '--rate', LaunchConfiguration('rate').perform(context)],
        output='log')])

    return [provider, fusion, bag]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_path',
            default_value='/mnt/data/bags/bno_ab/CASA1_Suave_Stereo60_RGB30_BNO'),
        DeclareLaunchArgument('out_dir', default_value='/tmp/slamko_pa_cuvslam'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('bag_delay', default_value='6.0'),
        DeclareLaunchArgument('vpr', default_value='false'),
        DeclareLaunchArgument('prior_map_dir', default_value=''),
        DeclareLaunchArgument('use_imu', default_value='true',
            description='cuVSLAM Inertial (VIO) mode — DEFAULT since the 2026-07-09 A/B: '
                        'fixes the stereo-only stairs under-scale (6.6%->1.7%), no flat regression.'),
        DeclareLaunchArgument('imu_scale', default_value='1.0',
            description='Accel scale fix (0.5 for the doubled-accel casa1-original bags).'),
        DeclareLaunchArgument('compass_yaw_prior', default_value='false',
            description='BNO055 absolute-yaw graph prior in the fusion node (gated).'),
        DeclareLaunchArgument('min_inliers', default_value='10'),
        DeclareLaunchArgument('max_info_condition', default_value='1e6'),
        DeclareLaunchArgument('cov_scale', default_value='80.0'),
        DeclareLaunchArgument('suspect_cov_mult', default_value='25.0'),
        OpaqueFunction(function=setup),
    ])
