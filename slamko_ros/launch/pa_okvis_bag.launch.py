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

    # With the VPR path on, our TRT engine deserialization + first inferences
    # overlap OKVIS's CNN warm-up; if the bag starts during that window OKVIS
    # drops enough consecutive frames to go IMU-only and blow up (observed:
    # 673 m on a run with 2657 dropped frames). Give warm-ups 20 s.
    vpr_pre = LaunchConfiguration('vpr').perform(context).lower() == 'true'
    okvis = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(OKVIS_LAUNCH),
        launch_arguments={
            'bag_path': LaunchConfiguration('bag_path'),
            'rate':     LaunchConfiguration('rate'),
            'imu_rate': LaunchConfiguration('imu_rate'),
            'csv_path': out_dir + '/okvis/',
            'rviz':     LaunchConfiguration('rviz'),
            'bag_delay': '20.0' if vpr_pre else '10.0',
            # CRITICAL (2026-06-12): the CASA1_*_BNO bags are 640x480 — the
            # included launch's default config (rsD455_odom848, fx=426) is for
            # the 848 casa2 bag and inflates the trajectory scale ~1.7x.
            'config_dir': LaunchConfiguration('okvis_config').perform(context),
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
            'traj_graph_path':  out_dir + '/graph.tum',
            'force_loss_start': float(LaunchConfiguration('force_loss_start').perform(context)),
            'force_loss_end':   float(LaunchConfiguration('force_loss_end').perform(context)),
            'imu_topic':    '/camera/camera/imu',
            'dr_gate_path': out_dir + '/dr_gate.csv',
            'mag_topic':    '/bno055/mag',
            'compass_csv_path': out_dir + '/compass.csv',
            'mature_out_dir': out_dir + '/matured_prior',
            'viz': LaunchConfiguration('viz').perform(context).lower() == 'true',
            'viz_endpoint': LaunchConfiguration('viz_endpoint').perform(context),
            'compass_yaw_prior':
                LaunchConfiguration('compass_yaw_prior').perform(context).lower() == 'true',
            'occ_refresh':
                LaunchConfiguration('occ_refresh').perform(context).lower() == 'true',
            'mappoint_assoc':
                LaunchConfiguration('mappoint_assoc').perform(context).lower() == 'true',
            'mappoint_refine':
                LaunchConfiguration('mappoint_refine').perform(context).lower() == 'true',
            'mappoint_xsession':
                LaunchConfiguration('mappoint_xsession').perform(context).lower() == 'true',
            'dr_gate_soft_cov':
                LaunchConfiguration('dr_gate_soft_cov').perform(context).lower() == 'true',
            'atlas_break_on_loss':
                LaunchConfiguration('atlas_break_on_loss').perform(context).lower() == 'true',
            'atlas_break_on_quality':
                LaunchConfiguration('atlas_break_on_quality').perform(context).lower() == 'true',
            'quality_soft_bridge':
                LaunchConfiguration('quality_soft_bridge').perform(context).lower() == 'true',
            'loop_min_coverage': float(LaunchConfiguration('loop_min_coverage').perform(context)),
            'proximity_within_session':
                LaunchConfiguration('proximity_within_session').perform(context).lower() == 'true',
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
        DeclareLaunchArgument('viz', default_value='false',
            description='Live Rerun visualizer (needs -DSLAMKO_WITH_RERUN build + a viewer).'),
        DeclareLaunchArgument('viz_endpoint', default_value='',
            description='Rerun endpoint; ".rrd" path = offline file capture, else gRPC viewer.'),
        DeclareLaunchArgument('compass_yaw_prior', default_value='false',
            description='BNO055 absolute-yaw graph prior (gated) — straightens heading drift.'),
        DeclareLaunchArgument('occ_refresh', default_value='true',
            description='P2: refresh occupancy from loop-corrected anchors (fuse revisits).'),
        DeclareLaunchArgument('mappoint_assoc', default_value='false',
            description='Phase A: drift-tolerant cross-submap data association by descriptor.'),
        DeclareLaunchArgument('mappoint_refine', default_value='false',
            description='Phase B: multi-view consensus refine + back-prop (needs mappoint_assoc).'),
        DeclareLaunchArgument('mappoint_xsession', default_value='false',
            description='Phase C: seed the store from the prior map (cross-session dedup).'),
        DeclareLaunchArgument('dr_gate_soft_cov', default_value='false',
            description='#12 trunk: DR-gate uncertainty on the loss-gap chain edge (un-warp).'),
        DeclareLaunchArgument('atlas_break_on_loss', default_value='false',
            description='Etapa 1b: tracking loss -> break into a new disjoint map component.'),
        DeclareLaunchArgument('atlas_break_on_quality', default_value='false',
            description="Etapa 1b': incoherent transition / cov spike -> break (false-traj island)."),
        DeclareLaunchArgument('quality_soft_bridge', default_value='false',
            description='Quality loss -> SOFT bridge (keep chain connected) instead of breaking.'),
        DeclareLaunchArgument('loop_min_coverage', default_value='0.0',
            description='Multi-factor gate: min inliers/submap-landmarks for a loop (0=off).'),
        DeclareLaunchArgument('proximity_within_session', default_value='false',
            description='Close VPR-missed within-session returns geometrically (proximity E).'),
        DeclareLaunchArgument('force_loss_start', default_value='-1.0',
            description='Test: drop odom from this bag-relative time [s] (-1 = off).'),
        DeclareLaunchArgument('force_loss_end', default_value='-1.0',
            description='Test: ... until this bag-relative time [s] -> seal+branch+soft edge.'),
        DeclareLaunchArgument('okvis_config',
            default_value=os.path.expanduser(
                '~/coding/OKVIS2-X/src/OKVIS2-X/config/rsD455_map_odom'),
            description='OKVIS calib config dir — MUST match the bag resolution '
                        '(rsD455_map_odom: 640x480 CASA1_BNO; rsD455_odom848: 848 casa2).'),
        OpaqueFunction(function=setup),
    ])
