#!/usr/bin/env python3
# EuRoC CROSS-SESSION fusion (Stage 2): OKVIS2-X on a EuRoC bag (raw, internally
# undistorted) + a live stereo-RECTIFY node feeding slamko's XFeat stereo-landmark
# + relocalization path. Session 1 maps a segment; session 2 replays with
# prior_map_dir -> relocalizes/proximity-merges against session 1 (the user's
# "ver como se fusiona"). Composes bag_play (with --start-offset for mid->end) +
# OKVIS + rectify + provider_fusion directly (the included euroc launch can't offset).
#
#   ros2 launch slamko_ros pa_okvis_euroc_x.launch.py \
#     bag_path:=/mnt/data/euroc_bags/mh_03_okvis seq:=MH_03_medium \
#     out_dir:=/tmp/s1 start_offset:=67.0                       # session 1: mid->end
#   ros2 launch slamko_ros pa_okvis_euroc_x.launch.py \
#     bag_path:=/mnt/data/euroc_bags/mh_03_okvis seq:=MH_03_medium \
#     out_dir:=/tmp/s2 prior_map_dir:=/tmp/s1/map               # session 2: full + prior
#
# OKVIS eats /euroc/cam{0,1}/image_raw (raw radtan, its own calib). slamko eats
# /euroc/cam{0,1}/image_rect (752x480 rectified pinhole) from euroc_rectify_node.py.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RECTIFY = os.path.expanduser('~/coding/slamko/scripts/euroc_rectify_node.py')


def setup(context):
    out_dir = LaunchConfiguration('out_dir').perform(context)
    os.makedirs(out_dir + '/okvis', exist_ok=True)
    seq = LaunchConfiguration('seq').perform(context)
    bag = LaunchConfiguration('bag_path').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    start_off = LaunchConfiguration('start_offset').perform(context)

    okvis_share = get_package_share_directory('okvis')
    cfg = os.path.join(okvis_share, '..', '..', '..', '..',
                       'src', 'OKVIS2-X', 'config', 'euroc', 'okvis2.yaml')
    se = os.path.join(okvis_share, '..', '..', '..', '..',
                      'src', 'OKVIS2-X', 'config', 'euroc', 'se2.yaml')

    play_cmd = ['ros2', 'bag', 'play', bag, '--rate', rate, '--clock']
    if float(start_off) > 0.0:
        play_cmd += ['--start-offset', start_off]
    # Give OKVIS + the TRT engines a warm-up before frames flow.
    bag_play = ExecuteProcess(cmd=['bash', '-c',
        'sleep 12; exec ' + ' '.join(play_cmd)], output='screen')

    okvis = Node(
        package='okvis', executable='okvis2x_stereo_network_node_subscriber',
        name='okvis', namespace='okvis', output='screen',
        additional_env={'OMP_NUM_THREADS': '2'},
        parameters=[{
            'config_filename': cfg, 'se_config_filename': se,
            'csv_path': out_dir + '/okvis/',
            'mesh_cutoff_z': 2.5, 'save_submap_meshes': False,
            'use_sim_time': True, 'prior_map_path': '', 'localization_only': False,
        }],
        remappings=[('/okvis/cam0/image_raw', '/euroc/cam0/image_raw'),
                    ('/okvis/cam1/image_raw', '/euroc/cam1/image_raw'),
                    ('/okvis/imu0', '/euroc/imu0')])

    rectify = ExecuteProcess(
        cmd=['python3', RECTIFY, '--seq', seq, '--ros-args',
             '-p', 'use_sim_time:=true'], output='screen')

    fusion = Node(
        package='slamko_ros', executable='provider_fusion_node',
        name='provider_fusion_node', output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': '/okvis/okvis_odometry',
            'traj_fused_path':    out_dir + '/fused.tum',
            'traj_provider_path': out_dir + '/provider.tum',
            'traj_global_path':   out_dir + '/global.tum',
            'traj_graph_path':    out_dir + '/graph.tum',
            # slamko consumes the RECTIFIED stream (752x480, pinhole).
            'image_topic':       '/euroc/cam0/image_rect',
            'right_image_topic': '/euroc/cam1/image_rect',
            'left_info_topic':   '/euroc/cam0/camera_info_rect',
            'right_info_topic':  '/euroc/cam1/camera_info_rect',
            'map_dir': out_dir + '/map',   # loop_assoc.csv auto-written here
            'prior_map_dir': LaunchConfiguration('prior_map_dir').perform(context),
            'imu_topic': '/euroc/imu0',
            'dr_gate_path': out_dir + '/dr_gate.csv',
            'atlas_break_on_loss':
                LaunchConfiguration('atlas_break_on_loss').perform(context).lower() == 'true',
            'force_loss_start': float(LaunchConfiguration('force_loss_start').perform(context)),
            'force_loss_end':   float(LaunchConfiguration('force_loss_end').perform(context)),
            'viz': LaunchConfiguration('viz').perform(context).lower() == 'true',
            'viz_endpoint': LaunchConfiguration('viz_endpoint').perform(context),
        }])

    return [okvis, rectify, fusion, bag_play]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value='/mnt/data/euroc_bags/mh_03_okvis'),
        DeclareLaunchArgument('seq', default_value='MH_03_medium'),
        DeclareLaunchArgument('out_dir', default_value='/tmp/slamko_euroc_x'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start_offset', default_value='0.0',
            description='Bag-relative start [s] — mid->end session 1 (0 = full).'),
        DeclareLaunchArgument('prior_map_dir', default_value='',
            description='Session-1 map dir for cross-session relocalization.'),
        DeclareLaunchArgument('atlas_break_on_loss', default_value='false',
            description='Atlas: a tracking loss (or injected blackout) breaks into a new map.'),
        DeclareLaunchArgument('force_loss_start', default_value='-1.0',
            description='Inject a blackout: drop odom from this bag-relative time [s] (-1=off).'),
        DeclareLaunchArgument('force_loss_end', default_value='-1.0',
            description='... until this bag-relative time [s] -> stale-gap -> atlas break.'),
        DeclareLaunchArgument('viz', default_value='false'),
        DeclareLaunchArgument('viz_endpoint', default_value=''),
        OpaqueFunction(function=setup),
    ])
