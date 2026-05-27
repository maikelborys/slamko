"""Bring up the EuRoC player + slamko_vio_node (sprint 1 monocular tracker)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    seq            = LaunchConfiguration('seq')
    rate           = LaunchConfiguration('rate')
    timing_csv     = LaunchConfiguration('timing_csv')
    max_corners    = LaunchConfiguration('max_corners')
    redetect_thr   = LaunchConfiguration('redetect_threshold')
    patch_size     = LaunchConfiguration('patch_size')
    pyramid_levels = LaunchConfiguration('pyramid_levels')
    pnp_use_cuda   = LaunchConfiguration('pnp_use_cuda')
    lmp_enabled    = LaunchConfiguration('lmp_enabled')
    lm_iters       = LaunchConfiguration('pnp_lm_max_iters')
    second_pass    = LaunchConfiguration('pnp_refine_second_pass')
    landmark_dump  = LaunchConfiguration('landmark_dump_path')
    enable_imu     = LaunchConfiguration('enable_imu')
    dr_enabled     = LaunchConfiguration('dr_enabled')
    dr_loss_start  = LaunchConfiguration('dr_force_loss_start_s')
    dr_loss_end    = LaunchConfiguration('dr_force_loss_end_s')
    feature_source = LaunchConfiguration('feature_source')
    backend        = LaunchConfiguration('backend')

    player = Node(
        package='euroc_publisher',
        executable='euroc_player',
        name='euroc_player',
        output='screen',
        arguments=[
            '--seq',  seq,
            '--rate', rate,
            '--base-frame',  'euroc_base',
            '--imu-frame',   'euroc_imu',
            '--left-frame',  'euroc_left_rect_optical',
            '--right-frame', 'euroc_right_rect_optical',
            '--start-s', LaunchConfiguration('start_s'),
            '--end-s',   LaunchConfiguration('end_s'),
        ],
    )

    klt = Node(
        package='slamko_vio',
        executable='slamko_vio_node',
        name='slamko_vio_node',
        output='screen',
        parameters=[{
            'image_width':        752,
            'image_height':       480,
            'max_corners':        max_corners,
            'redetect_threshold': redetect_thr,
            'patch_size':         patch_size,
            'pyramid_levels':     pyramid_levels,
            'timing_csv_path':    timing_csv,
            'odom_frame_id':      'slamko_vio_world',
            'child_frame_id':     'slamko_vio_cam',
            'publish_tf':         True,
            'pnp_use_cuda':       pnp_use_cuda,
            'lmp_enabled':        lmp_enabled,
            'pnp_lm_max_iters':   lm_iters,
            'pnp_refine_second_pass': second_pass,
            'landmark_dump_path': landmark_dump,
            'enable_imu':         enable_imu,
            'dr_enabled':         dr_enabled,
            'dr_force_loss_start_s': dr_loss_start,
            'dr_force_loss_end_s':   dr_loss_end,
            'feature_source':        feature_source,
            'backend':               backend,
        }],
        remappings=[
            ('left/image_rect_raw',  '/euroc/left/image_rect_raw'),
            ('right/image_rect_raw', '/euroc/right/image_rect_raw'),
            ('left/camera_info',     '/euroc/left/camera_info'),
            ('right/camera_info',    '/euroc/right/camera_info'),
            ('imu',                  '/euroc/imu'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('seq',
            description='Path to EuRoC sequence dir (contains mav0/)'),
        DeclareLaunchArgument('rate',           default_value='1.0'),
        DeclareLaunchArgument('timing_csv',     default_value=''),
        DeclareLaunchArgument('max_corners',    default_value='1500'),
        # Always run the detector each frame; let the dedup radius keep the
        # spatial distribution sane. Detect costs ~50us so this is cheap.
        DeclareLaunchArgument('redetect_threshold', default_value='1500'),
        DeclareLaunchArgument('patch_size',     default_value='9'),
        DeclareLaunchArgument('pyramid_levels', default_value='4'),
        DeclareLaunchArgument('pnp_use_cuda',   default_value='false'),
        DeclareLaunchArgument('lmp_enabled',    default_value='false'),
        DeclareLaunchArgument('pnp_lm_max_iters', default_value='5'),
        DeclareLaunchArgument('pnp_refine_second_pass', default_value='true'),
        DeclareLaunchArgument('landmark_dump_path', default_value=''),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('dr_enabled', default_value='true'),
        DeclareLaunchArgument('dr_force_loss_start_s', default_value='-1.0'),
        DeclareLaunchArgument('dr_force_loss_end_s', default_value='-1.0'),
        DeclareLaunchArgument('feature_source', default_value='shitomasi',
            description='Detector backend: shitomasi | xfeat'),
        DeclareLaunchArgument('backend', default_value='ceres',
            description='Tier-2 fusion backend: ceres | gtsam (gtsam in P1c)'),
        DeclareLaunchArgument('start_s', default_value='0.0',
            description='Crop: skip first N seconds of the sequence'),
        DeclareLaunchArgument('end_s', default_value='1000000',
            description='Crop: stop after this many seconds from start'),
        player,
        klt,
    ])
