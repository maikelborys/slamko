"""Bring up the EuRoC player + slamko_vio_node (sprint 1 monocular tracker)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def _bool(name):
    # Force string→bool coercion: a bare LaunchConfiguration resolves to a string,
    # which a bool-typed node param silently rejects (the param keeps its default).
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


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
    pose_dump      = LaunchConfiguration('pose_dump_path')

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
            'image_width':        ParameterValue(LaunchConfiguration('image_width'),  value_type=int),
            'image_height':       ParameterValue(LaunchConfiguration('image_height'), value_type=int),
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
            'xfeat_keypoint_threshold': ParameterValue(LaunchConfiguration('xfeat_keypoint_threshold'), value_type=float),
            'backend':               backend,
            'pose_dump_path':        pose_dump,
            'enable_neverlost':      _bool('enable_neverlost'),
            'dr_force_loss_windows':    LaunchConfiguration('dr_force_loss_windows'),
            'neverlost_use_pose_graph': _bool('neverlost_use_pose_graph'),
            'neverlost_weld_once':      _bool('neverlost_weld_once'),
            'neverlost_continuous_reloc': _bool('neverlost_continuous_reloc'),
            'neverlost_auto_seal_distance_m': ParameterValue(LaunchConfiguration('neverlost_auto_seal_distance_m'), value_type=float),
            'neverlost_weld_ba':              _bool('neverlost_weld_ba'),
            'neverlost_weld_ba_max_iters':    ParameterValue(LaunchConfiguration('neverlost_weld_ba_max_iters'), value_type=int),
            'neverlost_weld_ba_pixel_sigma':  ParameterValue(LaunchConfiguration('neverlost_weld_ba_pixel_sigma'), value_type=float),
            'reloc_match_ratio':        ParameterValue(LaunchConfiguration('reloc_match_ratio'), value_type=float),
            'reloc_use_bow':            ParameterValue(LaunchConfiguration('reloc_use_bow'), value_type=bool),
            'reloc_bow_top_k':          ParameterValue(LaunchConfiguration('reloc_bow_top_k'), value_type=int),
            'reloc_mutual_check':       ParameterValue(LaunchConfiguration('reloc_mutual_check'), value_type=bool),
            'reloc_min_inlier_ratio':   ParameterValue(LaunchConfiguration('reloc_min_inlier_ratio'), value_type=float),
            'reloc_min_inliers':        ParameterValue(LaunchConfiguration('reloc_min_inliers'), value_type=int),
            'reloc_use_lightglue':      ParameterValue(LaunchConfiguration('reloc_use_lightglue'), value_type=bool),
            'reloc_lightglue_model':    LaunchConfiguration('reloc_lightglue_model'),
            'reloc_lg_max_views':       ParameterValue(LaunchConfiguration('reloc_lg_max_views'), value_type=int),
            'reloc_vpr_top_n':          ParameterValue(LaunchConfiguration('reloc_vpr_top_n'), value_type=int),
            'prior_map_dir':            LaunchConfiguration('prior_map_dir'),
            'map_save_dir':             LaunchConfiguration('map_save_dir'),
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
        DeclareLaunchArgument('image_width',  default_value='752',
            description='Image width  (752 EuRoC · 512 for rectified TUM VI). Must match the published frames or every frame is dropped at the node size-gate.'),
        DeclareLaunchArgument('image_height', default_value='480',
            description='Image height (480 EuRoC · 512 for rectified TUM VI).'),
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
        DeclareLaunchArgument('xfeat_keypoint_threshold', default_value='0.05',
            description='XFeat keypoint-head score threshold. Lower (e.g. 0.01) to recover keypoints on dark/low-contrast frames like TUM VI.'),
        DeclareLaunchArgument('backend', default_value='ceres',
            description='Tier-2 fusion backend: ceres | gtsam (gtsam in P1c)'),
        DeclareLaunchArgument('pose_dump_path', default_value='',
            description='If set, node writes per-frame world pose as TUM (offline ATE)'),
        DeclareLaunchArgument('enable_neverlost', default_value='false',
            description='P2c: run the Tier-3 never-lost supervisor + relocalizer in-process'),
        DeclareLaunchArgument('dr_force_loss_windows', default_value='',
            description='Extra forced-loss windows "s:e,s:e" (sec, rel) → multiple seals'),
        DeclareLaunchArgument('neverlost_use_pose_graph', default_value='false',
            description='P2.5: route the weld through the SE3 pose-graph backend (multi-submap merge)'),
        DeclareLaunchArgument('neverlost_weld_once', default_value='true',
            description='Weld at most once per sealed target per recovery episode'),
        DeclareLaunchArgument('neverlost_continuous_reloc', default_value='false',
            description='P4b-2: attempt welds in OK (localize into prior map / close loops w/o getting lost)'),
        DeclareLaunchArgument('neverlost_auto_seal_distance_m', default_value='0.0',
            description='Periodic auto-seal every N m of travel (OK state) so loops can close on a clean long traversal (0=off). Pairs with neverlost_continuous_reloc:=true.'),
        DeclareLaunchArgument('neverlost_weld_ba', default_value='false',
            description='C.live V0: on every weld, run a visual+IMU BA over the ACTIVE submap (Phase B.2 substrate); refines KFs+landmarks intra-submap so the next weld\'s AnchorGate clusters tighter. Costs ~100-300 ms per weld. Off by default until benchmarked end-to-end.'),
        DeclareLaunchArgument('neverlost_weld_ba_max_iters', default_value='20'),
        DeclareLaunchArgument('neverlost_weld_ba_pixel_sigma', default_value='1.0'),
        DeclareLaunchArgument('reloc_match_ratio', default_value='0.9',
            description='Relocalizer Lowe ratio (higher = more matches; XFeat is weakly discriminative indoors).'),
        DeclareLaunchArgument('reloc_use_bow', default_value='true',
            description='BoW candidate pre-selection. false = PnP all submaps (catches loops the per-submap BoW filter drops).'),
        DeclareLaunchArgument('reloc_bow_top_k', default_value='25',
            description='BoW candidate submaps PnP-verified per query.'),
        DeclareLaunchArgument('reloc_mutual_check', default_value='false',
            description='Symmetric (cross-check) NN matching — extra precision.'),
        DeclareLaunchArgument('reloc_min_inlier_ratio', default_value='0.0',
            description='Reject a weld if PnP inliers/matches < this (0=off; OKVIS uses ~0.7).'),
        DeclareLaunchArgument('reloc_min_inliers', default_value='15',
            description='Min PnP inliers (and min putative matches) to accept a relocalization.'),
        DeclareLaunchArgument('reloc_use_lightglue', default_value='false',
            description='LighterGlue verification matcher (viewpoint-robust; closes hard revisits XFeat-NN cannot). Needs slamko_loop built with -DSLAMKO_LOOP_WITH_TORCH; falls back to brute force if the model fails to load.'),
        DeclareLaunchArgument('reloc_lightglue_model', default_value='',
            description='Path to lighterglue.pt (empty = share/models/lighterglue.pt).'),
        DeclareLaunchArgument('reloc_lg_max_views', default_value='4',
            description='Keyframe poses each candidate submap is projected into for LighterGlue matching.'),
        DeclareLaunchArgument('reloc_vpr_top_n', default_value='10',
            description='VPR candidate breadth — # submaps PnP-verified per query. Push >10 on hard revisits where VPR may not rank the loop target top-10 (LightGlue+PnP absorbs more candidates without false positives).'),
        DeclareLaunchArgument('prior_map_dir', default_value='',
            description='P4: load a prior Atlas (cross-session) from this dir at startup'),
        DeclareLaunchArgument('map_save_dir', default_value='',
            description='P4: save the sealed Atlas to this dir at shutdown'),
        DeclareLaunchArgument('start_s', default_value='0.0',
            description='Crop: skip first N seconds of the sequence'),
        DeclareLaunchArgument('end_s', default_value='1000000',
            description='Crop: stop after this many seconds from start'),
        player,
        klt,
    ])
