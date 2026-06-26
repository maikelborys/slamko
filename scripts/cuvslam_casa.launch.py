"""cuVSLAM (Isaac ROS visual_slam) on a replayed D455 casa bag — ODOMETRY-ONLY (loop
closure OFF, slamko owns global), VIO mode.

Frame tree: robot_state_publisher loads the REAL D455 URDF (scripts/d455.urdf, nominal
extrinsics) so cuVSLAM gets the factory frame tree — camera_link (REP-103: x-fwd, z-up)
-> all optical frames with the standard rpy(-pi/2,0,-pi/2) rotation, IMU @ (-0.01602,
-0.03022, 0.0074). base_frame=camera_link (NOT an optical frame — that was the bug that
made VIO diverge: an optical-convention base confuses cuVSLAM's gravity model).
imu_frame=camera_imu_optical_frame (URDF defines it == gyro_optical_frame). This mirrors
the known-good live chain_cuvslam config (base=camera_link, imu=gyro).

  CUVSLAM_MODE=1 ros2 launch (this file)   # 1=VIO(IMU), 0=stereo-only ; play bag separately
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'd455.urdf')
L = 'camera_infra1_optical_frame'
R = 'camera_infra2_optical_frame'

# Image/camera_info topics (env-overridable so the SAME launch works on raw infra bags
# AND on the flashing depth bag via the splitter's clean /okvis/cam0,cam1 + injected info).
IMG0 = os.environ.get('CUVSLAM_IMG0', '/camera/camera/infra1/image_rect_raw')
IMG1 = os.environ.get('CUVSLAM_IMG1', '/camera/camera/infra2/image_rect_raw')
CI0 = os.environ.get('CUVSLAM_CI0', '/camera/camera/infra1/camera_info')
CI1 = os.environ.get('CUVSLAM_CI1', '/camera/camera/infra2/camera_info')


def generate_launch_description():
    mode = int(os.environ.get('CUVSLAM_MODE', '1'))   # 1=VIO(IMU), 0=stereo-only(no IMU)
    nmul = float(os.environ.get('CUVSLAM_IMU_NOISE_MUL', '1.0'))  # >1 = trust IMU LESS
    with open(URDF) as f:
        robot_desc = f.read()

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='d455_rsp', output='log',
               parameters=[{'robot_description': robot_desc}])

    vslam = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': False,
            'rectified_images': True,
            'tracking_mode': mode,                    # 1=VIO (IMU fusion), 0=stereo-only
            'enable_localization_n_mapping': False,   # ODOMETRY-ONLY: loop closure OFF
            'num_cameras': 2,
            'base_frame': 'camera_link',              # REP-103 body (NOT optical) — fixes VIO divergence
            'imu_frame': 'camera_imu_optical_frame',  # == gyro_optical in the URDF
            'camera_optical_frames': [L, R],
            # IMU noise = the EXACT values OKVIS uses for THIS D455 (rsD455_odom848:
            # sigma_g_c/sigma_a_c/sigma_gw_c/sigma_aw_c). cuVSLAM's package defaults
            # (0.000244/0.001862/...) over-trust the D455 IMU 11-13x -> gradual VIO
            # divergence; OKVIS already tunes them looser and is robust. The IMU is fine,
            # cuVSLAM's defaults were wrong for this unit. nmul left as an extra knob.
            'gyro_noise_density': 0.00278 * nmul,
            'gyro_random_walk': 0.0008 * nmul,
            'accel_noise_density': 0.0252 * nmul,
            'accel_random_walk': 0.04 * nmul,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 60.0,
            'enable_slam_visualization': False,
            'enable_landmarks_view': False,
            'enable_observations_view': False,
        }],
        remappings=[
            ('visual_slam/image_0', IMG0),
            ('visual_slam/camera_info_0', CI0),
            ('visual_slam/image_1', IMG1),
            ('visual_slam/camera_info_1', CI1),
            ('visual_slam/imu', '/camera/camera/imu'),
        ],
    )
    container = ComposableNodeContainer(
        name='cuvslam_container', namespace='', package='rclcpp_components',
        executable='component_container', composable_node_descriptions=[vslam],
        output='screen')

    return LaunchDescription([rsp, container])
