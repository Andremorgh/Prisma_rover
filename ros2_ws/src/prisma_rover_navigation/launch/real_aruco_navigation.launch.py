import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Retrieve package share directories
    pkg_navigation = get_package_share_directory('prisma_rover_navigation')
    pkg_aruco = get_package_share_directory('aruco_pose_estimation')

    aruco_params_file = os.path.join(pkg_aruco, 'config', 'aruco_parameters.yaml')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    tf_prefix = LaunchConfiguration('tf_prefix')
    lidar_type = LaunchConfiguration('lidar_type')
    slam_type = LaunchConfiguration('slam_type')

    # 1. Include Real Navigation launch (with camera forced to true)
    real_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation, 'launch', 'real_navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'tf_prefix': tf_prefix,
            'camera': 'true',
            'lidar_type': lidar_type,
            'slam_type': slam_type
        }.items()
    )

    # 2. ArUco Pose Estimation node remapped to real camera topic
    aruco_node = Node(
        package='aruco_pose_estimation',
        executable='aruco_node',
        namespace=namespace,
        parameters=[
            aruco_params_file,
            {
                'use_sim_time': use_sim_time,
                'image_topic': 'camera/color/image_raw',
                'camera_info_topic': 'camera/color/camera_info',
                'camera_frame': [tf_prefix, 'camera_rgb_optical_frame'],
                'map_frame': [tf_prefix, 'map'],
            }
        ],
        output='screen',
        emulate_tty=True
    )

    # Delay aruco_node startup slightly to ensure drivers are fully initialized
    delayed_aruco_node = TimerAction(
        period=5.0,
        actions=[aruco_node]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='prisma_rover',
            description='Top-level namespace for the robot'
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='prisma_rover/',
            description='Prefix to prepend to all TF frames (must end with / if specified)'
        ),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='3d',
            description='Lidar type connected: "2d" (RPLidar) or "3d" (Livox Mid-360)'
        ),
        DeclareLaunchArgument(
            'slam_type',
            default_value='toolbox',
            description='SLAM method: "toolbox" or "rtabmap"'
        ),

        real_nav_launch,
        delayed_aruco_node
    ])
