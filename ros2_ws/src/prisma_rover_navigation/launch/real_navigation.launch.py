import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Retrieve package share directories
    pkg_bringup_share = get_package_share_directory('prisma_rover_bringup')
    pkg_navigation_share = get_package_share_directory('prisma_rover_navigation')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    tf_prefix = LaunchConfiguration('tf_prefix')
    camera = LaunchConfiguration('camera')
    lidar_type = LaunchConfiguration('lidar_type')
    slam_type = LaunchConfiguration('slam_type')

    # 1. Hardware Bringup (Motor drivers, EKF, Sensors)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup_share, "launch", "hardware_bringup.launch.py")),
        launch_arguments={
            "camera": camera,
            "lidar_type": lidar_type
        }.items()
    )

    # 2. SLAM (SLAM Toolbox or RTAB-Map)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation_share, "launch", "slam.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": namespace,
            "lidar_type": lidar_type,
            "slam_type": slam_type,
            "tf_prefix": tf_prefix
        }.items()
    )

    # 3. Nav2 Navigation Stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation_share, "launch", "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": namespace
        }.items()
    )

    # 4. TF to Pose Node (from prisma_rover_description)
    tf_to_pose_node = Node(
        package='prisma_rover_description',
        executable='tf_to_pose_node',
        name='tf_to_pose',
        namespace=namespace,
        output='screen',
        parameters=[{
            'map_frame': [tf_prefix, 'map'],
            'base_frame': [tf_prefix, 'base_footprint'],
            'pose_topic': 'pose',
            'use_sim_time': use_sim_time
        }]
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
            'camera',
            default_value='true',
            description='If true, launch physical camera'
        ),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='3d',
            description='Lidar type connected: "2d" (RPLidar) or "3d" (Livox Mid-360)'
        ),
        DeclareLaunchArgument(
            'slam_type',
            default_value='toolbox',
            description='SLAM method: "toolbox" (SLAM Toolbox) or "rtabmap" (RTAB-Map)'
        ),

        hardware_launch,
        slam_launch,
        navigation_launch,
        tf_to_pose_node
    ])
