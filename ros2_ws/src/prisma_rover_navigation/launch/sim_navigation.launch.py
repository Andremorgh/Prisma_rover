import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Retrieve package share directories
    pkg_sim_share = get_package_share_directory('prisma_rover_sim')
    pkg_localization_share = get_package_share_directory('prisma_rover_localization')
    pkg_navigation_share = get_package_share_directory('prisma_rover_navigation')

    # Launch configurations
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    tf_prefix = LaunchConfiguration('tf_prefix')
    rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    publish_camera = LaunchConfiguration('publish_camera')
    lidar_type = LaunchConfiguration('lidar_type')
    slam_type = LaunchConfiguration('slam_type')

    # 1. Simulation Launch (Includes Robot Description, Spawner, Bridges)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim_share, "launch", "sim.launch.py")),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "namespace": namespace,
            "tf_prefix": tf_prefix,
            "rviz": rviz,
            "headless": headless,
            "publish_camera": publish_camera,
            "lidar_type": lidar_type
        }.items()
    )

    # 2. EKF Localization Launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_localization_share, "launch", "localization.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": namespace,
            "tf_prefix": tf_prefix
        }.items()
    )

    # 3. SLAM Toolbox Launch
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

    # 4. Nav2 Navigation Launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation_share, "launch", "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": namespace
        }.items()
    )

    # 5. TF to Pose Node (from prisma_rover_description)
    # Listens to map -> base_footprint and publishes to namespaced /pose
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
            'world',
            default_value='maze.sdf',
            description='World file name (must exist in prisma_rover_sim/worlds/)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
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
            'rviz',
            default_value='false',
            description='Launch RViz2 if true'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='If true, run simulation server only (no GUI client)'
        ),
        DeclareLaunchArgument(
            'publish_camera',
            default_value='true',
            description='If true, include and bridge camera sensors'
        ),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='2d',
            description='Lidar type to simulate: "2d" (RPLidar) or "3d" (Livox Mid-360)'
        ),
        DeclareLaunchArgument(
            'slam_type',
            default_value='toolbox',
            description='SLAM method: "toolbox" (SLAM Toolbox) or "rtabmap" (RTAB-Map)'
        ),

        sim_launch,
        localization_launch,
        slam_launch,
        navigation_launch,
        tf_to_pose_node
    ])
