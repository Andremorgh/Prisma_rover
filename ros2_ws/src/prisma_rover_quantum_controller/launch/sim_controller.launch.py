from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('prisma_rover_quantum_controller')
    config_dir = os.path.join(pkg_share, 'config')

    # Launch Configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_sim = LaunchConfiguration('launch_sim')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    publish_camera = LaunchConfiguration('publish_camera')

    # --------------------- Optional Simulation & Localization Boot ---------------------
    # Only triggered if launch_sim is true
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('prisma_rover_sim'),
                'launch',
                'sim.launch.py'
            ])
        ),
        condition=IfCondition(launch_sim),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'tf_prefix': [namespace, '/'],
            'rviz': rviz,
            'headless': headless,
            'publish_camera': publish_camera,
            'lidar_type': '3d'  # PointCloud 3D required by Lidar Listener
        }.items()
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('prisma_rover_localization'),
                'launch',
                'localization.launch.py'
            ])
        ),
        condition=IfCondition(launch_sim),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'tf_prefix': [namespace, '/']
        }.items()
    )

    tf_to_pose_node = Node(
        package='prisma_rover_description',
        executable='tf_to_pose_node',
        name='tf_to_pose',
        namespace=namespace,
        output='screen',
        condition=IfCondition(launch_sim),
        parameters=[{
            'map_frame': [namespace, '/map'],
            'base_frame': [namespace, '/base_footprint'],
            'pose_topic': 'pose',
            'use_sim_time': use_sim_time
        }]
    )

    static_map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', [namespace, '/map'], [namespace, '/odom']],
        condition=IfCondition(launch_sim)
    )

    return LaunchDescription([
        # Declare Launch Arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='prisma_rover',
            description='Top-level namespace for the nodes'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'launch_sim',
            default_value='false',
            description='If true, launch Gazebo simulation, EKF localization, and tf_to_pose node'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='If running Gazebo, start GUI client if false'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz2 if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='depot.sdf',
            description='Gazebo world file to load'
        ),
        DeclareLaunchArgument(
            'publish_camera',
            default_value='true',
            description='If true, include and bridge camera sensors'
        ),

        # Optional Simulator + Localization components
        sim_launch,
        localization_launch,
        static_map_to_odom_tf,
        tf_to_pose_node,

        # Lidar listener (configured for simulation - 3D scan)
        Node(
            package='prisma_rover_quantum_controller',
            executable='lidar_listener',
            name='lidar_listener',
            namespace=namespace,
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[{
                'use_sim_time': use_sim_time,
                'lidar_topic': 'scan_3d',
                'elevation_band_deg': 5.0,
                'reliability': 'reliable'
            }]
        ),

        # Fuzzy controller
        Node(
            package='prisma_rover_quantum_controller',
            executable='quantum_controller_node',
            name='quantum_controller',
            namespace=namespace,
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[{
                'config_dir': config_dir,
                'use_sim_time': use_sim_time,
                'map_frame': [namespace, '/map'],
                'base_frame': [namespace, '/base_footprint']
            }]
        ),
    ])
