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

    # Declare arguments we want to expose
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo headless if true'
    )

    # 1. Include Sim Navigation launch (simulation, localization, slam, navigation)
    sim_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation, 'launch', 'sim_navigation.launch.py')),
        launch_arguments={
            'world': 'aruco_world.sdf',
            'rviz': LaunchConfiguration('rviz'),
            'headless': LaunchConfiguration('headless'),
            'camera': 'true',
            'namespace': 'prisma_rover',
            'tf_prefix': 'prisma_rover/'
        }.items()
    )

    # 2. ArUco Pose Estimation node (defined directly to avoid nested launch issues
    #    with image subscriber matching in ROS 2 Humble)
    aruco_node = Node(
        package='aruco_pose_estimation',
        executable='aruco_node',
        namespace='prisma_rover',
        parameters=[
            aruco_params_file,
            {
                'use_sim_time': True,
                'marker_size': 0.5,
                'map_frame': 'prisma_rover/map',
            }
        ],
        output='screen',
        emulate_tty=True
    )

    # Delay aruco_node startup to allow simulation and navigation stack
    # to fully initialize and stabilize CPU/DDS discovery
    delayed_aruco_node = TimerAction(
        period=10.0,
        actions=[aruco_node]
    )

    return LaunchDescription([
        rviz_arg,
        headless_arg,
        sim_nav_launch,
        delayed_aruco_node
    ])
