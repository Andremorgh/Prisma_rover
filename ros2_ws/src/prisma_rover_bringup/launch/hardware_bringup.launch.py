import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, EqualsCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_bringup = get_package_share_directory('prisma_rover_bringup')
    pkg_description = get_package_share_directory('prisma_rover_description')

    # Config file paths
    roboclaw_config = os.path.join(pkg_bringup, 'config', 'roboclaw.yaml')
    ekf_config = os.path.join(pkg_bringup, 'config', 'hardware_ekf.yaml')
    livox_json_config = os.path.join(pkg_bringup, 'config', 'livox_mid360.json')

    # Launch configurations
    lidar_type = LaunchConfiguration('lidar_type')
    publish_camera = LaunchConfiguration('publish_camera')

    # Launch arguments
    lidar_type_arg = DeclareLaunchArgument(
        'lidar_type',
        default_value='3d',
        description='Type of LiDAR connected: 2d (RPLidar S2) or 3d (Livox Mid-360)'
    )
    publish_camera_arg = DeclareLaunchArgument(
        'publish_camera',
        default_value='true',
        description='Whether to launch RealSense camera driver'
    )

    # 1. Include Description (State Publisher)
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'is_sim': 'false',
            'namespace': 'prisma_rover',
            'tf_prefix': 'prisma_rover/',
            'publish_camera': publish_camera,
            'lidar_type': lidar_type
        }.items()
    )

    # 2. Roboclaw motor driver node
    roboclaw_node = Node(
        package='roboclaw_ros2',
        executable='roboclaw_node',
        name='roboclaw_node',
        namespace='prisma_rover',
        output='screen',
        parameters=[roboclaw_config]
    )

    # 3. Differential kinematics/odom node
    diffdrive_node = Node(
        package='roboclaw_ros2',
        executable='diffdrive_node',
        name='diffdrive_node',
        namespace='prisma_rover',
        output='screen',
        parameters=[roboclaw_config]
    )

    # 4. Robot Localization (EKF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='prisma_rover',
        output='screen',
        parameters=[ekf_config]
    )

    # 5. RPLidar 2D (launched if lidar_type == '2d')
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        namespace='prisma_rover',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'prisma_rover/laser_link',
            'inverted': False,
            'angle_compensate': True,
        }],
        condition=EqualsCondition(lidar_type, '2d')
    )

    # 6. Livox Mid-360 Lidar 3D (launched if lidar_type == '3d')
    # Refactored to match modern ros2 driver namespaces and configs
    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_ros_driver2_node',
        namespace='prisma_rover',
        output='screen',
        parameters=[{
            'xfer_format': 0, # PointCloud2 format
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'frame_id': 'prisma_rover/laser_link',
            'user_config_path': livox_json_config
        }],
        condition=EqualsCondition(lidar_type, '3d')
    )

    # 7. RealSense Camera Node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='prisma_rover',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'pointcloud.enable': True,
            'base_frame_id': 'prisma_rover/camera_link',
        }],
        condition=IfCondition(publish_camera)
    )

    return LaunchDescription([
        lidar_type_arg,
        publish_camera_arg,
        description_launch,
        roboclaw_node,
        diffdrive_node,
        ekf_node,
        rplidar_node,
        livox_node,
        realsense_node
    ])
