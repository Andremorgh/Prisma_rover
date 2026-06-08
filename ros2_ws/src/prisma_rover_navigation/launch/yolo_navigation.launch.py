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
    pkg_obj_detection = get_package_share_directory('obj_detection')

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

    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value='/home/user/ros2_ws/src/obj_detection/scripts/dataset',
        description='Absolute path to directory where cropped images and global map database are stored'
    )

    # 1. Include Sim Navigation launch (simulation, localization, slam, navigation)
    sim_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation, 'launch', 'sim_navigation.launch.py')),
        launch_arguments={
            'world': 'yolo_world.sdf',
            'rviz': LaunchConfiguration('rviz'),
            'headless': LaunchConfiguration('headless'),
            'publish_camera': 'true',
            'namespace': 'prisma_rover',
            'tf_prefix': 'prisma_rover/'
        }.items()
    )

    # 2. Define YOLO perception & mapping nodes directly to avoid nested launch subscription issues
    namespace = 'prisma_rover'
    use_sim_time = True
    dataset_path = LaunchConfiguration('dataset_path')

    yolov11_node = Node(
        package='yolov11_ros2',
        executable='yolov11_node',
        namespace=namespace,
        name='yolov11_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model': 'yolo11n-seg.pt',
            'device': 'cpu',
            'threshold': 0.15,
            'enable_yolo': True
        }],
        output='screen'
    )

    detect_objects_node = Node(
        package='obj_detection',
        executable='detect_objects_rs.py',
        namespace=namespace,
        name='detect_objects_rs',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'prisma_rover/camera_rgb_optical_frame',
            'dataset_path': dataset_path
        }],
        output='screen'
    )

    objects_map_node = Node(
        package='obj_detection',
        executable='objects_map.py',
        namespace=namespace,
        name='objects_map',
        parameters=[{
            'use_sim_time': use_sim_time,
            'dataset_path': dataset_path
        }],
        output='screen'
    )

    # Delay perception stack by 10 seconds to allow EKF, SLAM Toolbox, and Nav2 to stabilize
    delayed_perception_stack = TimerAction(
        period=10.0,
        actions=[
            yolov11_node,
            detect_objects_node,
            objects_map_node
        ]
    )

    return LaunchDescription([
        rviz_arg,
        headless_arg,
        dataset_path_arg,
        sim_nav_launch,
        delayed_perception_stack
    ])
