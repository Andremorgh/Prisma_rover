import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='prisma_rover',
        description='Namespace for the perception and mapping nodes'
    )

    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value='/home/user/ros2_ws/src/obj_detection/scripts/dataset',
        description='Absolute path to directory where cropped images and global map database are stored'
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    dataset_path = LaunchConfiguration('dataset_path')

    # Node 1: YOLO v11 segmentation node
    yolov11_node = Node(
        package='yolov11_ros2',
        executable='yolov11_node',
        namespace=namespace,
        name='yolov11_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model': 'yolo11n-seg.pt',
            'device': 'cpu',  # Fallback to CPU for general execution, can be overridden to cuda:0 if GPU is present
            'threshold': 0.15,
            'enable_yolo': True
        }],
        output='screen'
    )

    # Node 2: Spatial Object Detection node (cropper + spatial relationships)
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

    # Node 3: Object Map manager node (global database & IoU filtering)
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

    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        dataset_path_arg,
        yolov11_node,
        detect_objects_node,
        objects_map_node
    ])
