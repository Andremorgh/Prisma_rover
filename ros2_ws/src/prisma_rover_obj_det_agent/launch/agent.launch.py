import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    visualization_arg = DeclareLaunchArgument(
        'visualization',
        default_value='true',
        description='Enable matplotlib visualization for agent'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='prisma_rover/camera_rgb_optical_frame',
        description='Camera optical frame to project 2D detections to 3D'
    )
    
    namespace = 'prisma_rover'
    
    # 1. detect_objects_agent node
    detect_objects_node = Node(
        package='prisma_rover_obj_det_agent',
        executable='detect_objects_agent.py',
        name='detect_objects_agent',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'image_topic': 'color/image_raw',
            'pointcloud_topic': 'depth/color/points',
            'target_frame': LaunchConfiguration('target_frame'),
            'map_frame': f'{namespace}/map',
        }]
    )
    
    # 2. agent node
    agent_node = Node(
        package='prisma_rover_obj_det_agent',
        executable='agent.py',
        name='agent_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'visualization': LaunchConfiguration('visualization'),
            'map_topic': 'map',
            'pose_topic': 'pose',
            'objects_topic': 'objects_detected',
            'map_frame': f'{namespace}/map',
            'navigate_to_pose_action': 'navigate_to_pose',
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        visualization_arg,
        target_frame_arg,
        detect_objects_node,
        agent_node,
    ])
