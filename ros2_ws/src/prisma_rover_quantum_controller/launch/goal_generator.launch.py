from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('prisma_rover_quantum_controller')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        # Lidar listener (default topic: scan_3d)
        Node(
            package='prisma_rover_quantum_controller',
            executable='lidar_listener',        
            name='lidar_listener',
            output='screen',
        ),

        # Intermediate goal generator
        Node(
            package='prisma_rover_quantum_controller',
            executable='quantum_goal_generator',  
            name='quantum_goal_generator',
            output='screen',
            parameters=[{
                'config_dir': config_dir
            }]
        ),

        # Local planner
        Node(
            package='nav2_simple_local_planner_py',
            executable='simple_local_planner',
            name='nav2_simple_local_planner_py',
            output='screen',
            parameters=[{
                'odom_topic': 'odom/wheels',
                'plan_topic': 'plan',
                'cmd_vel_topic': 'cmd_vel'
            }]
        ),
    ])
