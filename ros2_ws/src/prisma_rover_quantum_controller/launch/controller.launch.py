from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('prisma_rover_quantum_controller')
    config_dir = os.path.join(pkg_share, 'config')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='prisma_rover',
        description='Top-level namespace for the quantum controller nodes'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,

        # Lidar listener (configured for real hardware - Livox Mid-360)
        Node(
            package='prisma_rover_quantum_controller',
            executable='lidar_listener',
            name='lidar_listener',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'lidar_topic': 'livox/lidar',
                'elevation_band_deg': 2.0
            }]
        ),

        # Fuzzy controller
        Node(
            package='prisma_rover_quantum_controller',
            executable='quantum_controller_node',
            name='quantum_controller',
            namespace=namespace,
            output='screen',
            parameters=[{
                'config_dir': config_dir,
                'use_sim_time': use_sim_time
            }]
        ),
    ])
