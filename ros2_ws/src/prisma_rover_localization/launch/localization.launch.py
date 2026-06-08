import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Evaluate launch configurations
    use_sim_time_val = context.perform_substitution(LaunchConfiguration('use_sim_time'))
    namespace_val = context.perform_substitution(LaunchConfiguration('namespace'))
    tf_prefix_val = context.perform_substitution(LaunchConfiguration('tf_prefix'))

    # Path to EKF parameters
    pkg_share = get_package_share_directory('prisma_rover_localization')
    ekf_config_path = os.path.join(pkg_share, 'config', 'localization_ekf.yaml')

    # Prepended TF Frames if tf_prefix is specified
    map_frame_val = f"{tf_prefix_val}map" if tf_prefix_val else "map"
    odom_frame_val = f"{tf_prefix_val}odom" if tf_prefix_val else "odom"
    base_link_frame_val = f"{tf_prefix_val}base_footprint" if tf_prefix_val else "base_footprint"
    world_frame_val = f"{tf_prefix_val}odom" if tf_prefix_val else "odom"

    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=namespace_val,
        output='screen',
        parameters=[
            ekf_config_path,
            {
                'use_sim_time': use_sim_time_val.lower() == 'true',
                'map_frame': map_frame_val,
                'odom_frame': odom_frame_val,
                'base_link_frame': base_link_frame_val,
                'world_frame': world_frame_val
            }
        ]
    )

    return [ekf_node]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='prisma_rover',
            description='Top-level namespace for EKF node'
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='prisma_rover/',
            description='Prefix to prepend to EKF frames (must end with / if specified)'
        )
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
