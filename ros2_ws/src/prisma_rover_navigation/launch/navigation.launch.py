import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def launch_setup(context, *args, **kwargs):
    # Retrieve share directory
    pkg_share = get_package_share_directory('prisma_rover_navigation')
    
    # Evaluate launch configurations
    namespace_val = context.perform_substitution(LaunchConfiguration('namespace'))
    use_sim_time_val = context.perform_substitution(LaunchConfiguration('use_sim_time'))
    autostart_val = context.perform_substitution(LaunchConfiguration('autostart'))
    params_file_val = context.perform_substitution(LaunchConfiguration('params_file'))

    # Path for rewritten file in the share directory
    rewritten_params_file = os.path.join(pkg_share, 'params', 'configured_nav2_params.yaml')

    # Read, replace "prisma_rover" with the actual namespace, and write back
    with open(params_file_val, 'r') as f:
        content = f.read()
    
    # Replace default namespace with evaluated namespace
    content = content.replace('prisma_rover', namespace_val)

    os.makedirs(os.path.dirname(rewritten_params_file), exist_ok=True)
    with open(rewritten_params_file, 'w') as f:
        f.write(content)

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    # Create parameter substitutions for RewrittenYaml
    param_substitutions = {
        'use_sim_time': use_sim_time_val,
        'autostart': autostart_val
    }

    configured_params = RewrittenYaml(
        source_file=rewritten_params_file,
        root_key=namespace_val,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace_val,
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            namespace=namespace_val,
            output='screen',
            parameters=[configured_params]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace_val,
            output='screen',
            parameters=[configured_params]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace_val,
            output='screen',
            parameters=[configured_params]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace_val,
            output='screen',
            parameters=[configured_params]
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace=namespace_val,
            output='screen',
            parameters=[configured_params]
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            namespace=namespace_val,
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace_val,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_val.lower() == 'true',
                'autostart': autostart_val.lower() == 'true',
                'node_names': lifecycle_nodes
            }]
        )
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory('prisma_rover_navigation')
    default_params_file = os.path.join(pkg_share, 'params', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='prisma_rover',
            description='Top-level namespace for all Nav2 nodes'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),
        OpaqueFunction(function=launch_setup)
    ])
