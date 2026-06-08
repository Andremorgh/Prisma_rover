from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='prisma_rover',
        description='Top-level namespace'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,

        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["0.5", "0.5", "1.5", "0", "0", "0", "prisma_rover/map", "exp00"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["4.5", "2", "1.5", "-1.57", "0", "0", "prisma_rover/map", "exp10"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["2.0", "5.5", "1.5", "0", "0", "0", "prisma_rover/map", "exp01"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["4.5", "3", "1.5", "-1.57", "0", "0", "prisma_rover/map", "exp11"]),
        Node(
            package='prisma_rover_manager',
            executable='rover_manager',
            name='rover_manager',
            namespace=namespace,
            output="screen",
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
