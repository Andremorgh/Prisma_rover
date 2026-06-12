import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory('prisma_rover_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.xacro')
    rviz_config_file = os.path.join(pkg_share, 'launch', 'rviz.rviz')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    tf_prefix = LaunchConfiguration('tf_prefix')
    rviz = LaunchConfiguration('rviz')
    camera = LaunchConfiguration('camera')
    lidar_type = LaunchConfiguration('lidar_type')

    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for the robot nodes and topics'
    )
    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value='',
        description='Prefix to prepend to all tf frames (e.g. "rover/")'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with preconfigured layout if true'
    )
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='true',
        description='Whether to include the camera in the URDF description'
    )
    lidar_type_arg = DeclareLaunchArgument(
        'lidar_type',
        default_value='3d',
        description='Lidar type to load: 2d (RPLidar S2) or 3d (Livox Mid-360)'
    )

    # Process Xacro with arguments
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' namespace:=', namespace,
        ' camera:=', camera,
        ' lidar_type:=', lidar_type
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,  # Sets the node's namespace
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
            {'frame_prefix': tf_prefix}  # Natively prefixes tf frame IDs in TF messages
        ]
    )

    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz2 Node (launched conditionally)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,  # Sets the node's namespace
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        tf_prefix_arg,
        rviz_arg,
        camera_arg,
        lidar_type_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])

