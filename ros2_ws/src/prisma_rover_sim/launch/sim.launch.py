import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Evaluate launch configurations at runtime
    namespace_val = context.perform_substitution(LaunchConfiguration('namespace'))
    tf_prefix_val = context.perform_substitution(LaunchConfiguration('tf_prefix'))
    world_val = context.perform_substitution(LaunchConfiguration('world'))
    use_sim_time_val = context.perform_substitution(LaunchConfiguration('use_sim_time'))
    rviz_val = context.perform_substitution(LaunchConfiguration('rviz'))
    headless_val = context.perform_substitution(LaunchConfiguration('headless'))
    camera_val = context.perform_substitution(LaunchConfiguration('camera'))
    lidar_type_val = context.perform_substitution(LaunchConfiguration('lidar_type'))

    # Retrieve package share directories
    pkg_sim_share = get_package_share_directory('prisma_rover_sim')
    pkg_description_share = get_package_share_directory('prisma_rover_description')

    # Path to the world file
    gz_world_path = os.path.join(pkg_sim_share, 'worlds', world_val)

    # Configure Gazebo arguments based on headless choice
    if headless_val.lower() == 'true':
        gz_args_val = f"-s -r {gz_world_path}"
    else:
        gz_args_val = f"-r {gz_world_path}"

    # 1. Include Gazebo Sim Launch
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args_val}.items()
    )

    # 2. Include Description Launch (handles robot_state_publisher and RViz2)
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description_share, "launch", "description.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time_val,
            "is_sim": "true",
            "namespace": namespace_val,
            "tf_prefix": tf_prefix_val,
            "rviz": rviz_val,
            "camera": camera_val,
            "lidar_type": lidar_type_val
        }.items()
    )

    # 3. Spawn Entity (Rover) Node
    robot_desc_topic = f"/{namespace_val}/robot_description" if namespace_val else "/robot_description"
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_rover",
        arguments=[
            "-topic", robot_desc_topic,
            "-name", f"prisma_rover_{namespace_val}" if namespace_val else "prisma_rover",
            "-allow_renaming", "true",
            "-z", "0.2",
        ],
        output="screen"
    )

    # 4. ROS-Gazebo Parameter Bridge
    ns_prefix = f"/{namespace_val}" if namespace_val else ""
    bridge_args = [
        f"{ns_prefix}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        f"{ns_prefix}/odom/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
        "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        f"{ns_prefix}/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU",
    ]

    # Append Lidar topic dynamically based on Lidar type
    if lidar_type_val.lower() == '3d':
        bridge_args.append(f"{ns_prefix}/scan_3d/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked")
    else:
        bridge_args.append(f"{ns_prefix}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan")

    # Append camera topics dynamically only if camera is enabled
    if camera_val.lower() == 'true':
        bridge_args.extend([
            f"{ns_prefix}/color/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            f"{ns_prefix}/depth/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            f"{ns_prefix}/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image",
            f"{ns_prefix}/depth/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image",
            f"{ns_prefix}/depth/image_raw/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ])

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros2_bridge",
        namespace=namespace_val,  # Run bridge in the same namespace
        arguments=bridge_args,
        output="screen",
        remappings=[
            ('scan_3d/points', 'scan_3d'),
            ('depth/image_raw/points', 'depth/color/points')
        ]
    )

    models_path = os.path.join(pkg_sim_share, 'models')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    return [
        set_gz_resource_path,
        gz_sim,
        description_launch,
        gz_spawn_entity,
        gz_ros2_bridge
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'world',
            default_value='maze.sdf',
            description='World file name (must exist in worlds/ directory)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='prisma_rover',
            description='Top-level namespace for the robot nodes and topics'
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='prisma_rover/',
            description='Prefix to prepend to all tf frames (e.g. "prisma_rover/")'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 if true'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='If true, run simulation server only (no GUI client)'
        ),
        DeclareLaunchArgument(
            'camera',
            default_value='true',
            description='If true, include and bridge camera sensors'
        ),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='2d',
            description='Lidar type to simulate: "2d" (RPLidar) or "3d" (Livox Mid-360)'
        )
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
