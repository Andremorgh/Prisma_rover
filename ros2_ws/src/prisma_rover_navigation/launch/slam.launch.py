import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Retrieve share directory
    pkg_share = get_package_share_directory('prisma_rover_navigation')
    
    # Evaluate launch configurations
    namespace_val = context.perform_substitution(LaunchConfiguration('namespace'))
    use_sim_time_val = context.perform_substitution(LaunchConfiguration('use_sim_time'))
    slam_params_file_val = context.perform_substitution(LaunchConfiguration('slam_params_file'))
    lidar_type_val = context.perform_substitution(LaunchConfiguration('lidar_type'))
    slam_type_val = context.perform_substitution(LaunchConfiguration('slam_type'))
    tf_prefix_val = context.perform_substitution(LaunchConfiguration('tf_prefix'))

    nodes = []

    # 1. Handle Point Cloud projection if using 3D Lidar (so SLAM and costmaps can consume a 2D scan)
    if lidar_type_val.lower() == '3d':
        pointcloud_projector = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            namespace=namespace_val,
            output='screen',
            parameters=[{
                'target_frame': f"{tf_prefix_val}base_footprint" if tf_prefix_val else "base_footprint",
                'transform_tolerance': 0.01,
                'min_height': -0.1,
                'max_height': 0.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087, # ~0.5 degree resolution
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 20.0,
                'use_sim_time': use_sim_time_val.lower() == 'true',
            }],
            remappings=[
                ('cloud_in', 'scan_3d'),
                ('scan', 'scan')
            ]
        )
        nodes.append(pointcloud_projector)

    # 2. Launch SLAM Toolbox
    if slam_type_val.lower() == 'toolbox':
        rewritten_params_file = os.path.join(pkg_share, 'params', 'configured_mapper_params.yaml')

        # Read, replace "prisma_rover" with the actual namespace, and write back
        with open(slam_params_file_val, 'r') as f:
            content = f.read()
        
        content = content.replace('prisma_rover', namespace_val)

        os.makedirs(os.path.dirname(rewritten_params_file), exist_ok=True)
        with open(rewritten_params_file, 'w') as f:
            f.write(content)

        slam_toolbox_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=namespace_val,
            output='screen',
            parameters=[
                rewritten_params_file,
                {'use_sim_time': use_sim_time_val.lower() == 'true'}
            ],
            remappings=[
                ('/map', ['/', namespace_val, '/map']),
                ('/map_metadata', ['/', namespace_val, '/map_metadata']),
                ('/pose', ['/', namespace_val, '/pose']),
                ('/scan', ['/', namespace_val, '/scan']),
                ('/set_pose', ['/', namespace_val, '/set_pose']),
            ]
        )
        nodes.append(slam_toolbox_node)

    # 3. Launch RTAB-Map SLAM
    elif slam_type_val.lower() == 'rtabmap':
        rtabmap_yaml_file = os.path.join(pkg_share, 'params', 'rtabmap.yaml')
        rewritten_rtabmap_file = os.path.join(pkg_share, 'params', 'configured_rtabmap_params.yaml')

        with open(rtabmap_yaml_file, 'r') as f:
            content = f.read()

        content = content.replace('prisma_rover', namespace_val)

        os.makedirs(os.path.dirname(rewritten_rtabmap_file), exist_ok=True)
        with open(rewritten_rtabmap_file, 'w') as f:
            f.write(content)

        rtabmap_node = Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace=namespace_val,
            output='screen',
            parameters=[
                rewritten_rtabmap_file,
                {
                    'use_sim_time': use_sim_time_val.lower() == 'true',
                    'subscribe_scan': lidar_type_val.lower() == '2d',
                    'subscribe_scan_cloud': lidar_type_val.lower() == '3d',
                    'frame_id': f"{tf_prefix_val}base_footprint" if tf_prefix_val else "base_footprint",
                    'odom_frame_id': f"{tf_prefix_val}odom" if tf_prefix_val else "odom",
                    'map_frame_id': f"{tf_prefix_val}map" if tf_prefix_val else "map",
                }
            ],
            remappings=[
                ('odom', 'odometry/filtered'),
                ('scan', 'scan'),
                ('scan_cloud', 'scan_3d'),
                ('map', 'map')
            ],
            arguments=['-d'] # Delete database at startup to avoid loop-closure history contamination
        )
        nodes.append(rtabmap_node)

    return nodes

def generate_launch_description():
    pkg_share = get_package_share_directory('prisma_rover_navigation')
    default_params_file = os.path.join(pkg_share, 'params', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='prisma_rover',
            description='Top-level namespace for SLAM node'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_params_file,
            description='Full path to the SLAM Toolbox parameters file'
        ),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='2d',
            description='Lidar type to support: "2d" or "3d"'
        ),
        DeclareLaunchArgument(
            'slam_type',
            default_value='toolbox',
            description='SLAM method: "toolbox" or "rtabmap"'
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='prisma_rover/',
            description='Prefix to prepend to EKF frames (must end with / if specified)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
