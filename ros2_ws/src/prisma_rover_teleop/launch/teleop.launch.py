import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_teleop_share = get_package_share_directory('prisma_rover_teleop')

    # Argomenti di lancio per una massima riutilizzabilità
    mode = LaunchConfiguration('mode')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')

    # Dichiarazione degli argomenti
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='keyboard',
        description='Modalita di teleoperazione: "keyboard" o "joy"'
    )

    declare_cmd_vel_topic_cmd = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/prisma_rover/cmd_vel',
        description='Topic su cui pubblicare i comandi Twist'
    )

    declare_joy_dev_cmd = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='ID del dispositivo joystick'
    )

    declare_config_filepath_cmd = DeclareLaunchArgument(
        'config_filepath',
        default_value=os.path.join(pkg_teleop_share, 'config', 'teleop_params.yaml'),
        description='Percorso assoluto del file YAML di configurazione del joystick'
    )

    teleop_keyboard_node = Node(
        package='prisma_rover_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard_node',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'cmd_vel_topic': cmd_vel_topic,
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'keyboard'"]))
    )

    # 2. Nodi di teleoperazione da joystick (joy_node + teleop_twist_joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'joy'"]))
    )

    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[config_filepath, {'publish_stamped_twist': False}],
        remappings=[('/cmd_vel', cmd_vel_topic)],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'joy'"]))
    )

    return LaunchDescription([
        declare_mode_cmd,
        declare_cmd_vel_topic_cmd,
        declare_joy_dev_cmd,
        declare_config_filepath_cmd,
        teleop_keyboard_node,
        joy_node,
        teleop_joy_node
    ])
