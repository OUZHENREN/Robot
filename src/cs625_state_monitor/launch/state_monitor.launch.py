from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.200',
        description='Robot IP address'
    )

    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='30001',
        description='Robot primary port'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/yff/cs625_data',
        description='CSV output directory'
    )

    state_monitor_node = Node(
        package='cs625_state_monitor',
        executable='raw_state_receiver_node',
        name='cs625_raw_state_receiver_node',
        output='log',
        parameters=[
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
                'robot_port': LaunchConfiguration('robot_port'),
            }
        ]
    )

    csv_logger_node = Node(
        package='cs625_state_monitor',
        executable='state_csv_logger_node',
        name='cs625_state_csv_logger_node',
        output='log',
        parameters=[
            {
                'output_dir': LaunchConfiguration('output_dir'),
            }
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        robot_port_arg,
        output_dir_arg,
        state_monitor_node,
        csv_logger_node,
    ])
