from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_wrench_topic_arg = DeclareLaunchArgument(
        'input_wrench_topic',
        default_value='/force_torque_sensor_broadcaster/wrench',
        description='Input wrench topic from driver'
    )

    output_force_topic_arg = DeclareLaunchArgument(
        'output_force_topic',
        default_value='/cs625/tcp_force',
        description='Output Float64MultiArray force topic'
    )

    probe_node = Node(
        package='cs625_tcp_force_probe',
        executable='tcp_force_probe_node',
        name='tcp_force_probe_node',
        output='log',
        parameters=[
            {
                'input_wrench_topic': LaunchConfiguration('input_wrench_topic'),
                'output_force_topic': LaunchConfiguration('output_force_topic'),
            }
        ]
    )

    return LaunchDescription([
        input_wrench_topic_arg,
        output_force_topic_arg,
        probe_node,
    ])
