from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('cs625_compliant_placement'),
            'config',
            'compliant_placement.yaml'
        ),
        description='Path to compliant placement config file'
    )

    node = Node(
        package='cs625_compliant_placement',
        executable='compliant_placement_node',
        name='cs625_compliant_placement_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_arg,
        node
    ])
