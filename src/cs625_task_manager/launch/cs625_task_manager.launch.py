from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cs625_task_manager',
            executable='cs625_task_manager_node',
            name='cs625_task_manager_node',
            output='log',
            
        )
    ])
