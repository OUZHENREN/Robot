from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = "/tmp/cs625_preview.urdf"

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description_content = f.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="test_preview_robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description_content,
                }
            ],
            remappings=[
                ("/joint_states", "/cs625/preview_joint_states"),
            ],
        )
    ])
