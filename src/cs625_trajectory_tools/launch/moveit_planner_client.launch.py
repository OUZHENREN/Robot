from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_with_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("elite_cs625_moveit_config"),
                "launch",
                "cs625_moveit_with_client.launch.py",
            )
        )
    )

    return LaunchDescription([
        moveit_with_client_launch,
    ])
