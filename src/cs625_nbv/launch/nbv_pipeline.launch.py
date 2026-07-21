# -*- coding: utf-8 -*-
"""
NBV Pipeline Launch File.

Starts the cs625_nbv components (orchestrator + viewpoint publisher)
after the simulation is already running.

Usage (after simulation started):
  ros2 launch cs625_nbv nbv_pipeline.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # NBV server node (orchestrator + services)
    nbv_server_node = Node(
        package="cs625_nbv",
        executable="nbv_server_node",
        name="cs625_nbv_server",
        output="screen",
        parameters=[
            {"view_distance": 0.5},
            {"sample_count": 42},
            {"max_views": 6},
            {"translation_threshold": 0.003},
            {"rotation_threshold_deg": 2.0},
            {"log_base_dir": "~/nbv_experiments"},
        ],
    )

    # Viewpoint visualization publisher
    viewpoint_pub_node = Node(
        package="cs625_nbv",
        executable="viewpoint_publisher_node",
        name="cs625_nbv_viewpoint_publisher",
        output="log",
        parameters=[
            {"view_distance": 0.5},
            {"sample_count": 42},
        ],
    )

    return LaunchDescription([
        nbv_server_node,
        viewpoint_pub_node,
    ])
