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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_synthetic_camera = LaunchConfiguration("use_synthetic_camera")
    view_dependent_synthetic = LaunchConfiguration("view_dependent_synthetic")
    occlusion_level = LaunchConfiguration("occlusion_level")
    depth_noise_std = LaunchConfiguration("depth_noise_std")
    data_source = LaunchConfiguration("data_source")
    validity_label = LaunchConfiguration("validity_label")
    random_seed = LaunchConfiguration("random_seed")
    covariance_bootstrap_samples = LaunchConfiguration("covariance_bootstrap_samples")
    git_commit = LaunchConfiguration("git_commit")
    launch_profile = LaunchConfiguration("launch_profile")
    scene_name = LaunchConfiguration("scene_name")

    synthetic_camera_node = Node(
        package="cs625_nbv",
        executable="synthetic_camera_publisher_node",
        name="cs625_synthetic_camera",
        output="log",
        condition=IfCondition(use_synthetic_camera),
        parameters=[
            {"frame_id": "base_link"},
            {"publish_hz": 10.0},
            {"view_dependent": view_dependent_synthetic},
            {"occlusion_level": occlusion_level},
            {"depth_noise_std": depth_noise_std},
            {"random_seed": random_seed},
        ],
    )

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
            {"bootstrap_model_from_first_cloud": use_synthetic_camera},
            {"use_model_cloud_topic": view_dependent_synthetic},
            {"data_source": data_source},
            {"validity_label": validity_label},
            {"random_seed": random_seed},
            {"covariance_bootstrap_samples": covariance_bootstrap_samples},
            {"git_commit": git_commit},
            {"launch_profile": launch_profile},
            {"scene_name": scene_name},
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
        DeclareLaunchArgument(
            "use_synthetic_camera",
            default_value="true",
            description=(
                "Publish a deterministic cuboid cloud for remote software-only "
                "validation. Set false for Gazebo RGB-D or real-camera input."
            ),
        ),
        DeclareLaunchArgument(
            "view_dependent_synthetic",
            default_value="false",
            description=(
                "Publish partial clouds based on /cs625_nbv/virtual_camera_pose and "
                "use the complete synthetic model cloud for ICP."
            ),
        ),
        DeclareLaunchArgument(
            "occlusion_level",
            default_value="none",
            description="Deterministic virtual occlusion: none, light, or heavy.",
        ),
        DeclareLaunchArgument(
            "depth_noise_std",
            default_value="0.0",
            description="Deterministic depth-noise standard deviation in metres.",
        ),
        DeclareLaunchArgument(
            "data_source",
            default_value="synthetic_smoke_test",
            description="Provenance label: synthetic_smoke_test, gazebo_rgbd, or real_camera.",
        ),
        DeclareLaunchArgument(
            "validity_label",
            default_value="interface_only",
            description="Use interface_only for synthetic smoke tests.",
        ),
        DeclareLaunchArgument(
            "random_seed",
            default_value="625",
            description="Fixed seed for reproducible random_reachable baselines.",
        ),
        DeclareLaunchArgument(
            "covariance_bootstrap_samples",
            default_value="30",
            description="Bootstrap ICP registrations per covariance estimate (at least 2).",
        ),
        DeclareLaunchArgument(
            "git_commit",
            default_value="unknown",
            description="Git commit recorded in the experiment manifest.",
        ),
        DeclareLaunchArgument(
            "launch_profile",
            default_value="synthetic_headless",
            description="Human-readable launch profile stored with each episode.",
        ),
        DeclareLaunchArgument(
            "scene_name",
            default_value="default_scene",
            description="Scene label recorded in each virtual experiment export.",
        ),
        synthetic_camera_node,
        nbv_server_node,
        viewpoint_pub_node,
    ])
