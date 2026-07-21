# -*- coding: utf-8 -*-
"""
NBV Simulation Launch File.

Launches the full simulation pipeline for PoseGain-NBV development:
  - Gazebo (Ignition) with NBV scene world
  - CS625 robot with depth camera
  - MoveIt 2 for motion planning
  - Simulation state publisher (mock robot state)
  - Environment point cloud & collision generators
  - RViz for visualization

Usage:
  ros2 launch eli_cs_robot_simulation_gz nbv_simulation.launch.py

The NBV orchestrator (cs625_nbv) is NOT launched here — it should be launched
separately once the simulation is stable, to allow iterative development.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    cs_type = LaunchConfiguration("cs_type")
    launch_rviz = LaunchConfiguration("launch_rviz")

    world_path = PathJoinSubstitution([
        FindPackageShare("eli_cs_robot_simulation_gz"),
        "worlds",
        "nbv_scene.sdf",
    ])

    # ========================================================================
    # 1. Robot description + controllers + MoveIt + Gazebo
    #    (Gazebo is launched by cs_sim_control.launch.py with NBV scene world)
    # ========================================================================
    sim_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("eli_cs_robot_simulation_gz"),
            "/launch",
            "/cs_sim_moveit.launch.py",
        ]),
        launch_arguments={
            "cs_type": cs_type,
            "launch_rviz": launch_rviz,
            "world_file": world_path,
        }.items(),
    )

    # ========================================================================
    # 2. Simulation state publisher (replaces real-hardware TCP state)
    # ========================================================================
    sim_state_node = Node(
        package="cs625_state_monitor",
        executable="sim_state_publisher",
        name="cs625_sim_state_publisher",
        output="log",
        parameters=[{"use_sim_time": True}],
    )

    # ========================================================================
    # 3. CSV logger (records NBV experiment data)
    # ========================================================================
    csv_logger_node = Node(
        package="cs625_state_monitor",
        executable="state_csv_logger_node",
        name="cs625_state_csv_logger_node",
        output="log",
        parameters=[{"use_sim_time": True}],
    )

    # ========================================================================
    # 4. Environment point cloud publisher (delayed, after moveit)
    # ========================================================================
    env_pc_publisher = TimerAction(
        period=1.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("environment_point_cloud_publisher"),
                    "/launch",
                    "/environment_point_cloud_publisher.launch.py",
                ]),
            ),
        ],
    )

    # ========================================================================
    # 5. Environment collision generator (delayed, after PC publisher)
    # ========================================================================
    env_collision_gen = TimerAction(
        period=2.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("environment_collision_generator"),
                    "/launch",
                    "/environment_collision_generator.launch.py",
                ]),
            ),
        ],
    )

    return [
        sim_moveit_launch,
        sim_state_node,
        csv_logger_node,
        env_pc_publisher,
        env_collision_gen,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "cs_type",
            default_value="cs625",
            description="Type of ELITE CS robot.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz for visualization.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
