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

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    cs_type = LaunchConfiguration("cs_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless = LaunchConfiguration("headless")
    enable_rgbd_sensor = LaunchConfiguration("enable_rgbd_sensor")

    # Keep NBV runtime data independent from legacy/broken shared-folder links.
    nbv_data_dir = os.path.join(os.path.expanduser("~"), "elite_ros_ws", "nbv_data")
    pointcloud_dir = os.path.join(nbv_data_dir, "environment_point_cloud")
    csv_dir = os.path.join(nbv_data_dir, "csv")
    os.makedirs(pointcloud_dir, exist_ok=True)
    os.makedirs(csv_dir, exist_ok=True)

    world_source_path = os.path.join(
        get_package_share_directory("eli_cs_robot_simulation_gz"),
        "worlds",
        "nbv_scene.sdf",
    )
    if enable_rgbd_sensor.perform(context).lower() in ("1", "true", "yes", "on"):
        world_path = world_source_path
    else:
        # Rendering sensors can stall Gazebo in VMs without 3D acceleration.
        # Generate a control-only runtime world while keeping the RGB-D source
        # world intact for accelerated simulation and later hardware work.
        runtime_world_path = os.path.join(nbv_data_dir, "nbv_scene_control.sdf")
        tree = ET.parse(world_source_path)
        world = tree.getroot().find("world")
        for plugin in list(world.findall("plugin")):
            if plugin.get("name") == "gz::sim::systems::Sensors":
                world.remove(plugin)
        tree.write(runtime_world_path, encoding="utf-8", xml_declaration=True)
        world_path = runtime_world_path

    # Gazebo resolves package:// mesh URIs as model:// URIs. Add the parent
    # share directory so eli_cs_robot_description can be found by name.
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_description"),
                "..",
            ]),
            os.pathsep,
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )
    software_gl = SetEnvironmentVariable(
        name="LIBGL_ALWAYS_SOFTWARE",
        value="1",
        condition=IfCondition(enable_rgbd_sensor),
    )
    gallium_driver = SetEnvironmentVariable(
        name="GALLIUM_DRIVER",
        value="llvmpipe",
        condition=IfCondition(enable_rgbd_sensor),
    )

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
            "headless": headless,
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
        parameters=[{
            "use_sim_time": True,
            "output_dir": csv_dir,
        }],
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
                launch_arguments={
                    "directory_path": pointcloud_dir,
                    "output_directory_path": os.path.join(
                        nbv_data_dir, "environment_point_cloud_fusion"
                    ),
                    "use_xterm": "false",
                }.items(),
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
        gazebo_resource_path,
        software_gl,
        gallium_driver,
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
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description=(
                "Run Gazebo server-only without a GUI. Use true for SSH/CI "
                "validation; keep false for local visual demonstrations."
            ),
        ),
        DeclareLaunchArgument(
            "enable_rgbd_sensor",
            default_value="false",
            description=(
                "Enable the Gazebo RGB-D rendering system. Requires working "
                "3D acceleration; keep false for control-only VM simulation."
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
