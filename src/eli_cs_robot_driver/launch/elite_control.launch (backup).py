import os
# Copyright 2023 Elite Robots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    cs_type = LaunchConfiguration("cs_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    local_ip = LaunchConfiguration("local_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")


    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", cs_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", cs_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", cs_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", cs_type, "visual_parameters.yaml"]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare("eli_cs_robot_driver"), "resources", "external_control.script"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("eli_cs_robot_driver"), "resources", "input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("eli_cs_robot_driver"), "resources", "output_recipe.txt"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
            "robot_ip:=", robot_ip, " ",
            "joint_limit_params:=", joint_limit_params, " ",
            "kinematics_params:=", kinematics_params, " ",
            "physical_params:=", physical_params, " ",
            "visual_params:=", visual_params, " ",
            "safety_limits:=", safety_limits, " ",
            "safety_pos_margin:=", safety_pos_margin, " ",
            "safety_k_position:=", safety_k_position, " ",
            "name:=", cs_type, " ",
            "script_filename:=", script_filename, " ",
            "input_recipe_filename:=", input_recipe_filename, " ",
            "output_recipe_filename:=", output_recipe_filename, " ",
            "tf_prefix:=", tf_prefix, " ",
            "use_fake_hardware:=", use_fake_hardware, " ",
            "fake_sensor_commands:=", fake_sensor_commands, " ",
            "headless_mode:=", headless_mode, " ",
            "use_tool_communication:=", use_tool_communication, " ",
            "tool_parity:=", tool_parity, " ",
            "tool_baud_rate:=", tool_baud_rate, " ",
            "tool_stop_bits:=", tool_stop_bits, " ",
            "tool_tcp_port:=", tool_tcp_port, " ",
            "tool_voltage:=", tool_voltage, " ",
            "local_ip:=", local_ip, " ",
            "script_command_port:=", script_command_port, " ",
            "reverse_port:=", reverse_port, " ",
            "script_sender_port:=", script_sender_port, " ",
            "trajectory_port:=", trajectory_port, " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    
    print("runtime_config_package (value):", runtime_config_package.perform(context))
    print("controllers_file (value):", controllers_file.perform(context))
    print("initial_joint_controllers File Path:", initial_joint_controllers.perform(context))
    print("cs625_controllers.yaml exists?", os.path.exists(initial_joint_controllers.perform(context)))

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    # define update rate
    update_rate_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "cs_update_rate.yaml"]
    )

    # Controller manager - loaded when using fake hardware
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="eli_ros2_control_node",
        parameters=[
            robot_description,
            
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    # Controller manager - loaded when using real hardware
    eli_control_node = Node(
        package="eli_cs_robot_driver",
        executable="eli_ros2_control_node",
        name="eli_ros2_control_node",
        parameters=[
            robot_description,
            
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    # Component loader for hardware-related components
    components_loader = Node(
        package="eli_cs_robot_driver",
        executable="eli_components_loader",
        parameters=[{"robot_ip": robot_ip}],
        condition=UnlessCondition(use_fake_hardware)
    )

    controller_stopper_node = Node(
        package="eli_cs_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": activate_joint_controller},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                ]
            },
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Helper function to spawn controllers
    def controller_spawner(name, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/eli_ros2_control_node", "--controller-manager-timeout",
                       controller_spawner_timeout,"--param-file", initial_joint_controllers] + inactive_flags,
        )

    # List of controllers to spawn
    controller_spawner_names = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
    ]
    controller_spawner_inactive_names = ["forward_position_controller", "freedrive_controller"]

    controller_spawners = [controller_spawner(name) for name in controller_spawner_names] + [
        controller_spawner(name, active=False) for name in controller_spawner_inactive_names
    ]

    # Spawner for the initial joint controller, respecting the 'activate_joint_controller' argument
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/eli_ros2_control_node", "--controller-manager-timeout",
                   controller_spawner_timeout,"--param-file", initial_joint_controllers],
        condition=IfCondition(activate_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/eli_ros2_control_node", "--controller-manager-timeout",
                   controller_spawner_timeout,"--param-file", initial_joint_controllers, "--inactive"],
        condition=UnlessCondition(activate_joint_controller),
    )

    nodes_to_start = [
        control_node,
        eli_control_node,
        components_loader,
        controller_stopper_node,
        robot_state_publisher_node,
        rviz_node,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
    ] + controller_spawners

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    # Elite CS robots specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "cs_type",
            description="Type/series of used Elite robot CS series.",
            choices=["cs63", "cs66", "cs612", "cs616", "cs620", "cs625"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "local_ip",
            default_value="0.0.0.0",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="eli_cs_robot_driver",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="elite_cs_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="eli_cs_robot_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="cs625.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for \
        multi-robot setup. If changed, also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Temporarily unavailable.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Temporarily unavailable. Parity configuration for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Temporarily unavailable. Baud rate configuration for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Temporarily unavailable. Stop bits configuration for serial communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Temporarily unavailable. Remote port that will be used for bridging the tool's serial device.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",
            description="Tool voltage that will be setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port that will be opened to forward script commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the ExternalControl script on this port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


