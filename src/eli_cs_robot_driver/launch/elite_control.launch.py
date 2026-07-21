import os
# Copyright 2023 Elite Robots
# ... 版权声明略 ...
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    # ---------------------- 这里保持原样 ----------------------
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
    # ----------------------------------------------------------

    # =====================================================================
    # 1) 各类参数文件路径（保持原有逻辑）
    # =====================================================================
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
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            cs_type,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
            " ",
            "use_tool_communication:=",
            use_tool_communication,
            " ",
            "tool_parity:=",
            tool_parity,
            " ",
            "tool_baud_rate:=",
            tool_baud_rate,
            " ",
            "tool_stop_bits:=",
            tool_stop_bits,
            " ",
            "tool_tcp_port:=",
            tool_tcp_port,
            " ",
            "tool_voltage:=",
            tool_voltage,
            " ",
            "local_ip:=",
            local_ip,
            " ",
            "script_command_port:=",
            script_command_port,
            " ",
            "reverse_port:=",
            reverse_port,
            " ",
            "script_sender_port:=",
            script_sender_port,
            " ",
            "trajectory_port:=",
            trajectory_port,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # ---------------- 调试打印，可保留或删除 ----------------
    print("runtime_config_package (value):", runtime_config_package.perform(context))
    print("controllers_file (value):", controllers_file.perform(context))
    print("initial_joint_controllers File Path:", initial_joint_controllers.perform(context))
    print(
        "cs625_controllers.yaml exists?",
        os.path.exists(initial_joint_controllers.perform(context)),
    )
    # --------------------------------------------------------

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    # =====================================================================
    # 2) 新增：各 controller 自己的额外参数文件（用于 joints / frame_id）
    # =====================================================================
    arm_controller_yaml = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "arm_controller.yaml"]
    )
    forward_position_controller_yaml = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "forward_position_controller.yaml"]
    )
    tcp_pose_broadcaster_yaml = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "tcp_pose_broadcaster.yaml"]
    )
    force_torque_sensor_broadcaster_yaml = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "force_torque_sensor_broadcaster.yaml"]
    )

    # =====================================================================
    # 3) Controller manager - fake / real hardware
    # =====================================================================
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="eli_ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="log",
        
        condition=IfCondition(use_fake_hardware),
    )

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

    components_loader = Node(
        package="eli_cs_robot_driver",
        executable="eli_components_loader",
        parameters=[{"robot_ip": robot_ip}],
        condition=UnlessCondition(use_fake_hardware),
        output="log",
        
    )

    controller_stopper_node = Node(
        package="eli_cs_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="log",
        
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
        output="log",
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

    # =====================================================================
    # 4) Helper function to spawn controllers —— 这里做了关键修改
    # =====================================================================
    def controller_spawner(name, active=True):
        inactive_flags = ["--inactive"] if not active else []

        # 针对不同控制器附加对应的 --param-file
        extra_args = []
        if name == "arm_controller":
            extra_args = ["--param-file", arm_controller_yaml]
        elif name == "forward_position_controller":
            extra_args = ["--param-file", forward_position_controller_yaml]
        elif name == "tcp_pose_broadcaster":
            extra_args = ["--param-file", tcp_pose_broadcaster_yaml]
        elif name == "force_torque_sensor_broadcaster":
            extra_args = ["--param-file", force_torque_sensor_broadcaster_yaml]

        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/eli_ros2_control_node",
                "--controller-manager-timeout",
                controller_spawner_timeout,
                *inactive_flags,
                *extra_args,
            ],
            output="log",
            
        )

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

    # initial_joint_controller 通常是 arm_controller
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/eli_ros2_control_node",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--param-file",
            arm_controller_yaml,
        ],
        condition=IfCondition(activate_joint_controller),
        output="log",
        
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/eli_ros2_control_node",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--inactive",
            "--param-file",
            arm_controller_yaml,
        ],
        condition=UnlessCondition(activate_joint_controller),
        output="log",
        
    )

    # =====================================================================
    # 5) DashboardClient 组件容器（新增）
    # =====================================================================
    dashboard_client_node = ComposableNode(
        package='eli_cs_robot_driver',
        plugin='ELITE_CS_ROBOT_ROS_DRIVER::DashboardClient',
        name='dashboard_client',
        parameters=[{
            'robot_ip': robot_ip,  # 使用 launch 传入的 robot_ip
        }],
        # output='screen' 只对 Node 生效，ComposableNode 的输出由 container 控制
    )

    dashboard_container = ComposableNodeContainer(
        name='dashboard_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[dashboard_client_node],
        output='log',
        
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
        dashboard_container,  # ← 新增这一行
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "cs_type",
            description="Type/series of used Elite robot CS series.",
            choices=["cs63", "cs66", "cs612", "cs616", "cs620", "cs625"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="cs625_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="eli_cs_robot_driver",
            description="Package that contains the controllers configuration (YAML files).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="eli_cs_robot_description",
            description="Robot description package.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="cs625.urdf.xacro",
            description="Robot xacro or urdf filename.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for TF frame names.",
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
            description="Use fake command interfaces for sensors used for simple simulations.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="30",
            description="Timeout for controller spawner [seconds].",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="arm_controller",
            description="Initial joint controller name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Whether to activate joint controller automatically.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Whether to launch RViz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Run in headless (external program control) mode.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Enable tool communication (Modbus/TCP etc.).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Tool parity parameter.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Tool baud rate.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Tool stop bits.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="502",
            description="Tool TCP port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="24",
            description="Tool voltage.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "local_ip",
            default_value="0.0.0.0",
            description="Local IP address to bind to.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="30003",
            description="Script command TCP port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Reverse communication TCP port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="30004",
            description="Script sender TCP port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50002",
            description="Trajectory communication TCP port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="false",
            description="Enable safety limits.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.0",
            description="Safety position margin.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="1.0",
            description="Safety position controller gain.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
