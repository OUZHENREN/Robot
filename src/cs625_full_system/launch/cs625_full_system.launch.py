from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # ===== 参数声明区（沿用旧版本核心参数） =====
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.1.200')
    cs_type_arg = DeclareLaunchArgument('cs_type', default_value='cs625')
    tf_prefix_arg = DeclareLaunchArgument('tf_prefix', default_value='')
    use_fake_hardware_arg = DeclareLaunchArgument('use_fake_hardware', default_value='false')
    safety_limits_arg = DeclareLaunchArgument('safety_limits', default_value='false')
    safety_pos_margin_arg = DeclareLaunchArgument('safety_pos_margin', default_value='0.0')
    safety_k_position_arg = DeclareLaunchArgument(
        'safety_k_position', default_value='1.0',
        description='Safety position controller gain')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        'fake_sensor_commands', default_value='false',
        description='Use fake sensor command inputs (for testing/simulation)')
    headless_mode_arg = DeclareLaunchArgument(
        'headless_mode', default_value='false',
        description='Run in headless mode (no GUI)')
    use_tool_communication_arg = DeclareLaunchArgument(
        'use_tool_communication', default_value='false',
        description='Enable tool communication')
    tool_parity_arg = DeclareLaunchArgument(
        'tool_parity', default_value='0', description='Tool parity parameter')
    tool_baud_rate_arg = DeclareLaunchArgument(
        'tool_baud_rate', default_value='115200', description='Tool baud rate')
    tool_stop_bits_arg = DeclareLaunchArgument(
        'tool_stop_bits', default_value='1', description='Tool stop bits')
    tool_tcp_port_arg = DeclareLaunchArgument(
        'tool_tcp_port', default_value='502', description='Tool TCP port')
    tool_data_bits_arg = DeclareLaunchArgument(
        'tool_data_bits', default_value='8', description='Tool data bits')
    tool_flow_control_arg = DeclareLaunchArgument(
        'tool_flow_control', default_value='none', description='Tool flow control')
    tool_device_path_arg = DeclareLaunchArgument(
        'tool_device_path', default_value='', description='Tool device path')
    tool_timeout_arg = DeclareLaunchArgument(
        'tool_timeout', default_value='1.0', description='Tool timeout')

    tool_voltage_arg = DeclareLaunchArgument(
        'tool_voltage', default_value='24', description='Tool voltage')
    tool_current_arg = DeclareLaunchArgument(
        'tool_current', default_value='0', description='Tool current')
    tool_type_arg = DeclareLaunchArgument(
        'tool_type', default_value='0', description='Tool type')
    tool_power_arg = DeclareLaunchArgument(
        'tool_power', default_value='0', description='Tool power')
    tool_state_arg = DeclareLaunchArgument(
        'tool_state', default_value='0', description='Tool state')
    tool_enable_arg = DeclareLaunchArgument(
        'tool_enable', default_value='false', description='Enable tool')

    local_ip_arg = DeclareLaunchArgument(
        'local_ip', default_value='0.0.0.0',
        description='Local IP address to bind to')
    remote_ip_arg = DeclareLaunchArgument(
        'remote_ip', default_value='192.168.1.201',
        description='Remote robot/server IP')
    local_port_arg = DeclareLaunchArgument(
        'local_port', default_value='0', description='Local port')
    remote_port_arg = DeclareLaunchArgument(
        'remote_port', default_value='502', description='Remote port')

    script_command_port_arg = DeclareLaunchArgument(
        'script_command_port', default_value='30003',
        description='Script command TCP port')
    script_sender_port_arg = DeclareLaunchArgument(
        'script_sender_port', default_value='30004',
        description='Script sender TCP port')
    reverse_port_arg = DeclareLaunchArgument(
        'reverse_port', default_value='50001',
        description='Reverse communication TCP port')
    trajectory_port_arg = DeclareLaunchArgument(
        'trajectory_port', default_value='50002',
        description='Trajectory communication TCP port')

    controller_spawner_timeout_arg = DeclareLaunchArgument(
        'controller_spawner_timeout', default_value='5.0',
        description='Timeout for controller spawner [seconds]')
    runtime_config_package_arg = DeclareLaunchArgument(
        'runtime_config_package', default_value='eli_cs_robot_driver',
        description='Runtime config package')
    description_package_arg = DeclareLaunchArgument(
        'description_package', default_value='eli_cs_robot_description',
        description='Robot description package')
    description_file_arg = DeclareLaunchArgument(
        'description_file', default_value='cs625.urdf.xacro',
        description='Robot xacro or urdf filename')
    initial_joint_controller_arg = DeclareLaunchArgument(
        'initial_joint_controller', default_value='arm_controller',
        description='Initial joint controller name')
    activate_joint_controller_arg = DeclareLaunchArgument(
        'activate_joint_controller', default_value='true',
        description='Whether to activate controller automatically')
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='false',
        description='Whether to launch RViz')

    # ===== environment_point_cloud_publisher 参数 =====
    environment_point_cloud_directory_arg = DeclareLaunchArgument(
        'environment_point_cloud_directory',
        default_value='/home/yff/environment_point_cloud',
        description='Directory containing environment PCD files')
    environment_point_cloud_topic_arg = DeclareLaunchArgument(
        'environment_point_cloud_topic',
        default_value='/environment/point_cloud',
        description='Topic name for published environment point cloud')
    environment_point_cloud_frame_id_arg = DeclareLaunchArgument(
        'environment_point_cloud_frame_id',
        default_value='base_link',
        description='Frame ID for published environment point cloud')
    environment_point_cloud_publish_interval_sec_arg = DeclareLaunchArgument(
        'environment_point_cloud_publish_interval_sec',
        default_value='1.0',
        description='Publish interval in seconds for environment point cloud')
    environment_point_cloud_loop_playback_arg = DeclareLaunchArgument(
        'environment_point_cloud_loop_playback',
        default_value='true',
        description='Whether to loop playback of environment point cloud files')
    environment_point_cloud_use_xterm_arg = DeclareLaunchArgument(
        'environment_point_cloud_use_xterm',
        default_value='true',
        description='Whether to launch environment point cloud publisher in a dedicated xterm window')

    # ===== workspace_pointcloud_sampler 参数 =====
    workspace_sampler_use_xterm_arg = DeclareLaunchArgument(
        'workspace_sampler_use_xterm',
        default_value='true',
        description='Whether to launch workspace pointcloud sampler in a dedicated xterm window')

    # controllers.yaml 路径（需要传给 MoveIt）
    ros2_controllers_file = PathJoinSubstitution([
        FindPackageShare('eli_cs_robot_driver'), 'config', 'cs625_controllers.yaml'
    ])

    # ===== 只给驱动入口 cs625_driver.launch.py 的参数 =====
    driver_args_dict = {
        'robot_ip': LaunchConfiguration('robot_ip'),
        'cs_type': LaunchConfiguration('cs_type'),
        'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
        'safety_limits': LaunchConfiguration('safety_limits'),
        'safety_pos_margin': LaunchConfiguration('safety_pos_margin'),
        'safety_k_position': LaunchConfiguration('safety_k_position'),
    }

    # ===== 给 MoveIt 两个入口（cs625_moveit / cs625_moveit_rviz）的参数 =====
    moveit_args_dict = {
        'robot_ip': LaunchConfiguration('robot_ip'),
        'cs_type': LaunchConfiguration('cs_type'),
        'tf_prefix': LaunchConfiguration('tf_prefix'),
        'name': 'cs625',
        'description_package': LaunchConfiguration('description_package'),
        'description_file': LaunchConfiguration('description_file'),
        'initial_joint_controller': LaunchConfiguration('initial_joint_controller'),
        'activate_joint_controller': LaunchConfiguration('activate_joint_controller'),
        'launch_rviz': LaunchConfiguration('launch_rviz'),
        'runtime_config_package': LaunchConfiguration('runtime_config_package'),
        'ros2_controllers_file': ros2_controllers_file,
        'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
        'safety_limits': LaunchConfiguration('safety_limits'),
        'safety_pos_margin': LaunchConfiguration('safety_pos_margin'),
        'safety_k_position': LaunchConfiguration('safety_k_position'),
        'fake_sensor_commands': LaunchConfiguration('fake_sensor_commands'),
        'headless_mode': LaunchConfiguration('headless_mode'),
        'use_tool_communication': LaunchConfiguration('use_tool_communication'),
        'tool_parity': LaunchConfiguration('tool_parity'),
        'tool_baud_rate': LaunchConfiguration('tool_baud_rate'),
        'tool_stop_bits': LaunchConfiguration('tool_stop_bits'),
        'tool_tcp_port': LaunchConfiguration('tool_tcp_port'),
        'tool_data_bits': LaunchConfiguration('tool_data_bits'),
        'tool_flow_control': LaunchConfiguration('tool_flow_control'),
        'tool_device_path': LaunchConfiguration('tool_device_path'),
        'tool_timeout': LaunchConfiguration('tool_timeout'),
        'tool_voltage': LaunchConfiguration('tool_voltage'),
        'tool_current': LaunchConfiguration('tool_current'),
        'tool_type': LaunchConfiguration('tool_type'),
        'tool_power': LaunchConfiguration('tool_power'),
        'tool_state': LaunchConfiguration('tool_state'),
        'tool_enable': LaunchConfiguration('tool_enable'),
        'local_ip': LaunchConfiguration('local_ip'),
        'remote_ip': LaunchConfiguration('remote_ip'),
        'local_port': LaunchConfiguration('local_port'),
        'remote_port': LaunchConfiguration('remote_port'),
        'script_command_port': LaunchConfiguration('script_command_port'),
        'script_sender_port': LaunchConfiguration('script_sender_port'),
        'reverse_port': LaunchConfiguration('reverse_port'),
        'trajectory_port': LaunchConfiguration('trajectory_port'),
        'controller_spawner_timeout': LaunchConfiguration('controller_spawner_timeout'),
    }

    # ===== 给 environment_point_cloud_publisher 入口的参数 =====
    environment_point_cloud_args_dict = {
        'directory_path': LaunchConfiguration('environment_point_cloud_directory'),
        'topic_name': LaunchConfiguration('environment_point_cloud_topic'),
        'frame_id': LaunchConfiguration('environment_point_cloud_frame_id'),
        'publish_interval_sec': LaunchConfiguration('environment_point_cloud_publish_interval_sec'),
        'loop_playback': LaunchConfiguration('environment_point_cloud_loop_playback'),
        'use_xterm': LaunchConfiguration('environment_point_cloud_use_xterm'),
    }

    # ===== 给 workspace_pointcloud_sampler 入口的参数 =====
    workspace_sampler_args_dict = {
        'use_xterm': LaunchConfiguration('workspace_sampler_use_xterm'),
    }

    # ===== 1. 驱动入口（cs625_driver.launch.py） =====
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('eli_cs_robot_driver'),
                'launch',
                'cs625_driver.launch.py'
            ])
        ]),
        launch_arguments=driver_args_dict.items()
    )

    # ===== 2. MoveIt 核心入口（cs625_moveit_with_client.launch.py） =====
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('elite_cs625_moveit_config'),
                'launch',
                'cs625_moveit_with_client.launch.py'
            ])
        ]),
        launch_arguments=moveit_args_dict.items()
    )

    # ===== 3. MoveIt RViz 入口（cs625_moveit_rviz.launch.py） =====
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('elite_cs625_moveit_config'),
                'launch',
                'cs625_moveit_rviz.launch.py'
            ])
        ]),
        launch_arguments=moveit_args_dict.items()
    )

    # ===== 4. cs625_kinematics IK 服务节点 =====
    cs625_kinematics_node = Node(
        package='cs625_kinematics',
        executable='ik_server_node',
        name='ik_server_node',
        output='log'
    )

    # ===== 5. tcp_bridge 子入口 =====
    tcp_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tcp_bridge'),
                'launch',
                'cs625_launcher.py'
            ])
        ])
    )

    # ===== 5.1. cs625_tcp_force_probe TCP 力采样子入口 =====
    tcp_force_probe_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cs625_tcp_force_probe'),
                'launch',
                'tcp_force_probe.launch.py'
            ])
        ])
    )
    # ===== 6. environment_point_cloud_publisher 子入口 =====
    environment_point_cloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('environment_point_cloud_publisher'),
                'launch',
                'environment_point_cloud_publisher.launch.py'
            ])
        ]),
        launch_arguments=environment_point_cloud_args_dict.items()
    )

    # ===== 7. workspace_pointcloud_sampler 子入口 =====
    workspace_sampler_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('workspace_pointcloud_sampler'),
                'launch',
                'workspace_pointcloud_sampler.launch.py'
            ])
        ]),
        launch_arguments=workspace_sampler_args_dict.items()
    )

    # ===== 8. environment_collision_generator 子入口 =====
    environment_collision_generator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('environment_collision_generator'),
                'launch',
                'environment_collision_generator.launch.py'
            ])
        ])
    )

    # ===== 9. cs625_trajectory_tools TCP Path 可视化子系统 =====
    tcp_path_visual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cs625_trajectory_tools'),
                'launch',
                'cs625_tcp_path_visual.launch.py'
            ])
        ])
    )

    # ===== 10. cs625_state_monitor 原始状态监测与 CSV 记录 =====
    state_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cs625_state_monitor'),
                'launch',
                'state_monitor.launch.py'
            ])
        ])
    )

    # ===== 11. cs625_compliant_placement 柔顺放置子入口 =====
    compliant_placement_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cs625_compliant_placement'),
                'launch',
                'compliant_placement.launch.py'
            ])
        ])
    )

    # ===== 12. cs625_task_manager 任务管理器 =====
    task_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cs625_task_manager'),
                'launch',
                'cs625_task_manager.launch.py'
            ])
        ])
    )

    # ===== 13. preview selected-solution visualization =====
    preview_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cs625_kinematics'),
                'launch',
                'cs625_preview_display.launch.py'
            ])
        ]),
        launch_arguments={
            'cs_type': LaunchConfiguration('cs_type'),
        }.items()
    )

    # ===== 启动顺序控制（TimerAction） =====
    start_driver = driver_launch

    start_moveit = TimerAction(
        period=3.0,
        actions=[moveit_launch]
    )

    start_cs625_kinematics = TimerAction(
        period=5.0,
        actions=[cs625_kinematics_node]
    )

    start_preview_display = TimerAction(
        period=5.5,
        actions=[preview_display_launch]
    )

    start_tcp_bridge = TimerAction(
        period=6.0,
        actions=[tcp_bridge_launch]
    )

    start_tcp_force_probe = TimerAction(
        period=6.2,
        actions=[tcp_force_probe_launch]
    )

    start_environment_point_cloud = TimerAction(
        period=6.5,
        actions=[environment_point_cloud_launch]
    )

    start_workspace_sampler = TimerAction(
        period=6.8,
        actions=[workspace_sampler_launch]
    )

    start_tcp_path_visual = TimerAction(
        period=7.0,
        actions=[tcp_path_visual_launch]
    )

    start_environment_collision_generator = TimerAction(
        period=7.2,
        actions=[environment_collision_generator_launch]
    )

    start_rviz = TimerAction(
        period=8.0,
        actions=[rviz_launch]
    )

    start_state_monitor = TimerAction(
        period=9.0,
        actions=[state_monitor_launch]
    )

    # start_compliant_placement = TimerAction(
    #     period=9.5,
    #    actions=[compliant_placement_launch]
    # )

    start_task_manager = TimerAction(
        period=10.0,
        actions=[task_manager_launch]
    )

    return LaunchDescription([
        robot_ip_arg,
        cs_type_arg,
        tf_prefix_arg,
        use_fake_hardware_arg,
        safety_limits_arg,
        safety_pos_margin_arg,
        safety_k_position_arg,
        fake_sensor_commands_arg,
        headless_mode_arg,
        use_tool_communication_arg,
        tool_parity_arg,
        tool_baud_rate_arg,
        tool_stop_bits_arg,
        tool_tcp_port_arg,
        tool_data_bits_arg,
        tool_flow_control_arg,
        tool_device_path_arg,
        tool_timeout_arg,
        tool_voltage_arg,
        tool_current_arg,
        tool_type_arg,
        tool_power_arg,
        tool_state_arg,
        tool_enable_arg,
        local_ip_arg,
        remote_ip_arg,
        local_port_arg,
        remote_port_arg,
        script_command_port_arg,
        script_sender_port_arg,
        reverse_port_arg,
        trajectory_port_arg,
        controller_spawner_timeout_arg,
        runtime_config_package_arg,
        description_package_arg,
        description_file_arg,
        initial_joint_controller_arg,
        activate_joint_controller_arg,
        launch_rviz_arg,
        environment_point_cloud_directory_arg,
        environment_point_cloud_topic_arg,
        environment_point_cloud_frame_id_arg,
        environment_point_cloud_publish_interval_sec_arg,
        environment_point_cloud_loop_playback_arg,
        environment_point_cloud_use_xterm_arg,
        workspace_sampler_use_xterm_arg,
        start_driver,
        start_moveit,
        start_cs625_kinematics,
        start_preview_display,
        start_tcp_bridge,
        start_tcp_force_probe,
        start_environment_point_cloud,
        start_workspace_sampler,
        start_tcp_path_visual,
        start_environment_collision_generator,
        start_rviz,
        start_state_monitor,
        # start_compliant_placement,
        start_task_manager,
    ])
