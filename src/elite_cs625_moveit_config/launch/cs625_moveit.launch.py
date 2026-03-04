# 文件：elite_cs625_moveit_config/launch/cs625_moveit.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 参数声明（参照原 launch_args_dict）
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.1.200')
    cs_type_arg = DeclareLaunchArgument('cs_type', default_value='cs625')
    tf_prefix_arg = DeclareLaunchArgument('tf_prefix', default_value='')
    use_fake_hardware_arg = DeclareLaunchArgument('use_fake_hardware', default_value='false')
    safety_limits_arg = DeclareLaunchArgument('safety_limits', default_value='false')
    safety_pos_margin_arg = DeclareLaunchArgument('safety_pos_margin', default_value='0.0')
    safety_k_position_arg = DeclareLaunchArgument(
        'safety_k_position', default_value='1.0', description='Safety position controller gain')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        'fake_sensor_commands', default_value='false', description='Use fake sensor command inputs (for testing/simulation)')
    headless_mode_arg = DeclareLaunchArgument(
        'headless_mode', default_value='false', description='Run in headless mode (no GUI)')
    use_tool_communication_arg = DeclareLaunchArgument(
        'use_tool_communication', default_value='false', description='Enable tool communication')
    tool_parity_arg = DeclareLaunchArgument('tool_parity', default_value='0', description='Tool parity parameter')
    tool_baud_rate_arg = DeclareLaunchArgument('tool_baud_rate', default_value='115200', description='Tool baud rate')
    tool_stop_bits_arg = DeclareLaunchArgument('tool_stop_bits', default_value='1', description='Tool stop bits')
    tool_tcp_port_arg = DeclareLaunchArgument('tool_tcp_port', default_value='502', description='Tool TCP port')
    tool_data_bits_arg = DeclareLaunchArgument('tool_data_bits', default_value='8', description='Tool data bits')
    tool_flow_control_arg = DeclareLaunchArgument('tool_flow_control', default_value='none', description='Tool flow control')
    tool_device_path_arg = DeclareLaunchArgument('tool_device_path', default_value='', description='Tool device path')
    tool_timeout_arg = DeclareLaunchArgument('tool_timeout', default_value='1.0', description='Tool timeout')

    tool_voltage_arg = DeclareLaunchArgument('tool_voltage', default_value='24', description='Tool voltage')
    tool_current_arg = DeclareLaunchArgument('tool_current', default_value='0', description='Tool current')
    tool_type_arg = DeclareLaunchArgument('tool_type', default_value='0', description='Tool type')
    tool_power_arg = DeclareLaunchArgument('tool_power', default_value='0', description='Tool power')
    tool_state_arg = DeclareLaunchArgument('tool_state', default_value='0', description='Tool state')
    tool_enable_arg = DeclareLaunchArgument('tool_enable', default_value='false', description='Enable tool')

    local_ip_arg = DeclareLaunchArgument('local_ip', default_value='0.0.0.0', description='Local IP address to bind to')
    remote_ip_arg = DeclareLaunchArgument('remote_ip', default_value='192.168.1.201', description='Remote robot/server IP')
    local_port_arg = DeclareLaunchArgument('local_port', default_value='0', description='Local port')
    remote_port_arg = DeclareLaunchArgument('remote_port', default_value='502', description='Remote port')

    script_command_port_arg = DeclareLaunchArgument('script_command_port', default_value='30003', description='Script command TCP port')
    script_sender_port_arg = DeclareLaunchArgument('script_sender_port', default_value='30004', description='Script sender TCP port')
    reverse_port_arg = DeclareLaunchArgument('reverse_port', default_value='50001', description='Reverse communication TCP port')
    trajectory_port_arg = DeclareLaunchArgument('trajectory_port', default_value='50002', description='Trajectory communication TCP port')

    controller_spawner_timeout_arg = DeclareLaunchArgument(
        'controller_spawner_timeout', default_value='5.0', description='Timeout for controller spawner [seconds]')
    runtime_config_package_arg = DeclareLaunchArgument(
        'runtime_config_package', default_value='eli_cs_robot_driver', description='Runtime config package')
    description_package_arg = DeclareLaunchArgument(
        'description_package', default_value='eli_cs_robot_description', description='Robot description package')
    description_file_arg = DeclareLaunchArgument(
        'description_file', default_value='cs625.urdf.xacro', description='Robot xacro or urdf filename')
    initial_joint_controller_arg = DeclareLaunchArgument(
        'initial_joint_controller', default_value='arm_controller', description='Initial joint controller name')
    activate_joint_controller_arg = DeclareLaunchArgument(
        'activate_joint_controller', default_value='true', description='Whether to activate controller automatically')
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='false', description='Whether to launch RViz')

    ros2_controllers_file = PathJoinSubstitution([
        FindPackageShare('eli_cs_robot_driver'), 'config', 'cs625_controllers.yaml'
    ])

    launch_args_dict = {
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

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('elite_cs625_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments=launch_args_dict.items()
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
        moveit_launch,
    ])
