# 文件：eli_cs_robot_driver/launch/cs625_driver.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 参数声明（仅驱动需要的部分）
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.1.200')
    cs_type_arg = DeclareLaunchArgument('cs_type', default_value='cs625')
    use_fake_hardware_arg = DeclareLaunchArgument('use_fake_hardware', default_value='false')
    safety_limits_arg = DeclareLaunchArgument('safety_limits', default_value='false')
    safety_pos_margin_arg = DeclareLaunchArgument('safety_pos_margin', default_value='0.0')
    safety_k_position_arg = DeclareLaunchArgument(
        'safety_k_position', default_value='1.0', description='Safety position controller gain')

    driver_args_dict = {
        'robot_ip': LaunchConfiguration('robot_ip'),
        'cs_type': LaunchConfiguration('cs_type'),
        'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
        'safety_limits': LaunchConfiguration('safety_limits'),
        'safety_pos_margin': LaunchConfiguration('safety_pos_margin'),
        'safety_k_position': LaunchConfiguration('safety_k_position'),
    }

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('eli_cs_robot_driver'),
                'launch',
                'elite_control.launch.py'
            ])
        ]),
        launch_arguments=driver_args_dict.items()
    )

    return LaunchDescription([
        robot_ip_arg,
        cs_type_arg,
        use_fake_hardware_arg,
        safety_limits_arg,
        safety_pos_margin_arg,
        safety_k_position_arg,
        driver_launch
    ])
