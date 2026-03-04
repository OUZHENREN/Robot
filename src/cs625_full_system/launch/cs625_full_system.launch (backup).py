from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # 定义所有需要的参数
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.1.200')
    cs_type_arg = DeclareLaunchArgument('cs_type', default_value='cs625')
    tf_prefix_arg = DeclareLaunchArgument('tf_prefix', default_value='')

    # 传递给子系统的参数字典（关键修改已在下方标出）
    launch_args_dict = {
        'robot_ip': LaunchConfiguration('robot_ip'),
        'cs_type': LaunchConfiguration('cs_type'),
        'tf_prefix': LaunchConfiguration('tf_prefix'),
        'name': 'cs625',  # 为URDF提供必需的 'name' 参数
        'description_package': 'eli_cs_robot_description',
        'description_file': 'cs625.urdf.xacro',
        'initial_joint_controller': 'arm_controller',
        'activate_joint_controller': 'true',
        'launch_rviz': 'false',
        'runtime_config_package': 'eli_cs_robot_driver',
        # === 关键修正 START ===
        'controllers_file': 'cs625_controllers.yaml',
        'ros2_controllers_file': PathJoinSubstitution([
            FindPackageShare('eli_cs_robot_driver'), 'config', 'cs625_controllers.yaml'
        ]),
        # === 关键修正 END ===
    }

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('eli_cs_robot_driver'), 'launch', 'elite_control.launch.py'])
        ]),
        launch_arguments=launch_args_dict.items()
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('elite_cs625_moveit_config'), 'launch', 'move_group.launch.py'])
        ]),
        launch_arguments=launch_args_dict.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('elite_cs625_moveit_config'), 'launch', 'moveit_rviz.launch.py'])
        ]),
        launch_arguments=launch_args_dict.items()
    )

    custom_nodes = [
        Node(package='tcp_bridge', executable='tcp_server', name='tcp_server', output='screen'),
        Node(package='tcp_bridge', executable='robot_commander', name='robot_commander', output='screen'),
        Node(package='tcp_bridge', executable='pose_to_tf_broadcaster', name='pose_to_tf_broadcaster', output='screen'),
        Node(package='tcp_bridge', executable='static_model_publisher', name='static_model_publisher', output='screen'),
        Node(package='tcp_bridge', executable='pose_sender_node', name='pose_sender_node', output='screen'),
    ]

    return LaunchDescription([
        robot_ip_arg, cs_type_arg, tf_prefix_arg,
        driver_launch,
        moveit_launch,
        rviz_launch,
        *custom_nodes,
    ])
