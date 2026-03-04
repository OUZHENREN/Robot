import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取 MoveIt 配置包路径
    moveit_config_pkg = "elite_cs625_moveit_config"
    moveit_config_share = get_package_share_directory(moveit_config_pkg)
    
    # 2. 包含 MoveIt 官方生成的 demo.launch.py
    # 显式传递参数，确保环境与单独运行 demo.launch.py 时一致
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'demo.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'true',
            'use_sim_time': 'false', 
            # 如果您的 demo.launch.py 依赖 debug 参数，也可以加上
        }.items()
    )

    # 3. 定义需要弹出窗口运行的节点列表
    nodes_to_launch = [
        ("TCP_Server", "ros2 run tcp_bridge tcp_server"),
        ("Robot_Commander", "ros2 run tcp_bridge robot_commander"),
        ("TF_Broadcaster", "ros2 run tcp_bridge pose_to_tf_broadcaster"),
        ("GLB_Model_Publisher", "ros2 run tcp_bridge static_model_publisher"),
    ]

    launch_processes = [moveit_demo]

    # 4. 为每个节点创建 gnome-terminal 命令
    for title, cmd in nodes_to_launch:
        process = ExecuteProcess(
            cmd=['gnome-terminal', '--title', title, '--', 'bash', '-c', f'{cmd}; exec bash'],
            output='screen'
        )
        launch_processes.append(process)

    return LaunchDescription(launch_processes)
