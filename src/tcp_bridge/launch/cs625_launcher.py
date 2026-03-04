# 文件路径: /home/yff/elite_ros_ws/src/tcp_bridge/launch/cs625_launcher.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. tcp_server 节点
        Node(
            package='tcp_bridge',
            executable='tcp_server',
            name='tcp_server',
            output='screen',
            prefix='xterm -e',
        ),

        # 2. robot_commander 节点
        Node(
            package='tcp_bridge',
            executable='robot_commander',
            name='robot_commander',
            output='screen',
            prefix='xterm -e',
        ),

        # 3. pose_to_tf_broadcaster 节点（用于 TF 广播）
        Node(
            package='tcp_bridge',
            executable='pose_to_tf_broadcaster',
            name='pose_to_tf_broadcaster',
            output='screen',
            prefix='xterm -e',
        ),

        # 4. static_model_publisher 节点（静态模型发布节点）
        Node(
            package='tcp_bridge',
            executable='static_model_publisher',
            name='static_model_publisher',
            output='screen',
            prefix='xterm -e',
        ),

        # 5. pose_sender_node 节点（监听 /clicked_point，发送 S 消息）
        Node(
            package='tcp_bridge',
            executable='pose_sender_node',
            name='pose_sender_node',
            output='screen',
            prefix='xterm -e',
        ),
    ])
