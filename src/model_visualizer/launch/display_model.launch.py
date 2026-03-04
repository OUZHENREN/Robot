# 文件路径: ~/elite_ros_ws/src/model_visualizer/launch/display_model.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动TF发布节点，创建 base_link 坐标系
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
            output='screen'
        ),

        # 2. 启动我们的模型发布节点
        Node(
            package='model_visualizer',
            executable='static_model_publisher',
            name='my_model_publisher',
            output='screen',
            parameters=[
                # 在这里指定你的模型位置和姿态！
                # --- 自定义区域 ---
                {'pose.position.x': 1.334050},   # 向前移动 0.5 米
                {'pose.position.y': -0.121929},  # 向左移动 0.2 米
                {'pose.position.z': 0.192299},   # 向上移动 0.3 米
                {'pose.orientation.roll': 179.441},
                {'pose.orientation.pitch': 178.980}, # 绕Y轴旋转90度
                {'pose.orientation.yaw': 89.357},   # 绕Z轴旋转45度
                # --- 结束 ---
            ]
        ),
    ])


