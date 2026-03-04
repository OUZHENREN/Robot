# 文件路径: ~/elite_ros_ws/src/model_visualizer/model_visualizer/static_model_publisher.py

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, DurabilityPolicy
from pathlib import Path
import os
from scipy.spatial.transform import Rotation

class StaticModelPublisher(Node):
    def __init__(self):
        super().__init__('static_model_publisher')

        # 声明所有可配置的参数及其默认值
        self.declare_parameter('model_path', '/mnt/hgfs/Model/slot_pointcloud.glb')
        self.declare_parameter('pose.position.x', 0.0)
        self.declare_parameter('pose.position.y', 0.0)
        self.declare_parameter('pose.position.z', 0.0)
        self.declare_parameter('pose.orientation.roll', 0.0)  # 绕X轴旋转 (度)
        self.declare_parameter('pose.orientation.pitch', 0.0) # 绕Y轴旋转 (度)
        self.declare_parameter('pose.orientation.yaw', 0.0)   # 绕Z轴旋转 (度)

        # 使用持久化的QoS，确保RViz后启动也能看到模型
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(Marker, '/model_marker', qos_profile)

        self.timer = self.create_timer(1.0, self.publish_model_marker)
        self.get_logger().info("Node started. Will publish GLB model marker in 1 second.")

    def get_param(self, name):
        # 辅助函数，简化参数获取
        return self.get_parameter(name).get_parameter_value()

    def publish_model_marker(self):
        model_path_str = self.get_param('model_path').string_value
        self.get_logger().info(f"Attempting to load model from: {model_path_str}")

        # --- 文件检查 ---
        model_path = Path(model_path_str)
        if not model_path.is_file():
            self.get_logger().error(f"MODEL FILE NOT FOUND at: '{model_path_str}'")
            return

        # --- 获取位置和姿态参数 ---
        pos_x = self.get_param('pose.position.x').double_value
        pos_y = self.get_param('pose.position.y').double_value
        pos_z = self.get_param('pose.position.z').double_value
        roll = self.get_param('pose.orientation.roll').double_value
        pitch = self.get_param('pose.orientation.pitch').double_value
        yaw = self.get_param('pose.orientation.yaw').double_value

        self.get_logger().info(f"Placing model at Position(x,y,z): [{pos_x}, {pos_y}, {pos_z}]")
        self.get_logger().info(f"With Orientation(roll,pitch,yaw): [{roll}, {pitch}, {yaw}] degrees")
        
        # --- 将欧拉角(度)转换为四元数 ---
        r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        q = r.as_quat()  # Scipy返回 [x, y, z, w] 格式的四元数

        # --- 配置Marker消息 ---
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "model_display"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        # 指定模型资源
        marker.mesh_resource = "file://" + model_path_str
        marker.mesh_use_embedded_materials = True

        # 设置位置 (强制float)
        marker.pose.position.x = float(pos_x)
        marker.pose.position.y = float(pos_y)
        marker.pose.position.z = float(pos_z)

        # 设置姿态 (从scipy转换的结果)
        marker.pose.orientation.x = float(q[0])
        marker.pose.orientation.y = float(q[1])
        marker.pose.orientation.z = float(q[2])
        marker.pose.orientation.w = float(q[3])

        # 设置缩放 (mm 到 mm)
        marker.scale.x = float(1)
        marker.scale.y = float(1)
        marker.scale.z = float(1)

        # 设置颜色和透明度 (使用内嵌材质时此项被忽略)
        marker.color.a = float(1.0)
        marker.color.r = float(1.0)
        marker.color.g = float(1.0)
        marker.color.b = float(1.0)
        
        self.publisher_.publish(marker)
        self.get_logger().info(f"SUCCESS: Published MESH marker.")

        # 发布后取消定时器，节点进入空闲等待状态
        self.timer.cancel()
        self.get_logger().info("Marker published. Node is now idle.")

def main(args=None):
    rclpy.init(args=args)
    node = StaticModelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


