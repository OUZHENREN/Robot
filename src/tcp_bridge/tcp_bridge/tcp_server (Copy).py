# 文件路径: /home/yff/elite_ros_ws/src/tcp_bridge/tcp_bridge/tcp_server.py
# 功能: (最终修正版) 接收TCP数据(毫米,度)，并将其发布为PoseStamped消息(米,四元数)

import socket
import rclpy
from rclpy.node import Node
import time
import threading
import math

# 导入PoseStamped消息类型
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

class TCPServer(Node):
    def __init__(self, node_name='tcp_server'):
        super().__init__(node_name)
        
        self.pose_publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.get_logger().info('ROS 2 Publisher on "/target_pose" topic is ready.')

        self.host = '0.0.0.0'
        self.port = 9999
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.server_socket.bind((self.host, self.port))
        except socket.error as e:
            self.get_logger().error(f"Socket binding failed: {e}")
            rclpy.shutdown()

    def listen_for_clients(self):
        self.server_socket.listen(5)
        self.get_logger().info(f"TCP Server listening on ({self.host}, {self.port})...")
        
        while rclpy.ok():
            try:
                conn, addr = self.server_socket.accept()
                self.get_logger().info(f"Accepted connection from {addr}")
                client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                client_thread.daemon = True
                client_thread.start()
            except (socket.error, KeyboardInterrupt):
                if rclpy.ok(): self.get_logger().info("Server socket operation interrupted.")
                break
            except Exception as e:
                if rclpy.ok(): self.get_logger().error(f"An unexpected error in accept loop: {e}")

    def handle_client(self, conn, addr):
        with conn:
            try:
                data = conn.recv(1024)
                if not data:
                    self.get_logger().warn(f"Connection from {addr} closed with no data.")
                    return
                received_str = data.decode('utf-8').strip()
                self.get_logger().info(f"Received from {addr}: '{received_str}'")
                self.parse_and_publish_pose(received_str)
            except Exception as e:
                self.get_logger().error(f"Error handling client {addr}: {e}")
    
    def parse_and_publish_pose(self, data_str):
        try:
            parts = data_str.split(',')
            if len(parts) != 7:
                self.get_logger().warn(f"Invalid data format. Expected 7 parts (S1,x,y,z,rx,ry,rz), got {len(parts)}.")
                return

            pose_name = parts[0]
            values = [float(p) for p in parts[1:]]
            
            x_mm, y_mm, z_mm = values[0], values[1], values[2]
            rx_deg, ry_deg, rz_deg = values[3], values[4], values[5]
            
            # --- 核心修正 1: 单位转换 (毫米 -> 米) ---
            x_m = x_mm / 1000.0
            y_m = y_mm / 1000.0
            z_m = z_mm / 1000.0
            
            # --- 核心修正 2: 单位转换 (度 -> 弧度) ---
            rx_rad = math.radians(rx_deg)
            ry_rad = math.radians(ry_deg)
            rz_rad = math.radians(rz_deg)
            
            self.get_logger().info(f"Parsed and converted for ROS: position_meters=[{x_m:.3f}, {y_m:.3f}, {z_m:.3f}]")

            # 创建一个 PoseStamped 消息
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = pose_name

            # 2. 填充位置 Position (使用转换后的米单位)
            pose_msg.pose.position.x = x_m
            pose_msg.pose.position.y = y_m
            pose_msg.pose.position.z = z_m

            # 3. 填充姿态 Orientation (从转换后的弧度单位计算四元数)
            r = R.from_euler('xyz', [rx_rad, ry_rad, rz_rad])
            q = r.as_quat()
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]

            # 4. 发布消息
            self.pose_publisher.publish(pose_msg)
            self.get_logger().info(f"Successfully published PoseStamped message for '{pose_name}' to /target_pose topic.")

        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Failed to parse data: '{data_str}'. Error: {e}")

    def destroy_node(self):
        self.get_logger().info("Closing server socket...")
        self.server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    tcp_server_node = TCPServer()
    try:
        tcp_server_node.listen_for_clients()
    finally:
        tcp_server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

