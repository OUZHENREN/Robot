# 文件路径: /home/yff/elite_ros_ws/src/tcp_bridge/tcp_bridge/tcp_server.py
# 您提供的新版本

import socket
import rclpy
from rclpy.node import Node
import threading
import math
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

class TCPServer(Node):
    def __init__(self, node_name='tcp_server'):
        super().__init__(node_name)

        self.pose_publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.get_logger().info('Publisher for S-messages on "/target_pose" is ready.')

        self.model_param_client = self.create_client(SetParameters, '/static_model_publisher/set_parameters')
        if not self.model_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Service "/static_model_publisher/set_parameters" not available. R-messages may not work.')
        else:
            self.get_logger().info('Client for R-messages to "/static_model_publisher" is ready.')

        self.host = '0.0.0.0'
        self.port = 9999
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))

    def listen_for_clients(self):
        self.server_socket.listen(5)
        self.get_logger().info(f"TCP Server listening on ({self.host}, {self.port})...")
        while rclpy.ok():
            try:
                conn, addr = self.server_socket.accept()
                threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True).start()
            except (socket.error, KeyboardInterrupt):
                if rclpy.ok(): self.get_logger().info("Server socket operation interrupted.")
                break

    def handle_client(self, conn, addr):
        with conn:
            try:
                data = conn.recv(1024)
                if not data: return
                received_str = data.decode('utf-8').strip()
                self.get_logger().info(f"Received from {addr}: '{received_str}'")
                
                if received_str.upper().startswith('S'):
                    self.parse_and_publish_pose(received_str)
                elif received_str.upper().startswith('R'):
                    self.parse_and_set_model_pose(received_str)
                else:
                    self.get_logger().warn(f"Unknown msg type: '{received_str[:1]}'. Must be 'S' or 'R'.")
            except Exception as e:
                self.get_logger().error(f"Error handling client {addr}: {e}")

    def parse_and_publish_pose(self, data_str):
        try:
            parts = data_str.split(',')
            if len(parts) != 7: return
            pose_name, values = parts[0], [float(p) for p in parts[1:]]
            x_m, y_m, z_m = values[0] / 1000.0, values[1] / 1000.0, values[2] / 1000.0
            rx_rad, ry_rad, rz_rad = map(math.radians, values[3:6])
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp, pose_msg.header.frame_id = self.get_clock().now().to_msg(), "world" # 确保frame_id是'world'
            pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = x_m, y_m, z_m
            q = R.from_euler('xyz', [rx_rad, ry_rad, rz_rad]).as_quat()
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q[0], q[1], q[2], q[3]
            
            self.pose_publisher.publish(pose_msg)
            self.get_logger().info(f"Published S-message for '{pose_name}' to /target_pose.")
        except Exception as e:
            self.get_logger().error(f"Failed to parse S-message: '{data_str}'. Error: {e}")

    def parse_and_set_model_pose(self, data_str):
        try:
            parts = data_str.split(',')
            if len(parts) != 7: return
            values = [float(p) for p in parts[1:]]
            x_m, y_m, z_m = values[0] / 1000.0, values[1] / 1000.0, values[2] / 1000.0
            roll_deg, pitch_deg, yaw_deg = values[3], values[4], values[5]
            
            # --- 部分1: 调用参数服务, 移动GLB模型 ---
            params = [
                Parameter(name='pose.position.x', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=x_m)),
                Parameter(name='pose.position.y', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=y_m)),
                Parameter(name='pose.position.z', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=z_m)),
                Parameter(name='pose.orientation.roll', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=roll_deg)),
                Parameter(name='pose.orientation.pitch', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=pitch_deg)),
                Parameter(name='pose.orientation.yaw', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=yaw_deg)),
            ]
            
            if self.model_param_client.service_is_ready():
                self.model_param_client.call_async(SetParameters.Request(parameters=params))
                self.get_logger().info(f"Sent R-message pose update to static_model_publisher.")
            else:
                self.get_logger().error("static_model_publisher service not ready!")

            # ### 核心修正：添加发布话题的逻辑 ###
            # --- 部分2: 发布 /target_pose 话题, 移动手臂和TF坐标系 ---
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world" # 确保frame_id是'world'
            pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = x_m, y_m, z_m
            
            # 注意: RPY单位是度, 需要转换为弧度再生成四元数
            q = R.from_euler('xyz', [math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)]).as_quat()
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q[0], q[1], q[2], q[3]

            self.pose_publisher.publish(pose_msg)
            self.get_logger().info(f"Also Published R-message to /target_pose.")
            # ### 修正结束 ###

        except Exception as e:
            self.get_logger().error(f"Failed to parse R-message: '{data_str}'. Error: {e}")

    def destroy_node(self):
        self.server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TCPServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=node.listen_for_clients, daemon=True).start()
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

