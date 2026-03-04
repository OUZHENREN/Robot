# /home/yff/elite_ros_ws/src/tcp_bridge/tcp_bridge/pose_sender_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
import socket
import math
import time

class PoseSenderNode(Node):
    def __init__(self):
        super().__init__('pose_sender_node')
      
        self.robot_ip = '192.168.1.100'
        self.robot_port = 7000
      
        self.distance_threshold = 0.01 # 1厘米
      
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
      
        self.click_subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.click_callback,
            10)

        self.get_logger().info('Pose Sender Node with distance check logic is running.')
        self.get_logger().info(f'Distance threshold is set to: {self.distance_threshold} meters.')
        self.get_logger().info('Waiting for a click on /clicked_point from RViz...')

    def click_callback(self, msg):
        self.get_logger().info('RViz click detected! Evaluating command based on distance...')
      
        try:
            when = rclpy.time.Time()
            trans_tool0 = self.tf_buffer.lookup_transform('base_link', 'tool0', when, timeout=rclpy.duration.Duration(seconds=1.0))
          
            translation_tool0 = trans_tool0.transform.translation
            rotation_quat_tool0 = trans_tool0.transform.rotation
            pos_x_mm = translation_tool0.x * 1000.0
            pos_y_mm = translation_tool0.y * 1000.0
            pos_z_mm = translation_tool0.z * 1000.0
            r = R.from_quat([rotation_quat_tool0.x, rotation_quat_tool0.y, rotation_quat_tool0.z, rotation_quat_tool0.w])
            euler_rad = r.as_euler('xyz', degrees=False)
            rot_rx_deg = math.degrees(euler_rad[0])
            rot_ry_deg = math.degrees(euler_rad[1])
            rot_rz_deg = math.degrees(euler_rad[2])

            command_prefix = "T1"

            try:
                # --- 【核心修正】: 将查找的目标从 'target_pose' 改为 'target_pose_s1' ---
                trans_target = self.tf_buffer.lookup_transform('base_link', 'target_pose_s1', when, timeout=rclpy.duration.Duration(seconds=0.5))
              
                # 日志也同步修改，以便调试
                self.get_logger().info("'/target_pose_s1' found. Proceeding with distance check.")
                pos_target = trans_target.transform.translation

                distance = math.sqrt(
                    (translation_tool0.x - pos_target.x)**2 +
                    (translation_tool0.y - pos_target.y)**2 +
                    (translation_tool0.z - pos_target.z)**2
                )
                # 日志也同步修改
                self.get_logger().info(f'Distance between tool0 and target_pose_s1: {distance:.4f} meters.')

                if distance < self.distance_threshold:
                    self.get_logger().info('Distance is within threshold. Selecting command T2.')
                    command_prefix = "T2"
                else:
                    self.get_logger().info('Distance is too far. Selecting command T1.')

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # 日志也同步修改
                self.get_logger().info("'/target_pose_s1' not found. This is likely the first point in a cycle. Selecting command T1.")
                command_prefix = "T1"

            pose_str = f"{command_prefix},{pos_x_mm:.3f},{pos_y_mm:.3f},{pos_z_mm:.3f},{rot_rx_deg:.3f},{rot_ry_deg:.3f},{rot_rz_deg:.3f}\n"
            self.send_tcp_command(pose_str)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"CRITICAL ERROR: Could not get tool0's pose. Exception: {e}")

    # send_tcp_command 函数保持不变
    def send_tcp_command(self, command):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5)
                s.connect((self.robot_ip, self.robot_port))
                s.sendall(command.encode('utf-8'))
                time.sleep(0.1)
                self.get_logger().info(f"Successfully sent command: '{command.strip()}' to {self.robot_ip}:{self.robot_port}")
        except Exception as e:
            self.get_logger().error(f"An error occurred while sending TCP command: {e}")

# main 函数保持不变
def main(args=None):
    rclpy.init(args=args)
    node = PoseSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
