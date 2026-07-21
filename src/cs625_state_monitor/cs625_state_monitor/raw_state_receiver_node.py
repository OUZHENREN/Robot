#!/usr/bin/env python3
import socket
import time

import rclpy
from rclpy.node import Node

from cs625_state_monitor.msg import CS625State
from cs625_state_monitor.primary_parser import PrimaryStateParser
from std_msgs.msg import Float64MultiArray


class RawStateReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__('cs625_raw_state_receiver_node')

        self.declare_parameter('robot_ip', '192.168.1.200')
        self.declare_parameter('robot_port', 30001)
        self.declare_parameter('recv_buffer_size', 4096)
        self.declare_parameter('socket_timeout_sec', 0.1)
        self.declare_parameter('reconnect_interval_sec', 1.0)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        self.recv_buffer_size = self.get_parameter('recv_buffer_size').value
        self.socket_timeout_sec = self.get_parameter('socket_timeout_sec').value
        self.reconnect_interval_sec = self.get_parameter('reconnect_interval_sec').value

        self.publisher_ = self.create_publisher(CS625State, '/cs625/raw_state', 10)
        self.parser = PrimaryStateParser()
        self.sock = None

        self.has_tcp_force = False
        self.latest_tcp_force = [0.0] * 6

        self.tcp_force_sub_ = self.create_subscription(
            Float64MultiArray,
            '/cs625/tcp_force',
            self.tcp_force_callback,
            10
        )

        self.get_logger().info(f'Raw state receiver started for {self.robot_ip}:{self.robot_port}')
        self.timer = self.create_timer(0.001, self.loop_once)

    def connect_socket(self) -> bool:
        try:
            self.sock = socket.create_connection((self.robot_ip, self.robot_port), timeout=5.0)
            self.sock.settimeout(self.socket_timeout_sec)
            self.get_logger().info(f'Connected to robot {self.robot_ip}:{self.robot_port}')
            return True
        except Exception as e:
            self.get_logger().warning(f'Connect failed: {e}')
            self.sock = None
            return False

    def tcp_force_callback(self, msg: Float64MultiArray) -> None:
        if msg is None:
            return

        if len(msg.data) < 6:
            return

        self.latest_tcp_force = list(msg.data[:6])
        self.has_tcp_force = True

    def disconnect_socket(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def loop_once(self) -> None:
        if self.sock is None:
            if not self.connect_socket():
                time.sleep(self.reconnect_interval_sec)
            return

        try:
            data = self.sock.recv(self.recv_buffer_size)
            if not data:
                self.get_logger().warning('Socket closed by peer')
                self.disconnect_socket()
                time.sleep(self.reconnect_interval_sec)
                return

            parsed_results = self.parser.feed(data)
            for parsed in parsed_results:
                msg = self.build_msg(parsed)
                self.publisher_.publish(msg)

        except socket.timeout:
            return
        except Exception as e:
            self.get_logger().warning(f'Receive error: {e}')
            self.disconnect_socket()
            time.sleep(self.reconnect_interval_sec)

    def build_msg(self, parsed: dict) -> CS625State:
        msg = CS625State()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.parser_ok = parsed.get('parser_ok', False)
        msg.has_joint_data = parsed.get('has_joint_data', False)
        msg.has_actual_tool_data = parsed.get('has_actual_tool_data', False)
        msg.has_target_tool_data = parsed.get('has_target_tool_data', False)

        joint_data = parsed.get('joint_data')
        if joint_data:
            msg.actual_joint_deg = joint_data['actual_joint_deg']
            msg.target_joint_deg = joint_data['target_joint_deg']
            msg.actual_velocity_deg = joint_data['actual_velocity_deg']
            msg.joint_current = joint_data['joint_current']
            msg.joint_voltage = joint_data['joint_voltage']
            msg.joint_temperature = joint_data['joint_temperature']
            msg.joint_torques = joint_data['joint_torques']

        actual_tool = parsed.get('actual_tool_data')
        if actual_tool:
            msg.actual_tcp_x_mm = actual_tool['tcp_x_mm']
            msg.actual_tcp_y_mm = actual_tool['tcp_y_mm']
            msg.actual_tcp_z_mm = actual_tool['tcp_z_mm']
            msg.actual_rot_x_deg = actual_tool['rot_x_deg']
            msg.actual_rot_y_deg = actual_tool['rot_y_deg']
            msg.actual_rot_z_deg = actual_tool['rot_z_deg']
            msg.actual_tcp_offset_x_mm = actual_tool['tcp_offset_x_mm']
            msg.actual_tcp_offset_y_mm = actual_tool['tcp_offset_y_mm']
            msg.actual_tcp_offset_z_mm = actual_tool['tcp_offset_z_mm']
            msg.actual_tcp_offset_rx_deg = actual_tool['tcp_offset_rx_deg']
            msg.actual_tcp_offset_ry_deg = actual_tool['tcp_offset_ry_deg']
            msg.actual_tcp_offset_rz_deg = actual_tool['tcp_offset_rz_deg']

        target_tool = parsed.get('target_tool_data')
        if target_tool:
            msg.target_tcp_x_mm = target_tool['tcp_x_mm']
            msg.target_tcp_y_mm = target_tool['tcp_y_mm']
            msg.target_tcp_z_mm = target_tool['tcp_z_mm']
            msg.target_rot_x_deg = target_tool['rot_x_deg']
            msg.target_rot_y_deg = target_tool['rot_y_deg']
            msg.target_rot_z_deg = target_tool['rot_z_deg']
            msg.target_tcp_offset_x_mm = target_tool['tcp_offset_x_mm']
            msg.target_tcp_offset_y_mm = target_tool['tcp_offset_y_mm']
            msg.target_tcp_offset_z_mm = target_tool['tcp_offset_z_mm']
            msg.target_tcp_offset_rx_deg = target_tool['tcp_offset_rx_deg']
            msg.target_tcp_offset_ry_deg = target_tool['tcp_offset_ry_deg']
            msg.target_tcp_offset_rz_deg = target_tool['tcp_offset_rz_deg']

        msg.has_actual_tcp_wrench = self.has_tcp_force

        if self.has_tcp_force:
            msg.actual_tcp_force_x = float(self.latest_tcp_force[0])
            msg.actual_tcp_force_y = float(self.latest_tcp_force[1])
            msg.actual_tcp_force_z = float(self.latest_tcp_force[2])
            msg.actual_tcp_torque_x = float(self.latest_tcp_force[3])
            msg.actual_tcp_torque_y = float(self.latest_tcp_force[4])
            msg.actual_tcp_torque_z = float(self.latest_tcp_force[5])

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = RawStateReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disconnect_socket()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
