# 文件路径: /home/yff/elite_ros_ws/src/tcp_bridge/tcp_bridge/robot_commander.py
# 版本: 严格遵守手册规范的最终版 (基于v_final_fix进行修正)

import rclpy
from rclpy.node import Node
import socket
import threading
import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

# --- 配置正确 ---
HOST = '0.0.0.0'
PORT = 60000

class RobotCommander(Node):
    def __init__(self):
        super().__init__('robot_commander')
        self.get_logger().info('Robot Commander node started.')
        self.client_socket = None
        self.client_address = None
        self.connection_lock = threading.Lock()

        self.server_thread = threading.Thread(target=self.start_server)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.pose_callback,
            10)
        self.joint_sub = self.create_subscription(
            JointState,
            '/target_joint_states',
            self.joint_callback,
            10)
        
        self.get_logger().info(f"Node is ready. SERVER is listening on {HOST}:{PORT} for the robot to connect.")

    def start_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind((HOST, PORT))
                s.listen()
            except Exception as e:
                self.get_logger().fatal(f"FATAL: Could not bind to {HOST}:{PORT}. Error: {e}. Shutting down.")
                rclpy.shutdown()
                return

            while rclpy.ok():
                try:
                    conn, addr = s.accept()
                    with self.connection_lock:
                        if self.client_socket:
                            self.client_socket.close()
                        self.client_socket = conn
                        self.client_address = addr
                    self.get_logger().info(f'SUCCESS: Robot connected from: {addr}')
                    self.monitor_connection(conn)
                except Exception as e:
                    if rclpy.ok(): self.get_logger().error(f"Server 'accept' loop error: {e}")
                finally:
                    with self.connection_lock:
                        if self.client_socket: self.client_socket.close()
                        self.client_socket = None
                    if rclpy.ok(): self.get_logger().warn('Robot disconnected. Awaiting new connection...')
                    time.sleep(1)
    
    def monitor_connection(self, conn):
        while rclpy.ok() and self.client_socket:
            try:
                data = conn.recv(1, socket.MSG_PEEK)
                if not data: break
                time.sleep(2)
            except BlockingIOError:
                time.sleep(2)
                continue
            except Exception:
                break

    def send_command(self, cmd_type, data_tuple):
        with self.connection_lock:
            if not self.client_socket:
                self.get_logger().warn(f'Command ({cmd_type}) dropped: No robot connected.')
                return False
            try:
                self.client_socket.sendall(cmd_type.encode('ascii'))
                self.client_socket.settimeout(5.0)
                response = self.client_socket.recv(1024).decode('ascii').strip()
                if 'ready' not in response:
                    self.get_logger().error(f'Robot communication error. Expected "ready", but got: "{response}"')
                    return False
                
                # ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
                # ★★★ [最终修正] 严格按照手册7.1.5节规定，使用带括号和逗号的格式 ★★★
                # "data_string = ' '.join(map(str, data_tuple)) + '\n'" 是错误的
                # 正确格式: "(num1,num2,num3,num4,num5,num6)"
                data_string = f"({','.join(map(str, data_tuple))})"
                # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

                self.client_socket.sendall(data_string.encode('ascii'))
                # 更新了日志，以打印实际发送的字符串，方便调试
                self.get_logger().info(f"Command Sent -> Type: {cmd_type}, Data: {data_string}")
                return True
            except socket.timeout:
                self.get_logger().error('Socket timeout while sending command. Connection may be lost.')
            except Exception as e:
                self.get_logger().error(f'Failed to send command. Error: {e}. Connection lost.')
                self.client_socket.close()
                self.client_socket = None
            return False

    def pose_callback(self, msg: PoseStamped):
        # --- 姿态表示正确 ---
        self.get_logger().debug('Received a pose command.')
        p = msg.pose.position
        q = msg.pose.orientation
        r = R.from_quat([q.x, q.y, q.z, q.w])
        
        euler_angles = r.as_euler('xyz', degrees=False) # 使用弧度制

        pose_data = (
            round(p.x, 6), round(p.y, 6), round(p.z, 6), 
            round(euler_angles[0], 6), round(euler_angles[1], 6), round(euler_angles[2], 6)
        )
        self.send_command('P', pose_data)

    def joint_callback(self, msg: JointState):
        self.get_logger().debug('Received a joint command.')
        if len(msg.position) >= 6:
            joint_data = tuple(round(p, 6) for p in msg.position[:6])
            self.send_command('J', joint_data)
        else:
            self.get_logger().warn(f'Received JointState with < 6 positions ({len(msg.position)}). Dropped.')

    def destroy_node(self):
        self.get_logger().info("Shutting down Robot Commander, closing socket.")
        with self.connection_lock:
            if self.client_socket:
                self.client_socket.close()
        super().destroy_node()

    def main_loop(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info('KeyboardInterrupt received, shutting down.')
        finally:
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    commander_node = RobotCommander()
    commander_node.main_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

