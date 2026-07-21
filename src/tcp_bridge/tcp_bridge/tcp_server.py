#!/usr/bin/env python3

import socket
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation as R
import numpy as np


class TCPServer(Node):
    def __init__(self, node_name='tcp_server'):
        super().__init__(node_name)

        # ----------------------------
        # 状态型 Pose Topic 专用 QoS
        # 让晚启动订阅者也能拿到最近一次 slot / box pose
        # ----------------------------
        latched_pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ----------------------------
        # 主目标位姿：供机器人控制 / 兼容现有逻辑
        # ----------------------------
        self.pose_publisher = self.create_publisher(PoseStamped, '/target_pose', 10)

        # ----------------------------
        # 工件可视化专用位姿
        # /slot_target_pose 与 /box_target_pose 改为 transient_local
        # ----------------------------
        self.slot_pose_publisher = self.create_publisher(
            PoseStamped, '/slot_target_pose', latched_pose_qos
        )
        self.box_pose_publisher = self.create_publisher(
            PoseStamped, '/box_target_pose', latched_pose_qos
        )

        # ----------------------------
        # 精确拍摄位姿（S0 / S2）
        # ----------------------------
        self.slot_precision_view_tcp_publisher = self.create_publisher(
            PoseStamped, '/slot_precision_view_tcp_pose', 10
        )
        self.box_precision_view_tcp_publisher = self.create_publisher(
            PoseStamped, '/box_precision_view_tcp_pose', 10
        )

        # ----------------------------
        # 加工后的 TCP 目标位姿（抓取 / 插入）
        # ----------------------------
        self.box_grasp_tcp_publisher = self.create_publisher(PoseStamped, '/box_grasp_tcp_pose', 10)
        self.box_pre_grasp_tcp_publisher = self.create_publisher(PoseStamped, '/box_pre_grasp_tcp_pose', 10)

        self.slot_insert_tcp_publisher = self.create_publisher(PoseStamped, '/slot_insert_tcp_pose', 10)
        self.slot_pre_insert_rotated_tcp_publisher = self.create_publisher(
            PoseStamped, '/slot_pre_insert_rotated_tcp_pose', 10
        )
        self.slot_pre_insert_tcp_publisher = self.create_publisher(
            PoseStamped, '/slot_pre_insert_tcp_pose', 10
        )

        # ----------------------------
        # 取出流程准备服务
        # 基于最近一次 box 精定位结果，反推 slot 链
        # ----------------------------
        self.prepare_unload_flow_service = self.create_service(
            Trigger,
            '/prepare_unload_flow',
            self.handle_prepare_unload_flow
        )

        # 缓存最近一次的工件姿态与 TCP 目标
        self.last_box_pose = None
        self.last_slot_pose = None

        self.box_grasp_tcp_pose = None
        self.box_pre_grasp_tcp_pose = None

        self.slot_insert_tcp_pose = None
        self.slot_pre_insert_rotated_tcp_pose = None
        self.slot_pre_insert_tcp_pose = None

        self.get_logger().info('Publisher "/target_pose" is ready.')
        self.get_logger().info('Publisher "/slot_target_pose" is ready with TRANSIENT_LOCAL QoS.')
        self.get_logger().info('Publisher "/box_target_pose" is ready with TRANSIENT_LOCAL QoS.')
        self.get_logger().info('Publisher "/slot_precision_view_tcp_pose" is ready.')
        self.get_logger().info('Publisher "/box_precision_view_tcp_pose" is ready.')
        self.get_logger().info('Publisher "/box_grasp_tcp_pose" is ready.')
        self.get_logger().info('Publisher "/box_pre_grasp_tcp_pose" is ready.')
        self.get_logger().info('Publisher "/slot_insert_tcp_pose" is ready.')
        self.get_logger().info('Publisher "/slot_pre_insert_rotated_tcp_pose" is ready.')
        self.get_logger().info('Publisher "/slot_pre_insert_tcp_pose" is ready.')
        self.get_logger().info('Service "/prepare_unload_flow" is ready.')

        # ----------------------------
        # 固定变换矩阵：工件系 -> TCP（单位 m）
        # ----------------------------
        # 抓取时：ACSt 在 ACSb 下的变换
        self.T_b_tcp = np.array([
            [-1.0, 0.0, 0.0, -4.0 / 1000.0],
            [0.0,  1.0, 0.0, -215.0 / 1000.0],
            [0.0,  0.0, -1.0, -411.0 / 1000.0],
            [0.0,  0.0, 0.0, 1.0],
        ])

        # 插入时：ACSt 在 ACSs 下的变换
        self.T_s_tcp = np.array([
            [-1.0, 0.0, 0.0, -82.0 / 1000.0],
            [0.0,  1.0, 0.0, -193.0 / 1000.0],
            [0.0,  0.0, -1.0, -412.5 / 1000.0],
            [0.0,  0.0, 0.0, 1.0],
        ])

        # TCP Server
        self.host = '0.0.0.0'
        self.port = 9999
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))

    # ----------------------------
    # TCP 服务器主循环
    # ----------------------------
    def listen_for_clients(self):
        self.server_socket.listen(5)
        self.get_logger().info(f'TCP Server listening on ({self.host}, {self.port})...')
        while rclpy.ok():
            try:
                conn, addr = self.server_socket.accept()
                threading.Thread(
                    target=self.handle_client,
                    args=(conn, addr),
                    daemon=True
                ).start()
            except (socket.error, KeyboardInterrupt):
                if rclpy.ok():
                    self.get_logger().info('Server socket operation interrupted.')
                break

    def handle_client(self, conn, addr):
        with conn:
            try:
                data = conn.recv(1024)
                if not data:
                    return

                received_str = data.decode('utf-8').strip()
                self.get_logger().info(f"Received from {addr}: '{received_str}'")

                msg_head = received_str.split(',')[0].upper()

                if msg_head.startswith('S'):
                    self.handle_s_message(received_str)
                elif msg_head.startswith('R'):
                    self.handle_r_message(received_str)
                else:
                    self.get_logger().warn(
                        f"Unknown msg type: '{msg_head}'. Must start with 'S' or 'R'."
                    )

            except Exception as e:
                self.get_logger().error(f'Error handling client {addr}: {e}')

    # ----------------------------
    # 工具函数：文本 -> PoseStamped（world 下工件姿态）
    # ----------------------------
    def build_pose_msg(self, pose_name, values):
        """
        values: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        输出 PoseStamped, frame_id='world', 单位 m, 姿态为四元数
        """
        x_m = values[0] / 1000.0
        y_m = values[1] / 1000.0
        z_m = values[2] / 1000.0

        rx_rad = math.radians(values[3])
        ry_rad = math.radians(values[4])
        rz_rad = math.radians(values[5])

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'

        pose_msg.pose.position.x = x_m
        pose_msg.pose.position.y = y_m
        pose_msg.pose.position.z = z_m

        q = R.from_euler('xyz', [rx_rad, ry_rad, rz_rad]).as_quat()
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        return pose_msg

    def parse_pose(self, data_str):
        parts = data_str.split(',')
        if len(parts) != 7:
            raise ValueError(f'Invalid message format, expected 7 fields, got {len(parts)}: {data_str}')

        pose_name = parts[0].upper()
        values = [float(p) for p in parts[1:]]
        pose_msg = self.build_pose_msg(pose_name, values)
        return pose_name, values, pose_msg

    def pose_stamped_to_matrix(self, pose_msg: PoseStamped) -> np.ndarray:
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        R_world = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        T_world = np.eye(4)
        T_world[:3, :3] = R_world
        T_world[0, 3] = p.x
        T_world[1, 3] = p.y
        T_world[2, 3] = p.z

        return T_world

    def matrix_to_pose_stamped(self, T_world: np.ndarray) -> PoseStamped:
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'

        out.pose.position.x = float(T_world[0, 3])
        out.pose.position.y = float(T_world[1, 3])
        out.pose.position.z = float(T_world[2, 3])

        R_world = T_world[:3, :3]
        q = R.from_matrix(R_world).as_quat()
        out.pose.orientation.x = float(q[0])
        out.pose.orientation.y = float(q[1])
        out.pose.orientation.z = float(q[2])
        out.pose.orientation.w = float(q[3])

        return out

    # ----------------------------
    # 计算：box 抓取 TCP Pose
    # ----------------------------
    def compute_box_grasp_tcp(self, world_box_pose: PoseStamped) -> PoseStamped:
        T_world_box = self.pose_stamped_to_matrix(world_box_pose)
        T_world_tcp = T_world_box @ self.T_b_tcp
        return self.matrix_to_pose_stamped(T_world_tcp)

    def compute_box_pre_grasp_tcp(self, world_box_grasp_tcp_pose: PoseStamped) -> PoseStamped:
        """
        /box_pre_grasp_tcp_pose 定义：
        在 /box_grasp_tcp_pose 局部坐标系下，
        沿 x 方向偏移 120 mm，沿 z 方向偏移 0 mm
        """
        T_world_grasp = self.pose_stamped_to_matrix(world_box_grasp_tcp_pose)

        T_offset = np.eye(4)
        T_offset[0, 3] = 120 / 1000.0
        T_offset[2, 3] = 0 / 1000.0

        T_world_pre = T_world_grasp @ T_offset
        return self.matrix_to_pose_stamped(T_world_pre)

    # ----------------------------
    # 计算：slot 插入终点 / 中间旋转点 / 预插入点
    # ----------------------------
    def compute_slot_insert_tcp(self, world_slot_pose: PoseStamped) -> PoseStamped:
        T_world_slot = self.pose_stamped_to_matrix(world_slot_pose)
        T_world_tcp = T_world_slot @ self.T_s_tcp
        return self.matrix_to_pose_stamped(T_world_tcp)

    def compute_slot_pre_insert_rotated_tcp(self, world_slot_insert_tcp_pose: PoseStamped) -> PoseStamped:
        """
        中间姿态点：
        在 /slot_insert_tcp_pose 局部坐标系下绕 y 轴旋转 -10°
        仅改变姿态，不做平移偏移
        """
        T_world_insert = self.pose_stamped_to_matrix(world_slot_insert_tcp_pose)

        T_rot = np.eye(4)
        T_rot[:3, :3] = R.from_euler('y', -10.0, degrees=True).as_matrix()

        T_world_rotated = T_world_insert @ T_rot
        return self.matrix_to_pose_stamped(T_world_rotated)

    def compute_slot_pre_insert_tcp(self, world_slot_pre_insert_rotated_tcp_pose: PoseStamped) -> PoseStamped:
        """
        /slot_pre_insert_tcp_pose 定义：
        基于已旋转 -10° 的中间姿态，
        沿该新 pose 的 z 轴方向偏移 -350 mm
        """
        T_world_rotated = self.pose_stamped_to_matrix(world_slot_pre_insert_rotated_tcp_pose)

        T_trans = np.eye(4)
        T_trans[2, 3] = -350 / 1000.0

        T_world_pre = T_world_rotated @ T_trans
        return self.matrix_to_pose_stamped(T_world_pre)

    # ----------------------------
    # 取出流程准备：
    # 已知 box_origin，利用
    #   world_box_grasp_tcp = world_box_origin * T_b_tcp
    #   world_slot_insert_tcp = world_box_grasp_tcp
    #   world_slot_origin = world_slot_insert_tcp * inv(T_s_tcp)
    # 反推出 slot_origin，并重建整条 slot 链
    # ----------------------------
    def handle_prepare_unload_flow(self, request, response):
        del request

        if self.last_box_pose is None:
            response.success = False
            response.message = 'prepare_unload_flow failed: no last_box_pose available, please receive S3 first.'
            self.get_logger().warn(response.message)
            return response

        try:
            # 1) 基于最近一次 box_origin 计算 box_grasp_tcp
            self.box_grasp_tcp_pose = self.compute_box_grasp_tcp(self.last_box_pose)
            self.box_grasp_tcp_publisher.publish(self.box_grasp_tcp_pose)

            self.box_pre_grasp_tcp_pose = self.compute_box_pre_grasp_tcp(self.box_grasp_tcp_pose)
            self.box_pre_grasp_tcp_publisher.publish(self.box_pre_grasp_tcp_pose)

            # 2) 插入完成状态下，box_grasp_tcp 与 slot_insert_tcp 重合
            T_world_slot_insert = self.pose_stamped_to_matrix(self.box_grasp_tcp_pose)

            # 3) 反推 slot_origin
            T_world_slot = T_world_slot_insert @ np.linalg.inv(self.T_s_tcp)
            reconstructed_slot_pose = self.matrix_to_pose_stamped(T_world_slot)

            # 4) 再次正向计算整条 slot 链，确保逻辑与现有 R1 处理一致
            self.slot_insert_tcp_pose = self.compute_slot_insert_tcp(reconstructed_slot_pose)
            self.slot_pre_insert_rotated_tcp_pose = self.compute_slot_pre_insert_rotated_tcp(
                self.slot_insert_tcp_pose
            )
            self.slot_pre_insert_tcp_pose = self.compute_slot_pre_insert_tcp(
                self.slot_pre_insert_rotated_tcp_pose
            )

            # 5) 发布 slot 链
            self.last_slot_pose = reconstructed_slot_pose
            self.slot_pose_publisher.publish(self.last_slot_pose)
            self.slot_insert_tcp_publisher.publish(self.slot_insert_tcp_pose)
            self.slot_pre_insert_rotated_tcp_publisher.publish(self.slot_pre_insert_rotated_tcp_pose)
            self.slot_pre_insert_tcp_publisher.publish(self.slot_pre_insert_tcp_pose)

            self.get_logger().info(
                'prepare_unload_flow: reconstructed slot_target_pose '
                f'pos=({self.last_slot_pose.pose.position.x:.3f}, '
                f'{self.last_slot_pose.pose.position.y:.3f}, '
                f'{self.last_slot_pose.pose.position.z:.3f})'
            )
            self.get_logger().info(
                'prepare_unload_flow: published slot_insert_tcp_pose '
                f'pos=({self.slot_insert_tcp_pose.pose.position.x:.3f}, '
                f'{self.slot_insert_tcp_pose.pose.position.y:.3f}, '
                f'{self.slot_insert_tcp_pose.pose.position.z:.3f})'
            )
            self.get_logger().info(
                'prepare_unload_flow: published slot_pre_insert_rotated_tcp_pose '
                f'pos=({self.slot_pre_insert_rotated_tcp_pose.pose.position.x:.3f}, '
                f'{self.slot_pre_insert_rotated_tcp_pose.pose.position.y:.3f}, '
                f'{self.slot_pre_insert_rotated_tcp_pose.pose.position.z:.3f})'
            )
            self.get_logger().info(
                'prepare_unload_flow: published slot_pre_insert_tcp_pose '
                f'pos=({self.slot_pre_insert_tcp_pose.pose.position.x:.3f}, '
                f'{self.slot_pre_insert_tcp_pose.pose.position.y:.3f}, '
                f'{self.slot_pre_insert_tcp_pose.pose.position.z:.3f})'
            )

            response.success = True
            response.message = (
                'prepare_unload_flow succeeded: slot pose chain reconstructed from last_box_pose.'
            )
            self.get_logger().info(response.message)
            return response

        except Exception as e:
            response.success = False
            response.message = f'prepare_unload_flow failed: {e}'
            self.get_logger().error(response.message)
            return response

    # ----------------------------
    # S* 消息处理（含 S3 -> box 抓取计算）
    # ----------------------------
    def handle_s_message(self, data_str):
        try:
            pose_name, values, pose_msg = self.parse_pose(data_str)

            # 所有 S* 默认仍发布到 /target_pose，供现有机器人逻辑使用
            self.pose_publisher.publish(pose_msg)
            self.get_logger().info(f"Published {pose_name} to /target_pose.")

            if pose_name == 'S0':
                # S0: slot 精确拍摄位姿
                self.slot_precision_view_tcp_publisher.publish(pose_msg)
                self.get_logger().info(
                    f'Published {pose_name} to /slot_precision_view_tcp_pose.'
                )

            elif pose_name == 'S2':
                # S2: box 精确拍摄位姿
                self.box_precision_view_tcp_publisher.publish(pose_msg)
                self.get_logger().info(
                    f'Published {pose_name} to /box_precision_view_tcp_pose.'
                )

            elif pose_name == 'S3':
                # S3: box 精定位，视觉给的是 ACSb 在 world 下
                self.box_pose_publisher.publish(pose_msg)
                self.last_box_pose = pose_msg
                self.get_logger().info(
                    f'Published {pose_name} to /box_target_pose (ACSb visualization, latched).'
                )

                # 计算 TCP 抓取位姿
                self.box_grasp_tcp_pose = self.compute_box_grasp_tcp(pose_msg)
                self.box_grasp_tcp_publisher.publish(self.box_grasp_tcp_pose)
                self.get_logger().info(
                    'Computed & published box_grasp_tcp_pose '
                    f'pos=({self.box_grasp_tcp_pose.pose.position.x:.3f}, '
                    f'{self.box_grasp_tcp_pose.pose.position.y:.3f}, '
                    f'{self.box_grasp_tcp_pose.pose.position.z:.3f})'
                )

                # 计算 TCP 预抓取位姿
                self.box_pre_grasp_tcp_pose = self.compute_box_pre_grasp_tcp(self.box_grasp_tcp_pose)
                self.box_pre_grasp_tcp_publisher.publish(self.box_pre_grasp_tcp_pose)
                self.get_logger().info(
                    'Computed & published box_pre_grasp_tcp_pose '
                    f'pos=({self.box_pre_grasp_tcp_pose.pose.position.x:.3f}, '
                    f'{self.box_pre_grasp_tcp_pose.pose.position.y:.3f}, '
                    f'{self.box_pre_grasp_tcp_pose.pose.position.z:.3f})'
                )

            else:
                self.get_logger().info(
                    f'{pose_name} does not drive extra S-message targets. Only /target_pose updated.'
                )

        except Exception as e:
            self.get_logger().error(f"Failed to parse S-message: '{data_str}'. Error: {e}")

    # ----------------------------
    # R* 消息处理（含 R1 -> slot 插入终点计算）
    # ----------------------------
    def handle_r_message(self, data_str):
        try:
            pose_name, values, pose_msg = self.parse_pose(data_str)

            # R* 默认也发布到 /target_pose，保留现有逻辑兼容性
            self.pose_publisher.publish(pose_msg)
            self.get_logger().info(f"Published {pose_name} to /target_pose.")

            if pose_name == 'R1':
                # R1: slot 精定位，视觉给的是 ACSs 在 world 下
                self.slot_pose_publisher.publish(pose_msg)
                self.last_slot_pose = pose_msg
                self.get_logger().info(
                    f'Published {pose_name} to /slot_target_pose (ACSs visualization, latched).'
                )

                # 计算 TCP 插入终点位姿
                self.slot_insert_tcp_pose = self.compute_slot_insert_tcp(pose_msg)
                self.slot_insert_tcp_publisher.publish(self.slot_insert_tcp_pose)
                self.get_logger().info(
                    'Computed & published slot_insert_tcp_pose '
                    f'pos=({self.slot_insert_tcp_pose.pose.position.x:.3f}, '
                    f'{self.slot_insert_tcp_pose.pose.position.y:.3f}, '
                    f'{self.slot_insert_tcp_pose.pose.position.z:.3f})'
                )

                # 计算中间旋转姿态点
                self.slot_pre_insert_rotated_tcp_pose = self.compute_slot_pre_insert_rotated_tcp(
                    self.slot_insert_tcp_pose
                )
                self.slot_pre_insert_rotated_tcp_publisher.publish(self.slot_pre_insert_rotated_tcp_pose)
                self.get_logger().info(
                    'Computed & published slot_pre_insert_rotated_tcp_pose '
                    f'pos=({self.slot_pre_insert_rotated_tcp_pose.pose.position.x:.3f}, '
                    f'{self.slot_pre_insert_rotated_tcp_pose.pose.position.y:.3f}, '
                    f'{self.slot_pre_insert_rotated_tcp_pose.pose.position.z:.3f})'
                )

                # 计算预插入位姿
                self.slot_pre_insert_tcp_pose = self.compute_slot_pre_insert_tcp(
                    self.slot_pre_insert_rotated_tcp_pose
                )
                self.slot_pre_insert_tcp_publisher.publish(self.slot_pre_insert_tcp_pose)
                self.get_logger().info(
                    'Computed & published slot_pre_insert_tcp_pose '
                    f'pos=({self.slot_pre_insert_tcp_pose.pose.position.x:.3f}, '
                    f'{self.slot_pre_insert_tcp_pose.pose.position.y:.3f}, '
                    f'{self.slot_pre_insert_tcp_pose.pose.position.z:.3f})'
                )
            else:
                # 其他 R* 仍然默认映射到 slot 可视化
                self.slot_pose_publisher.publish(pose_msg)
                self.get_logger().warn(
                    f'{pose_name} is not explicitly mapped. Forwarded to /slot_target_pose by default.'
                )

        except Exception as e:
            self.get_logger().error(f"Failed to parse R-message: '{data_str}'. Error: {e}")

    def destroy_node(self):
        try:
            self.server_socket.close()
        except Exception:
            pass
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
