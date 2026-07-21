#!/usr/bin/env python3
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


JOINT_STATE_TOPIC = '/joint_states'
BASE_FRAME = 'base'        # 来自 view_frames/tf2_echo
TCP_FRAME = 'tcp_pose'


class SampleCollectorTF(Node):
    def __init__(self):
        super().__init__('cs625_sample_collector_tf')

        # 最新的 JointState 缓存
        self.joint_state = None
        self.joint_sub = self.create_subscription(
            JointState, JOINT_STATE_TOPIC, self.joint_cb, 10
        )

        # TF buffer + listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'Waiting for joint states on "{JOINT_STATE_TOPIC}" '
            f'and TF transform {BASE_FRAME} -> {TCP_FRAME} ...'
        )

    def joint_cb(self, msg: JointState):
        # 每次收到 JointState 都缓存最新一条
        self.joint_state = msg


def main():
    rclpy.init()
    node = SampleCollectorTF()

    try:
        # 最多等待多少秒让 joint_states 和 TF 都“就绪”
        timeout_sec = 8.0
        start_time = node.get_clock().now()

        have_joint = False
        have_tf = False
        warned_tf = False

        while rclpy.ok():
            # 处理一次回调（JointState + TF）
            rclpy.spin_once(node, timeout_sec=0.1)

            # 1) joint_states 是否已经至少收到一条
            if node.joint_state is not None and not have_joint:
                have_joint = True
                node.get_logger().info('Received first /joint_states message.')

            # 2) TF 是否可以查询 base -> tcp_pose
            if not have_tf:
                try:
                    # Time() 空时间：表示“最新的可用 TF”
                    trans = node.tf_buffer.lookup_transform(
                        BASE_FRAME, TCP_FRAME, Time()
                    )
                    have_tf = True
                    p = trans.transform.translation
                    node.get_logger().info(
                        f'Got TF transform {BASE_FRAME} -> {TCP_FRAME}. '
                        f'translation=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
                    )
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    if not warned_tf:
                        node.get_logger().warn(
                            f'Lookup TF {BASE_FRAME}->{TCP_FRAME} is not ready yet: {e}'
                        )
                        warned_tf = True

            # 两者都 OK 就可以采样并退出等待
            if have_joint and have_tf:
                break

            # 检查是否超时
            now = node.get_clock().now()
            if (now - start_time).nanoseconds * 1e-9 > timeout_sec:
                node.get_logger().error(
                    f'Timeout: joint_state received={have_joint}, TF received={have_tf}.'
                )
                rclpy.shutdown()
                return

        # 走到这里说明 joint 和 TF 都 OK
        js = node.joint_state

        if js is None:
            node.get_logger().error('JointState is None after wait, aborting.')
            rclpy.shutdown()
            return

        if len(js.position) < 6:
            node.get_logger().error(
                f'JointState.position 维度不足 6，当前为 {len(js.position)}'
            )
            rclpy.shutdown()
            return

        # 取前 6 个关节角（单位 rad），转换成 deg
        q_rad = js.position[:6]
        q_deg = [math.degrees(a) for a in q_rad]

        # --------- 这里调整前三个角度的顺序 ----------
        # 原顺序: [q1, q2, q3, q4, q5, q6]
        # 期望顺序: [q3, q2, q1, q4, q5, q6]
        q_deg_reordered = [
            q_deg[2],  # q3
            q_deg[1],  # q2
            q_deg[0],  # q1
            q_deg[3],  # q4
            q_deg[4],  # q5
            q_deg[5],  # q6
        ]
        # ---------------------------------------------

        # 再取一次最新 TF（base -> tcp_pose）
        try:
            trans = node.tf_buffer.lookup_transform(
                BASE_FRAME, TCP_FRAME, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            node.get_logger().error(
                f'Failed to lookup TF {BASE_FRAME}->{TCP_FRAME} for sampling: {e}'
            )
            rclpy.shutdown()
            return

        p = trans.transform.translation
        o = trans.transform.rotation

        tcp_x = p.x
        tcp_y = p.y
        tcp_z = p.z
        qx = o.x
        qy = o.y
        qz = o.z
        qw = o.w

        # 使用调整后的关节顺序
        row = [
            q_deg_reordered[0], q_deg_reordered[1], q_deg_reordered[2],
            q_deg_reordered[3], q_deg_reordered[4], q_deg_reordered[5],
            tcp_x, tcp_y, tcp_z,
            qx, qy, qz, qw,
        ]

        fmt = ' '.join(['{:.6f}'] * 13)
        line = fmt.format(*row)

        node.get_logger().info('Sample: ' + line)

        # 写入 ~/cs625_samples.txt（追加）
        home = os.path.expanduser('~')
        out_path = os.path.join(home, 'cs625_samples.txt')
        try:
            with open(out_path, 'a') as f:
                f.write(line + '\n')
            node.get_logger().info(f'Appended to {out_path}')
        except OSError as e:
            node.get_logger().error(f'Failed to write to {out_path}: {e}')

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
