#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R


class AttachedBoxVisualPublisher(Node):
    """
    attached box visual 发布节点。

    设计职责：
    - 只负责抓取后 attached box 的 visual mesh 显示
    - 不负责 world box visual 的主生命周期管理
    - world box visual 由 static_model_publisher.py 的 box 实例负责
    - collision 由 collision_object_publisher.py 负责

    输入：
    - /cs625/grasp_state_event

    输出：
    - Marker -> /visualization_marker

    事件语义：
    - grasp_confirmed:
        1) 发布 attached box visual
        2) 可选发送 world marker DELETE 作为兜底
           注意：该兜底只对同一 Marker topic 上的 world marker 有意义，
           若 world visual 由 MarkerArray 发布，则主删除逻辑仍应由
           static_model_publisher.py 自己完成。
    - release_confirmed:
        删除 attached box visual
    - reset:
        删除 attached box visual
    """

    def __init__(self):
        super().__init__('attached_box_visual_publisher')

        # ----------------------------
        # 参数
        # ----------------------------
        self.declare_parameter('grasp_event_topic', '/cs625/grasp_state_event')
        self.declare_parameter('marker_topic', '/visualization_marker')

        self.declare_parameter('attached_link', 'my_end_effector_link')
        self.declare_parameter(
            'mesh_resource',
            'file:///home/yff/elite_ros_ws/src/tcp_bridge/meshes/box.glb'
        )

        self.declare_parameter('marker_ns', 'attached_box_model_ns')
        self.declare_parameter('marker_id', 101)

        self.declare_parameter('scale.x', 1.0)
        self.declare_parameter('scale.y', 1.0)
        self.declare_parameter('scale.z', 1.0)

        self.declare_parameter('use_embedded_materials', True)

        # 是否在 grasp_confirmed 时执行 world marker DELETE 兜底
        self.declare_parameter('delete_world_marker_on_grasp', True)
        self.declare_parameter('world_marker_ns', 'box_model_ns')
        self.declare_parameter('world_marker_id', 1)

        self.grasp_event_topic = self.get_parameter('grasp_event_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value

        self.attached_link = self.get_parameter('attached_link').value
        self.mesh_resource = self.get_parameter('mesh_resource').value

        self.marker_ns = self.get_parameter('marker_ns').value
        self.marker_id = self.get_parameter('marker_id').value

        self.scale_x = self.get_parameter('scale.x').value
        self.scale_y = self.get_parameter('scale.y').value
        self.scale_z = self.get_parameter('scale.z').value

        self.use_embedded_materials = self.get_parameter('use_embedded_materials').value

        self.delete_world_marker_on_grasp = self.get_parameter('delete_world_marker_on_grasp').value
        self.world_marker_ns = self.get_parameter('world_marker_ns').value
        self.world_marker_id = self.get_parameter('world_marker_id').value

        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.marker_pub = self.create_publisher(Marker, self.marker_topic, qos)
        self.event_sub = self.create_subscription(
            String,
            self.grasp_event_topic,
            self.on_event,
            10
        )

        # 与 collision_object_publisher.py 保持一致
        # box -> tcp
        self.T_b_tcp = np.array([
            [-1.0, 0.0, 0.0, -4.0 / 1000.0],
            [0.0,  1.0, 0.0, -215.0 / 1000.0],
            [0.0,  0.0, -1.0, -411.0 / 1000.0],
            [0.0,  0.0, 0.0, 1.0],
        ])

        self.is_visible = False

        self.get_logger().info(
            '[AttachedBoxVisualPublisher] started: '
            f'event_topic={self.grasp_event_topic}, '
            f'marker_topic={self.marker_topic}, '
            f'attached_link={self.attached_link}, '
            f'mesh={self.mesh_resource}, '
            f'ns={self.marker_ns}, id={self.marker_id}, '
            f'delete_world_marker_on_grasp={self.delete_world_marker_on_grasp}'
        )

    def compute_tcp_to_box(self):
        """
        根据:
            T_world_tcp = T_world_box @ T_b_tcp

        附着时需要:
            T_tcp_box = inverse(T_b_tcp)
        """
        T_tcp_box = np.linalg.inv(self.T_b_tcp)
        t = T_tcp_box[:3, 3]
        q = R.from_matrix(T_tcp_box[:3, :3]).as_quat()
        return t, q

    def make_delete_marker(self, frame_id: str, ns: str, marker_id: int):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def make_attached_marker(self):
        t, q = self.compute_tcp_to_box()

        marker = Marker()
        marker.header.frame_id = self.attached_link
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.marker_ns
        marker.id = self.marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = self.mesh_resource
        marker.mesh_use_embedded_materials = self.use_embedded_materials

        marker.pose.position.x = float(t[0])
        marker.pose.position.y = float(t[1])
        marker.pose.position.z = float(t[2])

        marker.pose.orientation.x = float(q[0])
        marker.pose.orientation.y = float(q[1])
        marker.pose.orientation.z = float(q[2])
        marker.pose.orientation.w = float(q[3])

        marker.scale.x = float(self.scale_x)
        marker.scale.y = float(self.scale_y)
        marker.scale.z = float(self.scale_z)

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        marker.lifetime.sec = 0
        marker.frame_locked = True

        return marker

    def publish_delete_world_marker_fallback(self):
        marker = self.make_delete_marker(
            frame_id='world',
            ns=self.world_marker_ns,
            marker_id=self.world_marker_id
        )
        self.marker_pub.publish(marker)
        self.get_logger().info(
            '[AttachedBoxVisualPublisher] fallback DELETE world marker '
            f'ns={self.world_marker_ns}, id={self.world_marker_id} '
            f'on topic={self.marker_topic}'
        )

    def publish_delete_attached_marker(self):
        marker = self.make_delete_marker(
            frame_id=self.attached_link,
            ns=self.marker_ns,
            marker_id=self.marker_id
        )
        self.marker_pub.publish(marker)
        self.is_visible = False
        self.get_logger().info(
            f'[AttachedBoxVisualPublisher] DELETE attached marker ns={self.marker_ns}, id={self.marker_id}'
        )

    def publish_attached_marker(self):
        # 先删后加，避免残留
        self.publish_delete_attached_marker()

        marker = self.make_attached_marker()
        self.marker_pub.publish(marker)
        self.is_visible = True

        self.get_logger().info(
            f'[AttachedBoxVisualPublisher] ADD attached marker ns={self.marker_ns}, id={self.marker_id}, '
            f'frame={self.attached_link}'
        )

    def on_event(self, msg: String):
        event = msg.data.strip()
        self.get_logger().info(f'[AttachedBoxVisualPublisher] event={event}')

        if event == 'grasp_confirmed':
            if self.delete_world_marker_on_grasp:
                self.publish_delete_world_marker_fallback()

            self.publish_attached_marker()
            return

        if event == 'release_confirmed':
            self.publish_delete_attached_marker()
            return

        if event == 'reset':
            self.publish_delete_attached_marker()
            return

        self.get_logger().warn(
            f"[AttachedBoxVisualPublisher] unknown event: '{event}'"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AttachedBoxVisualPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
