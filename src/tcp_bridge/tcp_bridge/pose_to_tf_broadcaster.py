#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseToTFBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_to_tf_broadcaster')

        # ----------------------------
        # 声明参数
        # ----------------------------
        self.declare_parameter('pose_topic', '/target_pose')
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('child_frame', 'slot_origin')

        # ----------------------------
        # 读取参数
        # ----------------------------
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        self.get_logger().info(
            f'PoseToTFBroadcaster 已启动: '
            f'pose_topic={self.pose_topic}, '
            f'{self.parent_frame} -> {self.child_frame}'
        )

    def pose_callback(self, msg: PoseStamped):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(
            f'Published TF: {self.parent_frame} -> {self.child_frame}, '
            f'x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PoseToTFBroadcaster()
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
