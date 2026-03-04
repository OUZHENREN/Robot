# 文件路径: ~/elite_ros_ws/src/tcp_bridge/tcp_bridge/pose_to_tf_broadcaster.py
# 最终修正版: 删除了无用的 tf_transformations 依赖

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
# FIX: 删除了 "import tf_transformations" 这一行

class PoseToTFBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_to_tf_broadcaster')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.pose_callback,
            10)
            
        self.get_logger().info('PoseToTFBroadcaster (v2 - Final) is running and waiting for /target_pose messages.')

    def pose_callback(self, msg: PoseStamped):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'slot_origin'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info(f'Published TF transform for slot_origin at x={msg.pose.position.x:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

