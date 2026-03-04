#!/usr/bin/env python3

# 导入所需的库
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

class StaticModelPublisher(Node):
    """
    这个节点订阅一个位姿话题(/target_pose)，
    然后在RViz中发布一个3D模型(Marker)来可视化该位姿。
    """

    def __init__(self):
        """节点构造函数，初始化订阅者和发布者。"""
        super().__init__('static_model_publisher')
        
        # 创建一个订阅者，监听/target_pose话题
        self.subscription = self.create_subscription(
            PoseStamped,
            '/target_pose',       # 订阅的话题名称
            self.pose_callback,   # 收到消息后调用的回调函数
            10)                   # 队列大小
        
        # 创建一个发布者，发布MarkerArray到/visualization_marker_array话题
        self.publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        self.get_logger().info('Static Model Publisher 已启动，正在等待 /target_pose 上的位姿数据...')

    def pose_callback(self, msg: PoseStamped):
        """
        回调函数，当收到/target_pose上的消息时被调用。
        """
        # 从收到的消息中提取位置和姿态
        position = msg.pose.position
        orientation = msg.pose.orientation
        
        # 调用函数来发布模型
        self.publish_marker(position, orientation)

    def publish_marker(self, position: Point, orientation: Quaternion):
        """
        创建并发布一个代表3D模型的Marker。
        """
        marker = Marker()
        marker.header.frame_id = "world"  # 设置坐标系
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "static_model_ns"     # 命名空间
        marker.id = 0                     # Marker的唯一ID
        
        # --- 使用GLB模型 ---
        marker.type = Marker.MESH_RESOURCE
        # 注意：这里的路径必须是绝对路径，并且以"file://"开头
        # ↓↓↓↓↓↓ 这里已更新为您提供的正确路径 ↓↓↓↓↓↓
        marker.mesh_resource = "file:///home/yff/elite_ros_ws/src/tcp_bridge/meshes/slot_pointcloud.glb"
        # ↑↑↑↑↑↑ 这里已更新为您提供的正确路径 ↑↑↑↑↑↑
        marker.mesh_use_embedded_materials = True  # 使用模型自带的材质和颜色
        
        marker.action = Marker.ADD  # 表示添加或修改这个Marker
        
        # 设置Marker的位姿
        marker.pose.position = position
        marker.pose.orientation = orientation
        
        # 设置模型的缩放比例
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # 当使用mesh_use_embedded_materials=True时，颜色设置会被忽略
        marker.color.a = 0.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # RViz需要接收一个MarkerArray消息，所以我们将单个Marker放入数组中
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        
        # 发布MarkerArray
        self.publisher_.publish(marker_array)

def main(args=None):
    """
    ROS2节点的主入口函数。
    """
    rclpy.init(args=args)
    
    # 创建并实例化我们的节点
    static_model_publisher = StaticModelPublisher()
    
    try:
        # 让节点持续运行，处理回调函数
        rclpy.spin(static_model_publisher)
    except KeyboardInterrupt:
        # 如果用户按下Ctrl+C，则优雅地退出
        static_model_publisher.get_logger().info('用户中断，节点关闭中...')
    finally:
        # 销毁节点并关闭rclpy，释放资源
        static_model_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    # 脚本被直接执行时，调用main函数
    main()

