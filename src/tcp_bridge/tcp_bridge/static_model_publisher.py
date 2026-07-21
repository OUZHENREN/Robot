#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from cs625_task_manager.msg import TaskState

import numpy as np
from scipy.spatial.transform import Rotation as R


class StaticModelPublisher(Node):
    """
    world visual 发布节点。

    设计职责：
    - 只负责发布 world 中的静态 visual mesh
    - 支持多实例化，例如：
      1) slot_model_publisher
      2) box_model_publisher

    输入：
    - PoseStamped: 指定物体在 world 下的位姿
    - 可选 grasp state event:
      - grasp_confirmed:
          对 box world visual 执行 DELETE，并禁止后续 pose 自动恢复
      - release_confirmed:
          E8: 恢复到 slot 内最终放置位姿
          D8: 恢复到 box_place_tcp 对应最终放置位姿
      - reset:
          重新允许后续 pose 恢复 world visual，但不立即恢复
    - /task_state
    - TF: base_frame -> box_place_tcp

    输出：
    - MarkerArray -> /visualization_marker_array

    注意：
    - 本节点只负责 world visual
    - attached visual 由 attached_box_visual_publisher.py 负责
    - collision 由 collision_object_publisher.py 负责
    """

    STEP_E8_RELEASE = 9
    STEP_D8_RELEASE = 109

    def __init__(self):
        super().__init__('static_model_publisher')

        # ----------------------------
        # 参数
        # ----------------------------
        self.declare_parameter('pose_topic', '/target_pose')
        self.declare_parameter('mesh_resource', '')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('box_place_frame', 'box_place_tcp')
        self.declare_parameter('task_state_topic', '/task_state')
        self.declare_parameter('marker_ns', 'static_model_ns')
        self.declare_parameter('marker_id', 0)

        self.declare_parameter('scale.x', 1.0)
        self.declare_parameter('scale.y', 1.0)
        self.declare_parameter('scale.z', 1.0)

        self.declare_parameter('use_embedded_materials', True)

        self.declare_parameter('enable_grasp_event_control', False)
        self.declare_parameter('grasp_event_topic', '/cs625/grasp_state_event')
        self.declare_parameter('hide_on_grasp_confirmed', True)

        self.declare_parameter('restore_on_release_confirmed', False)
        self.declare_parameter('slot_pose_topic', '/slot_target_pose')

        self.pose_topic = self.get_parameter('pose_topic').value
        self.mesh_resource = self.get_parameter('mesh_resource').value
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.box_place_frame = self.get_parameter('box_place_frame').value
        self.task_state_topic = self.get_parameter('task_state_topic').value
        self.marker_ns = self.get_parameter('marker_ns').value
        self.marker_id = self.get_parameter('marker_id').value

        self.scale_x = self.get_parameter('scale.x').value
        self.scale_y = self.get_parameter('scale.y').value
        self.scale_z = self.get_parameter('scale.z').value

        self.use_embedded_materials = self.get_parameter('use_embedded_materials').value

        self.enable_grasp_event_control = self.get_parameter('enable_grasp_event_control').value
        self.grasp_event_topic = self.get_parameter('grasp_event_topic').value
        self.hide_on_grasp_confirmed = self.get_parameter('hide_on_grasp_confirmed').value

        self.restore_on_release_confirmed = self.get_parameter('restore_on_release_confirmed').value
        self.slot_pose_topic = self.get_parameter('slot_pose_topic').value

        if not self.mesh_resource:
            self.get_logger().error('参数 mesh_resource 为空，节点无法工作。')
            raise ValueError('mesh_resource is required')

        # 与 tcp_server.py 保持一致
        # box -> tcp
        self.T_b_tcp = np.array([
            [-1.0, 0.0, 0.0, -4.0 / 1000.0],
            [0.0,  1.0, 0.0, -215.0 / 1000.0],
            [0.0,  0.0, -1.0, -411.0 / 1000.0],
            [0.0,  0.0, 0.0, 1.0],
        ])

        # 与 tcp_server.py 保持一致
        # slot -> tcp
        self.T_s_tcp = np.array([
            [-1.0, 0.0, 0.0, -82.0 / 1000.0],
            [0.0,  1.0, 0.0, -193.0 / 1000.0],
            [0.0,  0.0, -1.0, -412.5 / 1000.0],
            [0.0,  0.0, 0.0, 1.0],
        ])

        self.visible_enabled = True
        self.placed_in_slot = False
        self.last_slot_pose_msg = None
        self.last_pose_msg = None
        self.current_step = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10
        )

        self.task_state_sub = self.create_subscription(
            TaskState,
            self.task_state_topic,
            self.task_state_callback,
            10
        )

        self.grasp_state_event_sub = None
        if self.enable_grasp_event_control:
            self.grasp_state_event_sub = self.create_subscription(
                String,
                self.grasp_event_topic,
                self.grasp_state_event_callback,
                10
            )

        self.slot_pose_sub = None
        if self.restore_on_release_confirmed:
            self.slot_pose_sub = self.create_subscription(
                PoseStamped,
                self.slot_pose_topic,
                self.slot_pose_callback,
                10
            )

        self.get_logger().info(
            '[StaticModelPublisher] started: '
            f'pose_topic={self.pose_topic}, '
            f'mesh={self.mesh_resource}, '
            f'frame_id={self.frame_id}, '
            f'base_frame={self.base_frame}, '
            f'box_place_frame={self.box_place_frame}, '
            f'task_state_topic={self.task_state_topic}, '
            f'ns={self.marker_ns}, id={self.marker_id}, '
            f'event_control={self.enable_grasp_event_control}, '
            f'restore_on_release_confirmed={self.restore_on_release_confirmed}'
        )

    def build_marker(self, action: int, pose=None):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.marker_ns
        marker.id = self.marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.action = action
        marker.mesh_resource = self.mesh_resource
        marker.mesh_use_embedded_materials = self.use_embedded_materials

        if pose is not None:
            marker.pose = pose

        marker.scale.x = float(self.scale_x)
        marker.scale.y = float(self.scale_y)
        marker.scale.z = float(self.scale_z)

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        marker.lifetime.sec = 0
        marker.frame_locked = False

        return marker

    def publish_marker(self, marker: Marker):
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.publisher_.publish(marker_array)

    def publish_delete_marker(self):
        marker = self.build_marker(Marker.DELETE)
        self.publish_marker(marker)
        self.get_logger().info(
            f'[StaticModelPublisher] DELETE ns={self.marker_ns}, id={self.marker_id}'
        )

    def publish_add_marker(self, pose):
        marker = self.build_marker(Marker.ADD, pose)
        self.publish_marker(marker)
        self.get_logger().info(
            f'[StaticModelPublisher] ADD ns={self.marker_ns}, id={self.marker_id}, '
            f'pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
        )

    def pose_stamped_to_matrix(self, pose_msg: PoseStamped) -> np.ndarray:
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        r_world = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        T_world = np.eye(4)
        T_world[:3, :3] = r_world
        T_world[0, 3] = p.x
        T_world[1, 3] = p.y
        T_world[2, 3] = p.z

        return T_world

    def matrix_to_pose_stamped(self, T_world: np.ndarray) -> PoseStamped:
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        out.pose.position.x = float(T_world[0, 3])
        out.pose.position.y = float(T_world[1, 3])
        out.pose.position.z = float(T_world[2, 3])

        r_world = T_world[:3, :3]
        q = R.from_matrix(r_world).as_quat()
        out.pose.orientation.x = float(q[0])
        out.pose.orientation.y = float(q[1])
        out.pose.orientation.z = float(q[2])
        out.pose.orientation.w = float(q[3])

        return out

    def lookup_pose_from_tf(self, target_frame: str):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                target_frame,
                Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'[StaticModelPublisher] TF lookup failed: {self.base_frame} -> {target_frame}: {ex}'
            )
            return None
        except Exception as ex:
            self.get_logger().warn(
                f'[StaticModelPublisher] unexpected TF lookup exception: {self.base_frame} -> {target_frame}: {repr(ex)}'
            )
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = tf_msg.transform.translation.x
        pose.pose.position.y = tf_msg.transform.translation.y
        pose.pose.position.z = tf_msg.transform.translation.z
        pose.pose.orientation = tf_msg.transform.rotation
        return pose

    def compute_released_box_pose_from_slot(self, world_slot_pose: PoseStamped) -> PoseStamped:
        """
        E8 release:
            slot_insert_tcp 与 box_grasp_tcp 重合

        已知：
            T_world_tcp = T_world_slot @ T_s_tcp
            T_world_tcp = T_world_box @ T_b_tcp

        则：
            T_world_box = T_world_slot @ T_s_tcp @ inv(T_b_tcp)
        """
        T_world_slot = self.pose_stamped_to_matrix(world_slot_pose)
        T_world_box = T_world_slot @ self.T_s_tcp @ np.linalg.inv(self.T_b_tcp)
        return self.matrix_to_pose_stamped(T_world_box)

    def compute_released_box_pose_from_place(self, world_place_pose: PoseStamped) -> PoseStamped:
        """
        D8 release:
            box_place_tcp 与 box_grasp_tcp 重合

        已知：
            T_world_tcp = T_world_place
            T_world_tcp = T_world_box @ T_b_tcp

        则：
            T_world_box = T_world_place @ inv(T_b_tcp)
        """
        T_world_place = self.pose_stamped_to_matrix(world_place_pose)
        T_world_box = T_world_place @ np.linalg.inv(self.T_b_tcp)
        return self.matrix_to_pose_stamped(T_world_box)

    def task_state_callback(self, msg: TaskState):
        self.current_step = msg.current_step

    def slot_pose_callback(self, msg: PoseStamped):
        self.last_slot_pose_msg = msg

    def grasp_state_event_callback(self, msg: String):
        event = msg.data.strip()

        if event == 'grasp_confirmed':
            if self.hide_on_grasp_confirmed:
                self.visible_enabled = False
                self.placed_in_slot = False
                self.publish_delete_marker()
                self.get_logger().info(
                    f'[StaticModelPublisher] grasp_confirmed -> disable world visual for ns={self.marker_ns}, id={self.marker_id}'
                )
            return

        if event == 'release_confirmed':
            self.visible_enabled = True

            if not self.restore_on_release_confirmed:
                self.placed_in_slot = False
                self.get_logger().info(
                    f'[StaticModelPublisher] release_confirmed -> immediate restore disabled, waiting next pose on {self.pose_topic}'
                )
                return

            placed_box_pose = None
            release_mode = 'slot'

            if self.current_step == self.STEP_D8_RELEASE:
                self.get_logger().info(
                    f'[StaticModelPublisher] release_confirmed -> detected STEP_D8_RELEASE ({self.current_step}), '
                    f'will use TF frame "{self.box_place_frame}"'
                )

                place_pose = self.lookup_pose_from_tf(self.box_place_frame)
                if place_pose is None:
                    self.get_logger().warn(
                        f'[StaticModelPublisher] release_confirmed -> failed to get {self.box_place_frame} from TF'
                    )
                else:
                    placed_box_pose = self.compute_released_box_pose_from_place(place_pose)
                    release_mode = 'place'

            if placed_box_pose is None:
                if self.last_slot_pose_msg is None:
                    self.placed_in_slot = False
                    self.get_logger().warn(
                        f'[StaticModelPublisher] release_confirmed -> no cached slot pose and no valid place pose, '
                        f'cannot restore placed world visual. Waiting next pose on {self.pose_topic}'
                    )
                    return

                placed_box_pose = self.compute_released_box_pose_from_slot(self.last_slot_pose_msg)
                release_mode = 'slot'

            self.publish_add_marker(placed_box_pose.pose)
            self.placed_in_slot = True

            self.get_logger().info(
                f'[StaticModelPublisher] release_confirmed -> restore world visual at {release_mode} placed pose '
                f'for ns={self.marker_ns}, id={self.marker_id}'
            )
            return

        if event == 'reset':
            self.visible_enabled = True
            self.placed_in_slot = False
            self.get_logger().info(
                f'[StaticModelPublisher] reset -> re-enable world visual for ns={self.marker_ns}, id={self.marker_id}, '
                f'waiting next pose on {self.pose_topic}'
            )
            return

    def pose_callback(self, msg: PoseStamped):
        self.last_pose_msg = msg

        if self.placed_in_slot:
            self.get_logger().info(
                f'[StaticModelPublisher] ignored pose update because model is currently placed after release: topic={self.pose_topic}'
            )
            return

        if not self.visible_enabled:
            self.get_logger().info(
                f'[StaticModelPublisher] ignored pose update because visual disabled: topic={self.pose_topic}'
            )
            return

        self.publish_add_marker(msg.pose)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StaticModelPublisher()
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
