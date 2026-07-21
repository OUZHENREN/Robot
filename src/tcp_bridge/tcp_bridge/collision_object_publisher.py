#!/usr/bin/env python3

import os
import copy
import traceback

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped, Point, Pose
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from cs625_task_manager.msg import TaskState

import trimesh
import numpy as np
from scipy.spatial.transform import Rotation as R


class CollisionObjectPublisher(Node):
    """
    MoveIt collision 发布节点。

    设计职责：
    - 只负责 collision，不负责 visual
    - 管理 world collision:
      - slot_collision
      - box_collision
    - 管理 attached collision:
      - box_collision attached to my_end_effector_link

    输入：
    - /slot_target_pose
    - /box_target_pose
    - /cs625/grasp_state_event
    - /task_state
    - TF: world_frame/base_frame -> box_place_tcp

    输出：
    - 通过 /apply_planning_scene service 更新 MoveIt scene

    事件语义：
    - grasp_confirmed:
        1) REMOVE world 中的 box_collision
        2) ADD attached box_collision
        3) 禁止后续 /box_target_pose 自动恢复或覆盖 world box_collision

    - release_confirmed:
        1) FORCE REMOVE attached box_collision
        2) FORCE REMOVE world 中的 box_collision
        3) 若当前 step 为 D8，则优先按 box_place_tcp 计算最终放置位姿
        4) 否则按 slot pose 计算最终放置位姿
        5) ADD world 中的 box_collision 到最终位置
        6) 进入 placed 模式，忽略后续 /box_target_pose 覆盖
        7) 若缺少所需 pose，则仅清掉 attached/world，并重新允许 /box_target_pose 更新 world box

    - reset:
        1) FORCE REMOVE attached box_collision
        2) FORCE REMOVE world 中的 box_collision
        3) 退出 placed 模式
        4) 重新允许后续 /box_target_pose 更新 world box_collision
        5) 不立即恢复，等待下一次 /box_target_pose
    """

    STEP_E8_RELEASE = 9
    STEP_D8_RELEASE = 109

    def __init__(self):
        super().__init__('collision_object_publisher')

        # ----------------------------
        # 参数
        # ----------------------------
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('attached_link', 'my_end_effector_link')
        self.declare_parameter('box_place_frame', 'box_place_tcp')
        self.declare_parameter('task_state_topic', '/task_state')

        self.declare_parameter(
            'slot_mesh_path',
            '/home/yff/elite_ros_ws/src/tcp_bridge/meshes/slot_collision.STL'
        )
        self.declare_parameter(
            'box_mesh_path',
            '/home/yff/elite_ros_ws/src/tcp_bridge/meshes/box_collision.STL'
        )

        self.declare_parameter('slot_mesh_scale.x', 0.001)
        self.declare_parameter('slot_mesh_scale.y', 0.001)
        self.declare_parameter('slot_mesh_scale.z', 0.001)

        self.declare_parameter('box_mesh_scale.x', 0.001)
        self.declare_parameter('box_mesh_scale.y', 0.001)
        self.declare_parameter('box_mesh_scale.z', 0.001)

        self.declare_parameter('slot_offset.x', 0.0)
        self.declare_parameter('slot_offset.y', 0.0)
        self.declare_parameter('slot_offset.z', 0.0)

        self.declare_parameter('box_offset.x', 0.0)
        self.declare_parameter('box_offset.y', 0.0)
        self.declare_parameter('box_offset.z', 0.0)

        self.declare_parameter('grasp_event_topic', '/cs625/grasp_state_event')
        self.declare_parameter('slot_pose_topic', '/slot_target_pose')
        self.declare_parameter('box_pose_topic', '/box_target_pose')

        self.world_frame = self.get_parameter('world_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.attached_link = self.get_parameter('attached_link').value
        self.box_place_frame = self.get_parameter('box_place_frame').value
        self.task_state_topic = self.get_parameter('task_state_topic').value

        self.slot_mesh_path = self.get_parameter('slot_mesh_path').value
        self.box_mesh_path = self.get_parameter('box_mesh_path').value

        self.slot_scale = [
            self.get_parameter('slot_mesh_scale.x').value,
            self.get_parameter('slot_mesh_scale.y').value,
            self.get_parameter('slot_mesh_scale.z').value,
        ]
        self.box_scale = [
            self.get_parameter('box_mesh_scale.x').value,
            self.get_parameter('box_mesh_scale.y').value,
            self.get_parameter('box_mesh_scale.z').value,
        ]

        self.slot_offset = [
            self.get_parameter('slot_offset.x').value,
            self.get_parameter('slot_offset.y').value,
            self.get_parameter('slot_offset.z').value,
        ]
        self.box_offset = [
            self.get_parameter('box_offset.x').value,
            self.get_parameter('box_offset.y').value,
            self.get_parameter('box_offset.z').value,
        ]

        self.grasp_event_topic = self.get_parameter('grasp_event_topic').value
        self.slot_pose_topic = self.get_parameter('slot_pose_topic').value
        self.box_pose_topic = self.get_parameter('box_pose_topic').value

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

        # 状态
        self.box_world_enabled = True
        self.box_attached = False
        self.box_placed_in_slot = False

        self.last_box_pose_msg = None
        self.last_slot_pose_msg = None
        self.current_step = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 异步请求调试
        self._scene_request_seq = 0
        self._pending_scene_futures = {}

        # grasp 两步式 attach 调度
        self._grasp_attach_delay_sec = 0.15
        self._pending_grasp_attach_timer = None

        self.get_logger().info('[CollisionObjectPublisher] loading mesh resources...')
        self.slot_mesh_msg = self.load_mesh_as_shape_msg(
            self.slot_mesh_path,
            self.slot_scale
        )
        self.box_mesh_msg = self.load_mesh_as_shape_msg(
            self.box_mesh_path,
            self.box_scale
        )

        self.slot_sub = self.create_subscription(
            PoseStamped,
            self.slot_pose_topic,
            self.slot_callback,
            10
        )

        self.box_sub = self.create_subscription(
            PoseStamped,
            self.box_pose_topic,
            self.box_callback,
            10
        )

        self.grasp_state_event_sub = self.create_subscription(
            String,
            self.grasp_event_topic,
            self.grasp_state_event_callback,
            10
        )

        self.task_state_sub = self.create_subscription(
            TaskState,
            self.task_state_topic,
            self.task_state_callback,
            10
        )

        self.apply_planning_scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )

        self.get_logger().info('[CollisionObjectPublisher] started')
        self.get_logger().info(
            f'[CollisionObjectPublisher] world_frame={self.world_frame}, '
            f'base_frame={self.base_frame}, '
            f'attached_link={self.attached_link}, '
            f'box_place_frame={self.box_place_frame}'
        )
        self.get_logger().info(
            f'[CollisionObjectPublisher] slot_pose_topic={self.slot_pose_topic}, '
            f'box_pose_topic={self.box_pose_topic}, grasp_event_topic={self.grasp_event_topic}, '
            f'task_state_topic={self.task_state_topic}'
        )
        self.get_logger().info(
            f'[CollisionObjectPublisher] slot_mesh_path={self.slot_mesh_path}, '
            f'scale={self.slot_scale}, offset={self.slot_offset}'
        )
        self.get_logger().info(
            f'[CollisionObjectPublisher] box_mesh_path={self.box_mesh_path}, '
            f'scale={self.box_scale}, offset={self.box_offset}'
        )
        self.get_logger().info(
            f'[CollisionObjectPublisher] grasp attach delay={self._grasp_attach_delay_sec:.3f}s'
        )
        self.log_state('initial_state')

        self.get_logger().info('[CollisionObjectPublisher] waiting for /apply_planning_scene ...')
        if not self.apply_planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '[CollisionObjectPublisher] /apply_planning_scene service not available. '
                'Scene updates will fail until move_group is ready.'
            )
        else:
            self.get_logger().info(
                '[CollisionObjectPublisher] connected to /apply_planning_scene'
            )

    # ---------------------------------------------------------------------
    # Debug helpers
    # ---------------------------------------------------------------------

    def log_state(self, tag: str):
        self.get_logger().info(
            f'[CollisionObjectPublisher][STATE][{tag}] '
            f'box_world_enabled={self.box_world_enabled}, '
            f'box_attached={self.box_attached}, '
            f'box_placed_in_slot={self.box_placed_in_slot}, '
            f'current_step={self.current_step}, '
            f'has_last_box_pose={self.last_box_pose_msg is not None}, '
            f'has_last_slot_pose={self.last_slot_pose_msg is not None}, '
            f'has_pending_grasp_attach_timer={self._pending_grasp_attach_timer is not None}'
        )

    def pose_to_str(self, pose: Pose) -> str:
        return (
            f'pos=({pose.position.x:.6f}, {pose.position.y:.6f}, {pose.position.z:.6f}), '
            f'quat=({pose.orientation.x:.6f}, {pose.orientation.y:.6f}, '
            f'{pose.orientation.z:.6f}, {pose.orientation.w:.6f})'
        )

    def pose_stamped_to_str(self, pose_msg: PoseStamped) -> str:
        return (
            f'frame={pose_msg.header.frame_id}, '
            f'pos=({pose_msg.pose.position.x:.6f}, {pose_msg.pose.position.y:.6f}, {pose_msg.pose.position.z:.6f}), '
            f'quat=({pose_msg.pose.orientation.x:.6f}, {pose_msg.pose.orientation.y:.6f}, '
            f'{pose_msg.pose.orientation.z:.6f}, {pose_msg.pose.orientation.w:.6f})'
        )

    def collision_operation_to_str(self, op: int) -> str:
        if op == CollisionObject.ADD:
            return 'ADD'
        if op == CollisionObject.REMOVE:
            return 'REMOVE'
        if op == CollisionObject.APPEND:
            return 'APPEND'
        if op == CollisionObject.MOVE:
            return 'MOVE'
        return f'UNKNOWN({op})'

    def summarize_collision_object(self, obj: CollisionObject) -> str:
        pose_desc = 'no_pose'
        if len(obj.mesh_poses) > 0:
            pose_desc = self.pose_to_str(obj.mesh_poses[0])
        return (
            f'id={obj.id}, frame_id={obj.header.frame_id}, '
            f'op={self.collision_operation_to_str(obj.operation)}, '
            f'meshes={len(obj.meshes)}, mesh_poses={len(obj.mesh_poses)}, {pose_desc}'
        )

    def summarize_attached_object(self, attached: AttachedCollisionObject) -> str:
        obj = attached.object
        pose_desc = 'no_pose'
        if len(obj.mesh_poses) > 0:
            pose_desc = self.pose_to_str(obj.mesh_poses[0])
        return (
            f'link_name={attached.link_name}, '
            f'object_id={obj.id}, object_frame={obj.header.frame_id}, '
            f'op={self.collision_operation_to_str(obj.operation)}, '
            f'touch_links={list(attached.touch_links)}, '
            f'meshes={len(obj.meshes)}, mesh_poses={len(obj.mesh_poses)}, {pose_desc}'
        )

    def log_scene_diff_summary(self, world_objects=None, attached_objects=None, log_prefix: str = ''):
        world_count = 0 if world_objects is None else len(world_objects)
        attached_count = 0 if attached_objects is None else len(attached_objects)

        self.get_logger().info(
            f'{log_prefix} scene diff summary: '
            f'world_objects={world_count}, attached_objects={attached_count}'
        )

        if world_objects is not None:
            for i, obj in enumerate(world_objects):
                self.get_logger().info(
                    f'{log_prefix} world[{i}] -> {self.summarize_collision_object(obj)}'
                )

        if attached_objects is not None:
            for i, attached in enumerate(attached_objects):
                self.get_logger().info(
                    f'{log_prefix} attached[{i}] -> {self.summarize_attached_object(attached)}'
                )

    # ---------------------------------------------------------------------
    # Pending timer helpers
    # ---------------------------------------------------------------------

    def cancel_pending_grasp_attach_timer(self, reason: str = ''):
        if self._pending_grasp_attach_timer is not None:
            self.get_logger().info(
                f'[CollisionObjectPublisher] cancel pending grasp attach timer, reason={reason}'
            )
            try:
                self._pending_grasp_attach_timer.cancel()
            except Exception as e:
                self.get_logger().warn(
                    f'[CollisionObjectPublisher] exception while canceling pending grasp attach timer: {repr(e)}'
                )
            try:
                self.destroy_timer(self._pending_grasp_attach_timer)
            except Exception as e:
                self.get_logger().warn(
                    f'[CollisionObjectPublisher] exception while destroying pending grasp attach timer: {repr(e)}'
                )
            self._pending_grasp_attach_timer = None

    def schedule_grasp_attach_step(self):
        self.cancel_pending_grasp_attach_timer(reason='reschedule_grasp_attach_step')

        self.get_logger().info(
            f'[CollisionObjectPublisher] scheduling delayed attach step after '
            f'{self._grasp_attach_delay_sec:.3f}s'
        )

        def _timer_callback():
            try:
                self.get_logger().info(
                    '[CollisionObjectPublisher] delayed grasp attach timer fired'
                )

                timer = self._pending_grasp_attach_timer
                self._pending_grasp_attach_timer = None

                attached_box = self.make_attached_box_object()
                self.get_logger().info(
                    '[CollisionObjectPublisher] grasp_confirmed(step2 attach) -> prepared attached box object: '
                    f'{self.summarize_attached_object(attached_box)}'
                )

                ok = self.apply_scene_diff(
                    world_objects=None,
                    attached_objects=[attached_box],
                    log_prefix='[CollisionObjectPublisher] grasp_confirmed(step2 attach) ->'
                )

                if not ok:
                    self.get_logger().error(
                        '[CollisionObjectPublisher] grasp_confirmed(step2 attach) -> failed to dispatch planning scene request'
                    )
                    self.log_state('after_failed_grasp_confirmed_step2_attach')
                    return

                self.get_logger().info(
                    f'[CollisionObjectPublisher] grasp_confirmed(step2 attach) -> dispatched attach request on {self.attached_link}'
                )
                self.log_state('after_grasp_confirmed_step2_attach')

                if timer is not None:
                    try:
                        self.destroy_timer(timer)
                    except Exception as e:
                        self.get_logger().warn(
                            f'[CollisionObjectPublisher] exception while destroying fired grasp attach timer: {repr(e)}'
                        )

            except Exception as e:
                self.get_logger().error(
                    f'[CollisionObjectPublisher] exception in delayed grasp attach timer: {repr(e)}'
                )
                self.get_logger().error(traceback.format_exc())

        self._pending_grasp_attach_timer = self.create_timer(
            self._grasp_attach_delay_sec,
            _timer_callback
        )

    # ---------------------------------------------------------------------
    # Mesh loading
    # ---------------------------------------------------------------------

    def load_mesh_as_shape_msg(self, mesh_path: str, scale_xyz):
        if not os.path.exists(mesh_path):
            raise FileNotFoundError(f'Mesh file not found: {mesh_path}')

        self.get_logger().info(f'[CollisionObjectPublisher] Loading mesh from: {mesh_path}')
        tm = trimesh.load(mesh_path, force='mesh')

        if tm.is_empty:
            raise RuntimeError(f'Loaded mesh is empty: {mesh_path}')

        tm.vertices[:, 0] *= scale_xyz[0]
        tm.vertices[:, 1] *= scale_xyz[1]
        tm.vertices[:, 2] *= scale_xyz[2]

        mesh_msg = Mesh()

        for v in tm.vertices:
            p = Point()
            p.x = float(v[0])
            p.y = float(v[1])
            p.z = float(v[2])
            mesh_msg.vertices.append(p)

        for f in tm.faces:
            tri = MeshTriangle()
            tri.vertex_indices = [int(f[0]), int(f[1]), int(f[2])]
            mesh_msg.triangles.append(tri)

        self.get_logger().info(
            f'[CollisionObjectPublisher] Loaded mesh "{mesh_path}" with '
            f'{len(mesh_msg.vertices)} vertices and {len(mesh_msg.triangles)} triangles.'
        )

        return mesh_msg

    # ---------------------------------------------------------------------
    # Object construction
    # ---------------------------------------------------------------------

    def make_mesh_collision_object(self, object_id: str, pose_msg: PoseStamped, mesh_msg: Mesh, offset_xyz):
        obj = CollisionObject()
        obj.id = object_id
        obj.header.frame_id = self.world_frame
        obj.header.stamp = self.get_clock().now().to_msg()

        pose = copy.deepcopy(pose_msg.pose)
        pose.position.x += offset_xyz[0]
        pose.position.y += offset_xyz[1]
        pose.position.z += offset_xyz[2]

        obj.meshes.append(mesh_msg)
        obj.mesh_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def make_remove_collision_object(self, object_id: str):
        obj = CollisionObject()
        obj.id = object_id
        obj.header.frame_id = self.world_frame
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.operation = CollisionObject.REMOVE
        return obj

    # ---------------------------------------------------------------------
    # Transform helpers
    # ---------------------------------------------------------------------

    def matrix_to_pose(self, T: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x = float(T[0, 3])
        pose.position.y = float(T[1, 3])
        pose.position.z = float(T[2, 3])

        q = R.from_matrix(T[:3, :3]).as_quat()
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])
        return pose

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
        out.header.frame_id = self.world_frame

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
                f'[CollisionObjectPublisher] TF lookup failed: {self.base_frame} -> {target_frame}: {ex}'
            )
            return None
        except Exception as ex:
            self.get_logger().warn(
                f'[CollisionObjectPublisher] unexpected TF lookup exception: {self.base_frame} -> {target_frame}: {repr(ex)}'
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

    def compute_attached_box_pose(self) -> Pose:
        T_tcp_box = np.linalg.inv(self.T_b_tcp)
        pose = self.matrix_to_pose(T_tcp_box)
        self.get_logger().info(
            f'[CollisionObjectPublisher] computed attached box pose in link "{self.attached_link}": '
            f'{self.pose_to_str(pose)}'
        )
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
        self.get_logger().info(
            '[CollisionObjectPublisher] computing released box pose from cached slot pose: '
            f'{self.pose_stamped_to_str(world_slot_pose)}'
        )

        T_world_slot = self.pose_stamped_to_matrix(world_slot_pose)
        T_world_box = T_world_slot @ self.T_s_tcp @ np.linalg.inv(self.T_b_tcp)
        out = self.matrix_to_pose_stamped(T_world_box)

        self.get_logger().info(
            '[CollisionObjectPublisher] computed released world box pose from slot pose: '
            f'{self.pose_stamped_to_str(out)}'
        )
        return out

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
        self.get_logger().info(
            '[CollisionObjectPublisher] computing released box pose from box_place_tcp pose: '
            f'{self.pose_stamped_to_str(world_place_pose)}'
        )

        T_world_place = self.pose_stamped_to_matrix(world_place_pose)
        T_world_box = T_world_place @ np.linalg.inv(self.T_b_tcp)
        out = self.matrix_to_pose_stamped(T_world_box)

        self.get_logger().info(
            '[CollisionObjectPublisher] computed released world box pose from place pose: '
            f'{self.pose_stamped_to_str(out)}'
        )
        return out

    def make_attached_box_object(self):
        attached = AttachedCollisionObject()
        attached.link_name = self.attached_link

        obj = CollisionObject()
        obj.id = 'box_collision'
        obj.header.frame_id = self.attached_link
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.meshes.append(self.box_mesh_msg)
        obj.mesh_poses.append(self.compute_attached_box_pose())
        obj.operation = CollisionObject.ADD

        attached.object = obj
        attached.touch_links = [self.attached_link]
        return attached

    def make_remove_attached_box_object(self):
        attached = AttachedCollisionObject()
        attached.link_name = self.attached_link

        obj = CollisionObject()
        obj.id = 'box_collision'
        obj.header.frame_id = self.attached_link
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.operation = CollisionObject.REMOVE

        attached.object = obj
        attached.touch_links = [self.attached_link]
        return attached

    # ---------------------------------------------------------------------
    # Planning scene apply (async, non-blocking)
    # ---------------------------------------------------------------------

    def _on_apply_planning_scene_done(self, future, request_id: int, log_prefix: str):
        try:
            self.get_logger().info(
                f'{log_prefix} ApplyPlanningScene done callback triggered, request_id={request_id}'
            )

            exc = future.exception()
            if exc is not None:
                self.get_logger().error(
                    f'{log_prefix} ApplyPlanningScene future exception, request_id={request_id}: {repr(exc)}'
                )
                return

            result = future.result()
            if result is None:
                self.get_logger().error(
                    f'{log_prefix} ApplyPlanningScene returned no result, request_id={request_id}'
                )
                return

            self.get_logger().info(
                f'{log_prefix} ApplyPlanningScene response.success={result.success}, request_id={request_id}'
            )

            if not result.success:
                self.get_logger().error(
                    f'{log_prefix} ApplyPlanningScene failed, request_id={request_id}'
                )
            else:
                self.get_logger().info(
                    f'{log_prefix} ApplyPlanningScene success, request_id={request_id}'
                )

        except Exception as e:
            self.get_logger().error(
                f'{log_prefix} exception in ApplyPlanningScene done callback, '
                f'request_id={request_id}: {repr(e)}'
            )
            self.get_logger().error(traceback.format_exc())
        finally:
            if request_id in self._pending_scene_futures:
                del self._pending_scene_futures[request_id]

    def apply_planning_scene(self, scene: PlanningScene, log_prefix: str = '') -> bool:
        self.get_logger().info(
            f'{log_prefix} apply_planning_scene begin: '
            f'scene.is_diff={scene.is_diff}, robot_state.is_diff={scene.robot_state.is_diff}, '
            f'service_ready={self.apply_planning_scene_client.service_is_ready()}'
        )

        if not self.apply_planning_scene_client.service_is_ready():
            self.get_logger().error(
                f'{log_prefix} /apply_planning_scene service not ready'
            )
            return False

        req = ApplyPlanningScene.Request()
        req.scene = scene

        self._scene_request_seq += 1
        request_id = self._scene_request_seq

        self.get_logger().info(
            f'{log_prefix} sending ApplyPlanningScene async request, request_id={request_id} ...'
        )

        future = self.apply_planning_scene_client.call_async(req)
        self._pending_scene_futures[request_id] = future
        future.add_done_callback(
            lambda fut, rid=request_id, lp=log_prefix: self._on_apply_planning_scene_done(fut, rid, lp)
        )

        self.get_logger().info(
            f'{log_prefix} ApplyPlanningScene request dispatched, request_id={request_id}'
        )

        return True

    def apply_scene_diff(self, world_objects=None, attached_objects=None, log_prefix: str = '') -> bool:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        if world_objects is not None:
            for obj in world_objects:
                scene.world.collision_objects.append(obj)

        if attached_objects is not None:
            for attached in attached_objects:
                scene.robot_state.attached_collision_objects.append(attached)

        self.log_scene_diff_summary(
            world_objects=world_objects,
            attached_objects=attached_objects,
            log_prefix=log_prefix
        )

        return self.apply_planning_scene(scene, log_prefix=log_prefix)

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------

    def task_state_callback(self, msg: TaskState):
        self.current_step = msg.current_step

    def grasp_state_event_callback(self, msg: String):
        try:
            event = msg.data.strip()
            self.get_logger().info(
                f"[CollisionObjectPublisher] received grasp event: '{event}'"
            )
            self.log_state(f'before_event_{event}')

            if event == 'grasp_confirmed':
                self.cancel_pending_grasp_attach_timer(reason='new_grasp_confirmed')

                world_updates = []
                world_updates.append(self.make_remove_collision_object('box_collision'))

                self.get_logger().info(
                    '[CollisionObjectPublisher] grasp_confirmed(step1 remove) -> FORCE REMOVE world box_collision'
                )

                ok = self.apply_scene_diff(
                    world_objects=world_updates,
                    attached_objects=None,
                    log_prefix='[CollisionObjectPublisher] grasp_confirmed(step1 remove) ->'
                )

                if not ok:
                    self.get_logger().error(
                        '[CollisionObjectPublisher] grasp_confirmed(step1 remove) -> failed to dispatch planning scene request'
                    )
                    self.log_state('after_failed_grasp_confirmed_step1_remove')
                    return

                self.box_world_enabled = False
                self.box_attached = True
                self.box_placed_in_slot = False

                self.get_logger().info(
                    '[CollisionObjectPublisher] grasp_confirmed(step1 remove) -> dispatched world remove request'
                )

                self.schedule_grasp_attach_step()
                self.log_state('after_grasp_confirmed_step1_remove')
                return

            if event == 'release_confirmed':
                self.cancel_pending_grasp_attach_timer(reason='release_confirmed')

                world_updates = []
                attached_updates = []

                attached_remove = self.make_remove_attached_box_object()
                self.get_logger().info(
                    f'[CollisionObjectPublisher] release_confirmed -> FORCE REMOVE attached box_collision from {self.attached_link}'
                )
                self.get_logger().info(
                    '[CollisionObjectPublisher] release_confirmed -> prepared attached removal: '
                    f'{self.summarize_attached_object(attached_remove)}'
                )
                attached_updates.append(attached_remove)

                world_remove = self.make_remove_collision_object('box_collision')
                self.get_logger().info(
                    '[CollisionObjectPublisher] release_confirmed -> FORCE REMOVE world box_collision'
                )
                self.get_logger().info(
                    '[CollisionObjectPublisher] release_confirmed -> prepared world removal: '
                    f'{self.summarize_collision_object(world_remove)}'
                )
                world_updates.append(world_remove)

                placed_box_pose = None
                release_mode = 'slot'

                if self.current_step == self.STEP_D8_RELEASE:
                    self.get_logger().info(
                        f'[CollisionObjectPublisher] release_confirmed -> detected STEP_D8_RELEASE ({self.current_step}), '
                        f'will use TF frame "{self.box_place_frame}"'
                    )

                    place_pose = self.lookup_pose_from_tf(self.box_place_frame)
                    if place_pose is None:
                        self.get_logger().warn(
                            f'[CollisionObjectPublisher] release_confirmed -> failed to get {self.box_place_frame} from TF'
                        )
                    else:
                        placed_box_pose = self.compute_released_box_pose_from_place(place_pose)
                        release_mode = 'place'

                if placed_box_pose is None:
                    if self.last_slot_pose_msg is None:
                        self.get_logger().warn(
                            '[CollisionObjectPublisher] release_confirmed -> no cached slot pose and no valid place pose'
                        )

                        ok = self.apply_scene_diff(
                            world_objects=world_updates,
                            attached_objects=attached_updates,
                            log_prefix='[CollisionObjectPublisher] release_confirmed(no usable pose) ->'
                        )

                        if not ok:
                            self.get_logger().error(
                                '[CollisionObjectPublisher] release_confirmed(no usable pose) -> failed to dispatch planning scene request'
                            )
                            self.log_state('after_failed_release_no_usable_pose')
                            return

                        self.box_attached = False
                        self.box_world_enabled = True
                        self.box_placed_in_slot = False

                        self.get_logger().warn(
                            f'[CollisionObjectPublisher] release_confirmed -> cannot compute final world box pose, '
                            f're-enable {self.box_pose_topic} updates.'
                        )
                        self.log_state('after_release_no_usable_pose')
                        return

                    self.get_logger().info(
                        '[CollisionObjectPublisher] release_confirmed -> using cached slot pose: '
                        f'{self.pose_stamped_to_str(self.last_slot_pose_msg)}'
                    )
                    placed_box_pose = self.compute_released_box_pose_from_slot(self.last_slot_pose_msg)
                    release_mode = 'slot'

                placed_box_obj = self.make_mesh_collision_object(
                    'box_collision',
                    placed_box_pose,
                    self.box_mesh_msg,
                    self.box_offset
                )
                self.get_logger().info(
                    '[CollisionObjectPublisher] release_confirmed -> prepared world add object: '
                    f'{self.summarize_collision_object(placed_box_obj)}'
                )
                world_updates.append(placed_box_obj)

                ok = self.apply_scene_diff(
                    world_objects=world_updates,
                    attached_objects=attached_updates,
                    log_prefix='[CollisionObjectPublisher] release_confirmed ->'
                )

                if not ok:
                    self.get_logger().error(
                        '[CollisionObjectPublisher] release_confirmed -> failed to dispatch planning scene request'
                    )
                    self.log_state('after_failed_release_confirmed')
                    return

                self.box_attached = False
                self.box_world_enabled = False
                self.box_placed_in_slot = True

                self.get_logger().info(
                    '[CollisionObjectPublisher] release_confirmed -> dispatched placed world box_collision '
                    f'in mode={release_mode}, '
                    f'pos=({placed_box_pose.pose.position.x:.3f}, '
                    f'{placed_box_pose.pose.position.y:.3f}, '
                    f'{placed_box_pose.pose.position.z:.3f})'
                )
                self.log_state('after_release_confirmed')
                return

            if event == 'reset':
                self.cancel_pending_grasp_attach_timer(reason='reset')

                world_updates = []
                attached_updates = []

                attached_remove = self.make_remove_attached_box_object()
                self.get_logger().info(
                    f'[CollisionObjectPublisher] reset -> FORCE REMOVE attached box_collision from {self.attached_link}'
                )
                self.get_logger().info(
                    '[CollisionObjectPublisher] reset -> prepared attached removal: '
                    f'{self.summarize_attached_object(attached_remove)}'
                )
                attached_updates.append(attached_remove)

                world_remove = self.make_remove_collision_object('box_collision')
                self.get_logger().info(
                    '[CollisionObjectPublisher] reset -> FORCE REMOVE world box_collision'
                )
                self.get_logger().info(
                    '[CollisionObjectPublisher] reset -> prepared world removal: '
                    f'{self.summarize_collision_object(world_remove)}'
                )
                world_updates.append(world_remove)

                ok = self.apply_scene_diff(
                    world_objects=world_updates,
                    attached_objects=attached_updates,
                    log_prefix='[CollisionObjectPublisher] reset ->'
                )

                if not ok:
                    self.get_logger().error(
                        '[CollisionObjectPublisher] reset -> failed to dispatch planning scene request'
                    )
                    self.log_state('after_failed_reset')
                    return

                self.box_attached = False
                self.box_world_enabled = True
                self.box_placed_in_slot = False

                self.get_logger().info(
                    f'[CollisionObjectPublisher] reset -> scene clear request dispatched, '
                    f're-enable world box collision, waiting next {self.box_pose_topic}'
                )
                self.log_state('after_reset')
                return

            self.get_logger().warn(
                f"[CollisionObjectPublisher] unknown event: '{event}'"
            )
            self.log_state(f'after_unknown_event_{event}')

        except Exception as e:
            self.get_logger().error(
                f'[CollisionObjectPublisher] exception in grasp_state_event_callback: {repr(e)}'
            )
            self.get_logger().error(traceback.format_exc())

    def slot_callback(self, msg: PoseStamped):
        try:
            self.last_slot_pose_msg = msg

            self.get_logger().info(
                '[CollisionObjectPublisher] received slot pose: '
                f'{self.pose_stamped_to_str(msg)}'
            )
            self.log_state('before_slot_callback')

            slot_obj = self.make_mesh_collision_object(
                'slot_collision',
                msg,
                self.slot_mesh_msg,
                self.slot_offset
            )

            self.get_logger().info(
                '[CollisionObjectPublisher] slot_callback -> prepared slot object: '
                f'{self.summarize_collision_object(slot_obj)}'
            )

            ok = self.apply_scene_diff(
                world_objects=[slot_obj],
                attached_objects=None,
                log_prefix='[CollisionObjectPublisher] slot_callback ->'
            )

            if not ok:
                self.get_logger().error(
                    '[CollisionObjectPublisher] slot_callback -> failed to dispatch slot_collision request'
                )
                self.log_state('after_failed_slot_callback')
                return

            self.get_logger().info(
                f'[CollisionObjectPublisher] ADD/UPDATE slot_collision request dispatched at '
                f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})'
            )
            self.log_state('after_slot_callback')

        except Exception as e:
            self.get_logger().error(
                f'[CollisionObjectPublisher] exception in slot_callback: {repr(e)}'
            )
            self.get_logger().error(traceback.format_exc())

    def box_callback(self, msg: PoseStamped):
        try:
            self.last_box_pose_msg = msg

            self.get_logger().info(
                '[CollisionObjectPublisher] received box pose: '
                f'{self.pose_stamped_to_str(msg)}'
            )
            self.log_state('before_box_callback')

            if self.box_placed_in_slot:
                self.get_logger().info(
                    f'[CollisionObjectPublisher] ignored {self.box_pose_topic} because box is currently placed after release'
                )
                self.log_state('ignored_box_callback_box_placed_after_release')
                return

            if not self.box_world_enabled:
                self.get_logger().info(
                    f'[CollisionObjectPublisher] ignored {self.box_pose_topic} because world box collision disabled'
                )
                self.log_state('ignored_box_callback_world_disabled')
                return

            box_obj = self.make_mesh_collision_object(
                'box_collision',
                msg,
                self.box_mesh_msg,
                self.box_offset
            )

            self.get_logger().info(
                '[CollisionObjectPublisher] box_callback -> prepared box object: '
                f'{self.summarize_collision_object(box_obj)}'
            )

            ok = self.apply_scene_diff(
                world_objects=[box_obj],
                attached_objects=None,
                log_prefix='[CollisionObjectPublisher] box_callback ->'
            )

            if not ok:
                self.get_logger().error(
                    '[CollisionObjectPublisher] box_callback -> failed to dispatch box_collision request'
                )
                self.log_state('after_failed_box_callback')
                return

            self.get_logger().info(
                f'[CollisionObjectPublisher] ADD/UPDATE box_collision request dispatched at '
                f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})'
            )
            self.log_state('after_box_callback')

        except Exception as e:
            self.get_logger().error(
                f'[CollisionObjectPublisher] exception in box_callback: {repr(e)}'
            )
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CollisionObjectPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f'[CollisionObjectPublisher] fatal exception: {repr(e)}')
            node.get_logger().error(traceback.format_exc())
        else:
            print(f'[CollisionObjectPublisher] fatal exception before node init: {repr(e)}')
            print(traceback.format_exc())
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
