from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. tcp_server 节点
        Node(
            package='tcp_bridge',
            executable='tcp_server',
            name='tcp_server',
            output='log',
            
        ),

        # 2. robot_commander 节点
        Node(
            package='tcp_bridge',
            executable='robot_commander',
            name='robot_commander',
            output='log',
            
        ),

        # 3. pose_sender_node 节点（监听 /clicked_point，发送消息）
        Node(
            package='tcp_bridge',
            executable='pose_sender_node',
            name='pose_sender_node',
            output='log',
            
        ),

        # 4. slot TF 广播器
        Node(
            package='tcp_bridge',
            executable='pose_to_tf_broadcaster',
            name='slot_tf_broadcaster',
            output='log',
            
            parameters=[{
                'pose_topic': '/slot_target_pose',
                'parent_frame': 'world',
                'child_frame': 'slot_origin',
            }],
        ),

        # 4.1. slot_precision_view_tcp TF 广播器
        Node(
            package='tcp_bridge',
            executable='pose_to_tf_broadcaster',
            name='slot_precision_view_tf_broadcaster',
            output='log',
            
            parameters=[{
                'pose_topic': '/slot_precision_view_tcp_pose',
                'parent_frame': 'world',
                'child_frame': 'slot_precision_view_tcp',
            }],
        ),

        # 5. box TF 广播器
        Node(
            package='tcp_bridge',
            executable='pose_to_tf_broadcaster',
            name='box_tf_broadcaster',
            output='log',
            
            parameters=[{
                'pose_topic': '/box_target_pose',
                'parent_frame': 'world',
                'child_frame': 'box_origin',
            }],
        ),

        # 5.1. box_precision_view_tcp TF 广播器
        Node(
            package='tcp_bridge',
            executable='pose_to_tf_broadcaster',
            name='box_precision_view_tf_broadcaster',
            output='log',
            
            parameters=[{
                'pose_topic': '/box_precision_view_tcp_pose',
                'parent_frame': 'world',
                'child_frame': 'box_precision_view_tcp',
            }],
        ),

        # 6. box_grasp_tcp TF 广播器
        Node(
            package='tcp_bridge',
            executable='pose_to_tf_broadcaster',
            name='box_grasp_tf_broadcaster',
            output='log',
            
            parameters=[{
                'pose_topic': '/box_grasp_tcp_pose',
                'parent_frame': 'world',
                'child_frame': 'box_grasp_tcp',
            }],
        ),

        # 7. box_pre_grasp_tcp TF 广播器
        Node(
            package='tcp_bridge',
            executable='pose_to_tf_broadcaster',
            name='box_pre_grasp_tf_broadcaster',
            output='log',
            
            parameters=[{
                'pose_topic': '/box_pre_grasp_tcp_pose',
                'parent_frame': 'world',
                'child_frame': 'box_pre_grasp_tcp',
            }],
        ),

        # 8. slot_insert_tcp TF 广播器
        Node(
            package='tcp_bridge',
            executable='pose_to_tf_broadcaster',
            name='slot_insert_tf_broadcaster',
            output='log',
            
            parameters=[{
                'pose_topic': '/slot_insert_tcp_pose',
                'parent_frame': 'world',
                'child_frame': 'slot_insert_tcp',
            }],
        ),

        # 9. slot_pre_insert_rotated_tcp TF 广播器
        Node(
            package='tcp_bridge',
            executable='pose_to_tf_broadcaster',
            name='slot_pre_insert_rotated_tf_broadcaster',
            output='log',
            
            parameters=[{
                'pose_topic': '/slot_pre_insert_rotated_tcp_pose',
                'parent_frame': 'world',
                'child_frame': 'slot_pre_insert_rotated_tcp',
            }],
        ),

        # 10. slot_pre_insert_tcp TF 广播器
        Node(
            package='tcp_bridge',
            executable='pose_to_tf_broadcaster',
            name='slot_pre_insert_tf_broadcaster',
            output='log',
            
            parameters=[{
                'pose_topic': '/slot_pre_insert_tcp_pose',
                'parent_frame': 'world',
                'child_frame': 'slot_pre_insert_tcp',
            }],
        ),

        # 11. slot world visual 发布器
        Node(
            package='tcp_bridge',
            executable='static_model_publisher',
            name='slot_model_publisher',
            output='log',
            
            parameters=[{
                'pose_topic': '/slot_target_pose',
                'mesh_resource': 'file:///home/yff/elite_ros_ws/src/tcp_bridge/meshes/slot_pointcloud.glb',
                'frame_id': 'world',
                'marker_ns': 'slot_model_ns',
                'marker_id': 0,
                'scale.x': 1.0,
                'scale.y': 1.0,
                'scale.z': 1.0,
                'use_embedded_materials': True,
                'enable_grasp_event_control': False,
                'restore_on_release_confirmed': False,
            }],
        ),

        # 12. box world visual 发布器
        Node(
            package='tcp_bridge',
            executable='static_model_publisher',
            name='box_model_publisher',
            output='log',
            
            parameters=[{
                'pose_topic': '/box_target_pose',
                'mesh_resource': 'file:///home/yff/elite_ros_ws/src/tcp_bridge/meshes/box.glb',
                'frame_id': 'world',
                'marker_ns': 'box_model_ns',
                'marker_id': 1,
                'scale.x': 1.0,
                'scale.y': 1.0,
                'scale.z': 1.0,
                'use_embedded_materials': True,
                'enable_grasp_event_control': True,
                'grasp_event_topic': '/cs625/grasp_state_event',
                'hide_on_grasp_confirmed': True,
                'restore_on_release_confirmed': True,
                'slot_pose_topic': '/slot_target_pose',
            }],
        ),

        # 12.1. attached box visual 发布器
        Node(
            package='tcp_bridge',
            executable='attached_box_visual_publisher',
            name='attached_box_visual_publisher',
            output='log',
            
            parameters=[{
                'grasp_event_topic': '/cs625/grasp_state_event',
                'marker_topic': '/visualization_marker',
                'attached_link': 'my_end_effector_link',
                'mesh_resource': 'file:///home/yff/elite_ros_ws/src/tcp_bridge/meshes/box.glb',
                'marker_ns': 'attached_box_model_ns',
                'marker_id': 101,
                'scale.x': 1.0,
                'scale.y': 1.0,
                'scale.z': 1.0,
                'use_embedded_materials': True,
                'delete_world_marker_on_grasp': True,
                'world_marker_ns': 'box_model_ns',
                'world_marker_id': 1,
            }],
        ),

        # 13. MoveIt collision 发布器
        Node(
            package='tcp_bridge',
            executable='collision_object_publisher',
            name='collision_object_publisher',
            output='log',
            
            parameters=[{
                'world_frame': 'world',
                'attached_link': 'my_end_effector_link',

                'grasp_event_topic': '/cs625/grasp_state_event',
                'slot_pose_topic': '/slot_target_pose',
                'box_pose_topic': '/box_target_pose',

                'slot_mesh_path': '/home/yff/elite_ros_ws/src/tcp_bridge/meshes/slot_collision.STL',
                'box_mesh_path': '/home/yff/elite_ros_ws/src/tcp_bridge/meshes/box_collision.STL',

                'slot_mesh_scale.x': 0.001,
                'slot_mesh_scale.y': 0.001,
                'slot_mesh_scale.z': 0.001,

                'box_mesh_scale.x': 0.001,
                'box_mesh_scale.y': 0.001,
                'box_mesh_scale.z': 0.001,

                'slot_offset.x': 0.0,
                'slot_offset.y': 0.0,
                'slot_offset.z': 0.0,

                'box_offset.x': 0.0,
                'box_offset.y': 0.0,
                'box_offset.z': 0.0,
            }],
        ),
    ])
