from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='environment_collision_generator',
            executable='environment_collision_generator_node',
            name='environment_collision_generator',
            output='log',
            parameters=[{
                'input_cloud_topic': '/environment/point_cloud',
                'slot_pose_topic': '/slot_target_pose',
                'box_pose_topic': '/box_target_pose',
                'planning_scene_topic': '/planning_scene',
                'world_frame': 'world',

                'slot_mesh_path': '/home/yff/elite_ros_ws/src/tcp_bridge/meshes/slot_collision.STL',
                'box_mesh_path': '/home/yff/elite_ros_ws/src/tcp_bridge/meshes/box_collision.STL',

                'slot_mesh_scale.x': 0.001,
                'slot_mesh_scale.y': 0.001,
                'slot_mesh_scale.z': 0.001,
                'box_mesh_scale.x': 0.001,
                'box_mesh_scale.y': 0.001,
                'box_mesh_scale.z': 0.001,

                'enable_slot_subtraction': True,
                'enable_box_subtraction': True,
                'enable_tf_fallback': True,

                'slot_tf_frame': 'slot_origin',
                'box_tf_frame': 'box_origin',

                'subtraction_padding_x': 0.015,
                'subtraction_padding_y': 0.015,
                'subtraction_padding_z': 0.015,

                'environment_voxel_size': 0.05,
                'max_environment_boxes': 500,
                'min_points_per_voxel': 1,

                'environment_object_prefix': 'env_voxel_',
                'publish_remaining_cloud': True,
                'publish_removed_cloud': True,
            }],
        )
    ])
