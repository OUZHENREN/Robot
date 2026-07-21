from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_pose_name',
            default_value='box_rough_capture_tcp'
        ),
        DeclareLaunchArgument(
            'end_pose_name',
            default_value='slot_rough_capture_tcp'
        ),
        DeclareLaunchArgument(
            'sampling_segment_count',
            default_value='5'
        ),
        DeclareLaunchArgument(
            'stable_required_count',
            default_value='5'
        ),
        DeclareLaunchArgument(
            'stable_decimal_places',
            default_value='3'
        ),
        DeclareLaunchArgument(
            'pcd_root_directory',
            default_value='/home/yff/environment_point_cloud'
        ),
        DeclareLaunchArgument(
            'pcd_filename',
            default_value='environment_point_cloud.pcd'
        ),
        DeclareLaunchArgument(
            'pcd_wait_timeout_sec',
            default_value='60.0'
        ),
        DeclareLaunchArgument(
            'pcd_poll_interval_ms',
            default_value='500'
        ),
        DeclareLaunchArgument(
            'trigger_ip',
            default_value='192.168.1.100'
        ),
        DeclareLaunchArgument(
            'trigger_port',
            default_value='7000'
        ),
        DeclareLaunchArgument(
            'trigger_header',
            default_value='E1'
        ),
        DeclareLaunchArgument(
            'trigger_base_frame',
            default_value='base_link'
        ),
        DeclareLaunchArgument(
            'trigger_tool_frame',
            default_value='flange'
        ),
        DeclareLaunchArgument(
            'post_trigger_wait_sec',
            default_value='2.0'
        ),
        DeclareLaunchArgument(
            'position_arrival_tolerance_mm',
            default_value='5.0'
        ),
        DeclareLaunchArgument(
            'startup_delay_sec',
            default_value='2.0'
        ),

        Node(
            package='workspace_pointcloud_sampler',
            executable='workspace_pointcloud_sampler_node',
            name='workspace_pointcloud_sampler',
            output='log',
            parameters=[{
                'start_pose_name': LaunchConfiguration('start_pose_name'),
                'end_pose_name': LaunchConfiguration('end_pose_name'),
                'sampling_segment_count': LaunchConfiguration('sampling_segment_count'),
                'stable_required_count': LaunchConfiguration('stable_required_count'),
                'stable_decimal_places': LaunchConfiguration('stable_decimal_places'),
                'pcd_root_directory': LaunchConfiguration('pcd_root_directory'),
                'pcd_filename': LaunchConfiguration('pcd_filename'),
                'pcd_wait_timeout_sec': LaunchConfiguration('pcd_wait_timeout_sec'),
                'pcd_poll_interval_ms': LaunchConfiguration('pcd_poll_interval_ms'),
                'trigger_ip': LaunchConfiguration('trigger_ip'),
                'trigger_port': LaunchConfiguration('trigger_port'),
                'trigger_header': LaunchConfiguration('trigger_header'),
                'trigger_base_frame': LaunchConfiguration('trigger_base_frame'),
                'trigger_tool_frame': LaunchConfiguration('trigger_tool_frame'),
                'post_trigger_wait_sec': LaunchConfiguration('post_trigger_wait_sec'),
                'position_arrival_tolerance_mm': LaunchConfiguration('position_arrival_tolerance_mm'),
                'startup_delay_sec': LaunchConfiguration('startup_delay_sec'),
            }]
        )
    ])
