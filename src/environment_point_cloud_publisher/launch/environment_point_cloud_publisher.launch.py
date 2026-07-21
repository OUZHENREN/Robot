from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    directory_path_arg = DeclareLaunchArgument(
        'directory_path',
        default_value='/home/yff/environment_point_cloud',
        description='Root directory containing timestamp-named subdirectories'
    )

    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/environment/point_cloud',
        description='Topic name for published merged environment point cloud'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='Frame ID for published merged environment point cloud'
    )

    target_pcd_filename_arg = DeclareLaunchArgument(
        'target_pcd_filename',
        default_value='environment_point_cloud.pcd',
        description='Fixed PCD filename expected inside each timestamp subdirectory'
    )

    output_directory_path_arg = DeclareLaunchArgument(
        'output_directory_path',
        default_value='/home/yff/environment_point_cloud_fusion',
        description='Local output directory for saving merged point cloud PCD'
    )

    shared_output_directory_path_arg = DeclareLaunchArgument(
        'shared_output_directory_path',
        default_value='/mnt/hgfs/Model/environment_point_cloud_fusion',
        description='Shared directory path for syncing the merged point cloud PCD'
    )

    publish_interval_sec_arg = DeclareLaunchArgument(
        'publish_interval_sec',
        default_value='1.0',
        description='Publish interval in seconds for the merged point cloud'
    )

    rescan_interval_sec_arg = DeclareLaunchArgument(
        'rescan_interval_sec',
        default_value='2.0',
        description='Directory rescan interval in seconds for auto-reload'
    )

    enable_crop_box_arg = DeclareLaunchArgument(
        'enable_crop_box',
        default_value='false',
        description='Enable secondary CropBox filtering after merging'
    )

    crop_min_x_arg = DeclareLaunchArgument('crop_min_x', default_value='-2.0')
    crop_min_y_arg = DeclareLaunchArgument('crop_min_y', default_value='-2.0')
    crop_min_z_arg = DeclareLaunchArgument('crop_min_z', default_value='-2.0')
    crop_max_x_arg = DeclareLaunchArgument('crop_max_x', default_value='2.0')
    crop_max_y_arg = DeclareLaunchArgument('crop_max_y', default_value='2.0')
    crop_max_z_arg = DeclareLaunchArgument('crop_max_z', default_value='2.0')

    enable_voxel_downsample_arg = DeclareLaunchArgument(
        'enable_voxel_downsample',
        default_value='true',
        description='Enable VoxelGrid downsampling after merging'
    )

    voxel_leaf_size_arg = DeclareLaunchArgument(
        'voxel_leaf_size',
        default_value='0.01',
        description='VoxelGrid leaf size in meters'
    )

    enable_statistical_outlier_removal_arg = DeclareLaunchArgument(
        'enable_statistical_outlier_removal',
        default_value='true',
        description='Enable Statistical Outlier Removal'
    )

    sor_mean_k_arg = DeclareLaunchArgument(
        'sor_mean_k',
        default_value='30',
        description='Mean K for Statistical Outlier Removal'
    )

    sor_stddev_mul_thresh_arg = DeclareLaunchArgument(
        'sor_stddev_mul_thresh',
        default_value='1.0',
        description='Stddev multiplier threshold for Statistical Outlier Removal'
    )

    enable_radius_outlier_removal_arg = DeclareLaunchArgument(
        'enable_radius_outlier_removal',
        default_value='true',
        description='Enable Radius Outlier Removal'
    )

    ror_radius_search_arg = DeclareLaunchArgument(
        'ror_radius_search',
        default_value='0.03',
        description='Radius search in meters for Radius Outlier Removal'
    )

    ror_min_neighbors_arg = DeclareLaunchArgument(
        'ror_min_neighbors',
        default_value='4',
        description='Minimum neighbors in radius for Radius Outlier Removal'
    )

    enable_smoothing_arg = DeclareLaunchArgument(
        'enable_smoothing',
        default_value='false',
        description='Reserved smoothing switch, currently only logs a warning'
    )

    use_xterm_arg = DeclareLaunchArgument(
        'use_xterm',
        default_value='true',
        description='Whether to launch the node in a dedicated xterm window'
    )

    common_parameters = [
        {
            'directory_path': LaunchConfiguration('directory_path'),
            'topic_name': LaunchConfiguration('topic_name'),
            'frame_id': LaunchConfiguration('frame_id'),
            'target_pcd_filename': LaunchConfiguration('target_pcd_filename'),
            'output_directory_path': LaunchConfiguration('output_directory_path'),
            'shared_output_directory_path': LaunchConfiguration('shared_output_directory_path'),
            'publish_interval_sec': LaunchConfiguration('publish_interval_sec'),
            'rescan_interval_sec': LaunchConfiguration('rescan_interval_sec'),

            'enable_crop_box': LaunchConfiguration('enable_crop_box'),
            'crop_min_x': LaunchConfiguration('crop_min_x'),
            'crop_min_y': LaunchConfiguration('crop_min_y'),
            'crop_min_z': LaunchConfiguration('crop_min_z'),
            'crop_max_x': LaunchConfiguration('crop_max_x'),
            'crop_max_y': LaunchConfiguration('crop_max_y'),
            'crop_max_z': LaunchConfiguration('crop_max_z'),

            'enable_voxel_downsample': LaunchConfiguration('enable_voxel_downsample'),
            'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),

            'enable_statistical_outlier_removal': LaunchConfiguration('enable_statistical_outlier_removal'),
            'sor_mean_k': LaunchConfiguration('sor_mean_k'),
            'sor_stddev_mul_thresh': LaunchConfiguration('sor_stddev_mul_thresh'),

            'enable_radius_outlier_removal': LaunchConfiguration('enable_radius_outlier_removal'),
            'ror_radius_search': LaunchConfiguration('ror_radius_search'),
            'ror_min_neighbors': LaunchConfiguration('ror_min_neighbors'),

            'enable_smoothing': LaunchConfiguration('enable_smoothing'),
        }
    ]

    environment_point_cloud_node_xterm = Node(
        package='environment_point_cloud_publisher',
        executable='environment_point_cloud_publisher_node',
        name='environment_point_cloud_publisher',
        output='log',
        
        parameters=common_parameters,
        condition=IfCondition(LaunchConfiguration('use_xterm'))
    )

    environment_point_cloud_node_normal = Node(
        package='environment_point_cloud_publisher',
        executable='environment_point_cloud_publisher_node',
        name='environment_point_cloud_publisher',
        output='log',
        parameters=common_parameters,
        condition=UnlessCondition(LaunchConfiguration('use_xterm'))
    )

    return LaunchDescription([
        directory_path_arg,
        topic_name_arg,
        frame_id_arg,
        target_pcd_filename_arg,
        output_directory_path_arg,
        shared_output_directory_path_arg,
        publish_interval_sec_arg,
        rescan_interval_sec_arg,

        enable_crop_box_arg,
        crop_min_x_arg,
        crop_min_y_arg,
        crop_min_z_arg,
        crop_max_x_arg,
        crop_max_y_arg,
        crop_max_z_arg,

        enable_voxel_downsample_arg,
        voxel_leaf_size_arg,

        enable_statistical_outlier_removal_arg,
        sor_mean_k_arg,
        sor_stddev_mul_thresh_arg,

        enable_radius_outlier_removal_arg,
        ror_radius_search_arg,
        ror_min_neighbors_arg,

        enable_smoothing_arg,

        use_xterm_arg,
        environment_point_cloud_node_xterm,
        environment_point_cloud_node_normal,
    ])
