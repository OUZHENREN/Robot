import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # 解析 LaunchConfiguration
    name = LaunchConfiguration("name").perform(context)
    cs_type = LaunchConfiguration("cs_type").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands").perform(context)
    ros2_controllers_file = LaunchConfiguration("ros2_controllers_file").perform(context)

    # 保留原有描述包路径解析逻辑（当前文件中未直接使用，但先不删，避免引入额外变化）
    cs_description_share = get_package_share_directory("eli_cs_robot_description")
    cs_xacro_path = os.path.join(cs_description_share, "urdf", "cs.urdf.xacro")

    # 构造 MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=name,
            package_name="elite_cs625_moveit_config",
        )
        .robot_description(
            mappings={
                "name": name,
                "tf_prefix": tf_prefix,
                "cs_type": cs_type,
                "use_fake_hardware": use_fake_hardware,
                "fake_sensor_commands": fake_sensor_commands,
            }
        )
        .robot_description_semantic()
        .to_moveit_configs()
    )

    # 计算 config 目录绝对路径
    package_share = os.path.join(
        os.getenv("COLCON_CURRENT_PREFIX", os.path.join(os.getcwd(), "install")),
        "elite_cs625_moveit_config",
        "share",
        "elite_cs625_moveit_config",
        "config",
    )

    # controllers 配置文件路径
    controllers_yaml_path = os.path.join(package_share, ros2_controllers_file)

    # move_group 节点
    # 这里采用“内联参数”方式配置 3D 感知/Octomap，避免 ROS2 参数文件对复杂列表字典结构解析失败
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        
        parameters=[
            moveit_config.to_dict(),
            controllers_yaml_path,
            {
                # ===== Octomap 基础参数 =====
                # 先使用 base_link 作为 octomap 坐标系，减少 TF 变量，便于先打通整条链路
                "octomap_frame": "base_link",
                "octomap_resolution": 0.02,

                # ===== 3D 传感器配置 =====
                # 使用一个命名传感器 point_cloud_sensor
                "sensors": ["point_cloud_sensor"],

                # PointCloudOctomapUpdater 插件
                "point_cloud_sensor.sensor_plugin": "occupancy_map_monitor/PointCloudOctomapUpdater",

                # 环境点云话题
                "point_cloud_sensor.point_cloud_topic": "/environment/point_cloud",

                # 点云更新参数
                "point_cloud_sensor.max_range": 5.0,
                "point_cloud_sensor.point_subsample": 1,
                "point_cloud_sensor.padding_offset": 0.01,
                "point_cloud_sensor.padding_scale": 1.0,
                "point_cloud_sensor.max_update_rate": 2.0,

                # 过滤后点云输出话题（可选）
                "point_cloud_sensor.filtered_cloud_topic": "filtered_cloud",
            },
        ],
    )

    return [move_group_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "name",
            default_value="cs625",
            description="The name of the robot.",
        ),
        DeclareLaunchArgument(
            "cs_type",
            default_value="cs625",
            description="The type of the CS robot.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for all TF frames.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware for simulation.",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Use fake sensor commands.",
        ),
        DeclareLaunchArgument(
            "ros2_controllers_file",
            default_value="cs625_moveit_controllers.yaml",
            description="The file name of the MoveIt controllers YAML (inside the config directory).",
        ),
    ]

    return LaunchDescription(
        [
            *declared_arguments,
            OpaqueFunction(function=launch_setup),
        ]
    )
