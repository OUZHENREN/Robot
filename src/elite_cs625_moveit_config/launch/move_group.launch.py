import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # 解析 LaunchConfiguration
    name = LaunchConfiguration("name").perform(context)
    cs_type = LaunchConfiguration("cs_type").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands").perform(context)
    ros2_controllers_file = LaunchConfiguration("ros2_controllers_file").perform(context)

    # 构造 MoveIt 配置（不在这里传 trajectory_execution 文件，单独给 node）
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
        .to_moveit_configs()
    )

    # 计算 cs625_moveit_controllers.yaml 的绝对路径
    package_share = os.path.join(
        os.getenv("COLCON_CURRENT_PREFIX", os.path.join(os.getcwd(), "install")),
        "elite_cs625_moveit_config",
        "share",
        "elite_cs625_moveit_config",
        "config",
    )
    controllers_yaml_path = os.path.join(package_share, ros2_controllers_file)

    # 创建 move_group 节点，显式加载 controllers_yaml_path
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            controllers_yaml_path,
        ],
    )

    return [move_group_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="cs625", description="The name of the robot."),
        DeclareLaunchArgument("cs_type", default_value="cs625", description="The type of the CS robot."),
        DeclareLaunchArgument("tf_prefix", default_value="", description="Prefix for all TF frames."),
        DeclareLaunchArgument("use_fake_hardware", default_value="false", description="Use fake hardware for simulation."),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false", description="Use fake sensor commands."),
        # 这里期望 cs625_moveit_controllers.yaml 已为一级 key，无需 ros__parameters 包裹
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
