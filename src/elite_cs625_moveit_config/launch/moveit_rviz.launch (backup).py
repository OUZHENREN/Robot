import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    # --- 在此函数内，所有LaunchConfiguration都可以被安全地解析为字符串 ---
    name = LaunchConfiguration("name").perform(context)
    cs_type = LaunchConfiguration("cs_type").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands").perform(context)

    # 构建动态的MoveIt配置
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=name, # <-- 此处使用的是字符串'cs625'
            package_name="elite_cs625_moveit_config"
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

    # RViz节点配置
    rviz_config_file = os.path.join(
        get_package_share_directory("elite_cs625_moveit_config"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict()],
    )
    
    # OpaqueFunction必须返回一个节点/动作的列表
    return [rviz_node]

def generate_launch_description():
    # 1. 声明参数
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="cs625"),
        DeclareLaunchArgument("cs_type", default_value="cs625"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
    ]

    # 2. 返回声明的参数和用于延迟执行的OpaqueFunction
    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup),
    ])
