import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def launch_setup(context, *args, **kwargs):
    # --- 在此函数内，所有LaunchConfiguration都可以被安全地解析为字符串 ---
    
    # ==========> 在这里插入我们的“窃听器” <==========
    print("="*50)
    print(f"[DEBUGGER] In move_group.launch.py, the resolved value of 'name' is: '{LaunchConfiguration('name').perform(context)}'")
    print(f"[DEBUGGER] In move_group.launch.py, the resolved value of 'cs_type' is: '{LaunchConfiguration('cs_type').perform(context)}'")
    print("="*50)
    # ==========> 窃听器结束 <==========
    
    name = LaunchConfiguration("name").perform(context)
    cs_type = LaunchConfiguration("cs_type").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands").perform(context)
    ros2_controllers_file = LaunchConfiguration("ros2_controllers_file").perform(context)
    # 使用解析后的字符串变量构建MoveIt配置
    # 使用解析后的字符串变量构建MoveIt配置

    # 使用解析后的字符串变量构建MoveIt配置
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=name,
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
        .trajectory_execution(
           file_path=ros2_controllers_file
        )
        .to_moveit_configs()
    )

    # 生成move_group的启动描述
    move_group_launch = generate_move_group_launch(moveit_config)

    # OpaqueFunction必须返回一个节点/动作的列表
    return [move_group_launch]

def generate_launch_description():
    # 1. 声明此文件将接收的所有参数
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="cs625", description="The name of the robot."),
        DeclareLaunchArgument("cs_type", default_value="cs625", description="The type of the CS robot."),
        DeclareLaunchArgument("tf_prefix", default_value="", description="Prefix for all TF frames."),
        DeclareLaunchArgument("use_fake_hardware", default_value="false", description="Use fake hardware for simulation."),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false", description="Use fake sensor commands."),
        DeclareLaunchArgument("ros2_controllers_file", default_value="cs625_moveit_controllers.yaml", description="The file path for ROS 2 controllers."),
    ]

    # 2. 返回声明的参数和用于延迟执行的OpaqueFunction
    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup),
    ])
