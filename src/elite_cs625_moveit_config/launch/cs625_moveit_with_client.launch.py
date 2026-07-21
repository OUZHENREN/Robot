from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # 1. 先把 LaunchConfiguration 解成普通字符串
    name = LaunchConfiguration("name").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    cs_type = LaunchConfiguration("cs_type").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands").perform(context)

    # 2. 用字符串调用 MoveItConfigsBuilder（与 move_group.launch.py 完全一致的方式）
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

    moveit_params = moveit_config.to_dict()

    # 3. move_group 节点：加载同一份参数
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        
        parameters=[moveit_params],
    )

    # 4. planned_tcp_path_node：共享 MoveIt 参数 + 自己的一些参数
    planned_tcp_path_node = Node(
        package="cs625_trajectory_tools",
        executable="planned_tcp_path_node",
        name="planned_tcp_path_node",
        output="log",
        
        parameters=[
            moveit_params,
            {
                "planning_group": "cs625_arm",
                "base_frame": "base_link",
                "ee_link": "my_end_effector_link",
                "robot_description_param": "robot_description",
            },
        ],
    )

    # 5. moveit_planner_client_node：同样共享 MoveIt 参数 + 自己的参数
    planner_client_node = Node(
        package="cs625_trajectory_tools",
        executable="moveit_planner_client_node",
        name="moveit_planner_client_node",
        output="log",
        
        parameters=[
            moveit_params,
            {
                "planning_group": "cs625_arm",
                "base_frame": "base_link",
                "ee_link": "my_end_effector_link",
            },
        ],
    )

    # OpaqueFunction 要返回一个 action 列表
    return [move_group_node, planned_tcp_path_node, planner_client_node]


def generate_launch_description():
    # 与 move_group.launch.py 一致的参数声明
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="cs625", description="The name of the robot."),
        DeclareLaunchArgument("cs_type", default_value="cs625", description="The type of the CS robot."),
        DeclareLaunchArgument("tf_prefix", default_value="", description="Prefix for all TF frames."),
        DeclareLaunchArgument("use_fake_hardware", default_value="false", description="Use fake hardware for simulation."),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false", description="Use fake sensor commands."),
    ]

    return LaunchDescription(
        [
            *declared_arguments,
            OpaqueFunction(function=launch_setup),
        ]
    )
