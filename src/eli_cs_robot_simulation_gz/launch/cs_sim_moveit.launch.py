# Author: Chen Shichao

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # 获取参数
    cs_type = LaunchConfiguration("cs_type")
    safety_limits = LaunchConfiguration("safety_limits")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    world_file = LaunchConfiguration("world_file")
    launch_rviz = LaunchConfiguration("launch_rviz")

    cs_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("eli_cs_robot_simulation_gz"), "/launch", "/cs_sim_control.launch.py"
        ]),
        launch_arguments={
            "cs_type": cs_type,
            "safety_limits": safety_limits,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "launch_rviz": "false",
            "world_file": world_file,
        }.items(),
    )

    cs_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("elite_cs625_moveit_config"), "/launch", "/cs625_moveit.launch.py"
        ]),
        launch_arguments={
            "cs_type": cs_type,
            "safety_limits": safety_limits,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )

    cs_moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("elite_cs625_moveit_config"),
            "/launch",
            "/moveit_rviz.launch.py",
        ]),
        launch_arguments={
            "cs_type": cs_type,
            "tf_prefix": prefix,
            "use_fake_hardware": "false",
        }.items(),
        condition=IfCondition(launch_rviz),
    )

    return [cs_control_launch, cs_moveit_launch, cs_moveit_rviz_launch]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "cs_type",
            description="Type/series of used ELITE CS robot.",
            choices=["cs63", "cs66", "cs612", "cs616", "cs620", "cs625"],
            default_value="cs625",
        ),
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="eli_cs_robot_simulation_gz",
            description="Package with the controller's configuration in 'config' folder.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="cs_controllers.yaml",
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="eli_cs_robot_description",
            description="Description package with robot URDF/XACRO files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="cs625.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="elite_cs625_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files.",
        ),
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="cs625.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup.",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file to load.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch the MoveIt RViz configuration.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
