from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "cs_type",
            default_value="cs625",
            description="Type/series of used elite cs robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="eli_cs_robot_description",
            description="Description package containing preview xacro.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="false",
            description="Enable safety limits in preview model.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="Safety margin.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="Safety k position.",
        )
    )

    cs_type = LaunchConfiguration("cs_type")
    description_package = LaunchConfiguration("description_package")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    preview_xacro_file = PathJoinSubstitution([
        FindPackageShare(description_package),
        "urdf",
        "cs625_preview.urdf.xacro",
    ])

    preview_robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            preview_xacro_file,
            " ",
            "name:=cs625_preview ",
            "cs_type:=", cs_type, " ",
            "safety_limits:=", safety_limits, " ",
            "safety_pos_margin:=", safety_pos_margin, " ",
            "safety_k_position:=", safety_k_position,
        ]
    )

    preview_robot_description = {
        "robot_description": ParameterValue(
            preview_robot_description_content,
            value_type=str
        )
    }

    relay_node = Node(
        package="cs625_kinematics",
        executable="preview_joint_state_relay",
        name="preview_joint_state_relay",
        output="screen",
        parameters=[
            {
                "input_topic": "/cs625/selected_multi_config_target_joint_state",
                "output_topic": "/cs625/preview_joint_states",
                "joint_prefix": "preview_",
            }
        ],
    )

    preview_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="preview_robot_state_publisher",
        output="screen",
        parameters=[preview_robot_description],
        remappings=[
            ("/joint_states", "/cs625/preview_joint_states"),
            ("/robot_description", "/preview_robot_description"),
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            relay_node,
            preview_robot_state_publisher_node,
        ]
    )
