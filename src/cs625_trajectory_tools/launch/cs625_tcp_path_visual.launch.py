# cs625_trajectory_tools/launch/cs625_tcp_path_visual.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    common_params = [{
        # 与 MoveIt 配置保持一致
        "planning_group": "cs625_arm",
        "base_frame": "base_link",
        "ee_link": "tool0",  # TODO: 这里要用你在 MoveIt/SRDF 中的实际 EE link 名
        # 如果你在源码里用的是 "robot_description_param" 其他名字，也可以覆盖：
        # "robot_description_param": "robot_description",
    }]

#    planned_tcp_path_node = Node(
#        package="cs625_trajectory_tools",
#        executable="planned_tcp_path_node",
#        name="planned_tcp_path_node",
#        output="log",
#        parameters=common_params,
#    )

    display_traj_bridge_node = Node(
        package="cs625_trajectory_tools",
        executable="display_trajectory_bridge_node",
        name="display_trajectory_bridge_node",
        output="log",
        
        # 可以用默认参数，也可以在这里改 topic 名：
        # parameters=[{
        #     "input_topic": "/display_planned_path",
        #     "output_topic": "/cs625/planned_joint_trajectory",
        # }],
    )

    return LaunchDescription([
#        planned_tcp_path_node,
        display_traj_bridge_node,
    ])
