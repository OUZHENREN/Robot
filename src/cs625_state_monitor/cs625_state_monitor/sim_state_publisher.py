#!/usr/bin/env python3
"""
Simulation mode state publisher for CS625.

When no real robot is available (use_sim:=true), this node:
1. Subscribes to /joint_states (from robot_state_publisher / Gazebo)
2. Converts joint angles from rad to deg
3. Looks up TCP pose via TF (base_link -> my_end_effector_link)
4. Publishes CS625State on /cs625/raw_state

This allows downstream nodes (NBV orchestrator, CSV logger, etc.) to work
without modification in simulation mode.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from cs625_state_monitor.msg import CS625State
from std_msgs.msg import Float64MultiArray

import tf2_ros
import math


class SimStatePublisher(Node):
    """Publishes CS625State from simulation data (joint_states + TF)."""

    # Joint names expected in CS625 order
    JOINT_NAMES = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
    ]

    def __init__(self):
        super().__init__('cs625_sim_state_publisher')

        # Publishers
        self.state_pub = self.create_publisher(CS625State, '/cs625/raw_state', 10)
        self.tcp_force_pub = self.create_publisher(
            Float64MultiArray, '/cs625/tcp_force', 10
        )

        # TF buffer for TCP pose lookup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.get_logger().info('Simulation state publisher started')

    def joint_state_callback(self, msg: JointState):
        """Convert JointState to CS625State and publish."""
        state = CS625State()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'base_link'

        # Extract joint data from the joint state message
        joint_positions_rad = [0.0] * 6
        joint_velocities_rad = [0.0] * 6
        joint_effort = [0.0] * 6

        for i, jname in enumerate(self.JOINT_NAMES):
            if jname in msg.name:
                idx = msg.name.index(jname)
                if idx < len(msg.position):
                    joint_positions_rad[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    joint_velocities_rad[i] = msg.velocity[idx]
                if idx < len(msg.effort):
                    joint_effort[i] = msg.effort[idx]

        # Fill joint data (convert rad to deg)
        for i in range(6):
            state.actual_joint_deg[i] = joint_positions_rad[i] * 180.0 / math.pi
            state.target_joint_deg[i] = joint_positions_rad[i] * 180.0 / math.pi
            state.actual_velocity_deg[i] = joint_velocities_rad[i] * 180.0 / math.pi
            state.joint_torques[i] = joint_effort[i]
            state.joint_current[i] = 0.0
            state.joint_voltage[i] = 0.0
            state.joint_temperature[i] = 25.0  # ambient

        state.has_joint_data = True

        # Get TCP pose via TF
        try:
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform(
                'base_link', 'my_end_effector_link', now, timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Convert translation to mm
            state.actual_tcp_x_mm = t.transform.translation.x * 1000.0
            state.actual_tcp_y_mm = t.transform.translation.y * 1000.0
            state.actual_tcp_z_mm = t.transform.translation.z * 1000.0

            # Convert quaternion to RPY (deg) — ZYX intrinsic = XYZ extrinsic
            qx, qy, qz, qw = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
            sinr_cosp = 2.0 * (qw * qx + qy * qz)
            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            sinp = 2.0 * (qw * qy - qz * qx)
            pitch = math.asin(max(-1.0, min(1.0, sinp)))
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            state.actual_rot_x_deg = roll * 180.0 / math.pi
            state.actual_rot_y_deg = pitch * 180.0 / math.pi
            state.actual_rot_z_deg = yaw * 180.0 / math.pi

            # Target same as actual in simulation
            state.target_tcp_x_mm = state.actual_tcp_x_mm
            state.target_tcp_y_mm = state.actual_tcp_y_mm
            state.target_tcp_z_mm = state.actual_tcp_z_mm
            state.target_rot_x_deg = state.actual_rot_x_deg
            state.target_rot_y_deg = state.actual_rot_y_deg
            state.target_rot_z_deg = state.actual_rot_z_deg

            state.has_actual_tool_data = True
            state.has_target_tool_data = True
        except Exception as e:
            # TF lookup may fail during startup
            pass

        # TCP wrench (zero in simulation unless force sensor active)
        state.actual_tcp_force_x = 0.0
        state.actual_tcp_force_y = 0.0
        state.actual_tcp_force_z = 0.0
        state.actual_tcp_torque_x = 0.0
        state.actual_tcp_torque_y = 0.0
        state.actual_tcp_torque_z = 0.0
        state.has_actual_tcp_wrench = False

        state.parser_ok = True
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = SimStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
