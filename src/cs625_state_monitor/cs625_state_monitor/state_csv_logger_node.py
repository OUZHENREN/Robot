#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from cs625_state_monitor.csv_writer import CSVWriterManager
from cs625_state_monitor.msg import CS625State
from cs625_state_monitor.srv import StateLoggingControl


class StateCSVLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__('cs625_state_csv_logger_node')

        self.declare_parameter('output_dir', os.path.expanduser('~/cs625_data'))
        self.declare_parameter('write_tool_target_when_missing', False)

        self.output_dir = self.get_parameter('output_dir').value
        self.write_tool_target_when_missing = self.get_parameter('write_tool_target_when_missing').value

        os.makedirs(self.output_dir, exist_ok=True)

        self.writer = None
        self.start_time_sec = None
        self.record_state = 'STOP'

        self.raw_state_sub = self.create_subscription(
            CS625State,
            '/cs625/raw_state',
            self.raw_state_callback,
            10
        )

        self.tcp_force_sub = self.create_subscription(
            Float64MultiArray,
            '/cs625/tcp_force',
            self.tcp_force_callback,
            10
        )

        self.compliant_metrics_sub = self.create_subscription(
            Float64MultiArray,
            '/cs625/compliant_placement/metrics',
            self.compliant_metrics_callback,
            10
        )

        self.control_service = self.create_service(
            StateLoggingControl,
            '/cs625/state_logging_control',
            self.handle_control_request
        )

        self.get_logger().info(
            f'CSV logger ready. Initial state: {self.record_state}, '
            f'base output_dir: {self.output_dir}'
        )

    def get_elapsed_time_from_stamp(self, stamp) -> float:
        current_time_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        if self.start_time_sec is None:
            self.start_time_sec = current_time_sec

        return round(current_time_sec - self.start_time_sec, 6)

    def get_elapsed_time_now(self) -> float:
        now_msg = self.get_clock().now().to_msg()
        return self.get_elapsed_time_from_stamp(now_msg)

    def handle_control_request(
        self,
        request: StateLoggingControl.Request,
        response: StateLoggingControl.Response
    ) -> StateLoggingControl.Response:
        command = (request.command or '').strip().lower()

        if command == 'start':
            ok, msg = self.start_recording()
        elif command == 'pause':
            ok, msg = self.pause_recording()
        elif command == 'stop':
            ok, msg = self.stop_recording()
        else:
            ok = False
            msg = f'Unknown command: {request.command} (expected: start|pause|stop)'

        response.success = ok
        response.message = msg
        if ok:
            self.get_logger().info(f'[control] {command} -> {msg}')
        else:
            self.get_logger().warning(f'[control] {command} -> {msg}')
        return response

    def start_recording(self):
        if self.record_state == 'RECORDING':
            return True, 'Already recording'

        if self.writer is None:
            self.writer = CSVWriterManager(self.output_dir)
            self.writer.create_all()
            self.start_time_sec = None
            self.record_state = 'RECORDING'
            return True, f'Started new recording session at {self.writer.output_folder}'
        else:
            self.record_state = 'RECORDING'
            return True, 'Resumed recording'

    def pause_recording(self):
        if self.record_state != 'RECORDING':
            return False, f'Cannot pause when state = {self.record_state}'
        self.record_state = 'PAUSED'
        return True, 'Paused recording'

    def stop_recording(self):
        if self.writer is not None:
            try:
                self.writer.close_all()
            except Exception:
                pass
            self.writer = None

        self.record_state = 'STOP'
        self.start_time_sec = None
        return True, 'Stopped recording and closed CSV files'

    def raw_state_callback(self, msg: CS625State) -> None:
        if self.record_state != 'RECORDING':
            return

        if self.writer is None:
            self.get_logger().warn(
                'raw_state_callback called while record_state=RECORDING but writer is None; '
                'ignoring this message.'
            )
            return

        elapsed_time = self.get_elapsed_time_from_stamp(msg.header.stamp)

        if msg.has_joint_data:
            self.writer.write_row(
                'actual_joint_deg.csv',
                [elapsed_time] + list(msg.actual_joint_deg)
            )
            self.writer.write_row(
                'target_joint_deg.csv',
                [elapsed_time] + list(msg.target_joint_deg)
            )
            self.writer.write_row(
                'actual_velocity_deg.csv',
                [elapsed_time] + list(msg.actual_velocity_deg)
            )
            self.writer.write_row(
                'joint_current.csv',
                [elapsed_time] + list(msg.joint_current)
            )
            self.writer.write_row(
                'joint_voltage.csv',
                [elapsed_time] + list(msg.joint_voltage)
            )
            self.writer.write_row(
                'joint_temperature.csv',
                [elapsed_time] + list(msg.joint_temperature)
            )
            self.writer.write_row(
                'joint_torques.csv',
                [elapsed_time] + list(msg.joint_torques)
            )

        if msg.has_actual_tool_data:
            self.writer.write_row(
                'tool_data_actual.csv',
                [
                    elapsed_time,
                    msg.actual_tcp_x_mm,
                    msg.actual_tcp_y_mm,
                    msg.actual_tcp_z_mm,
                    msg.actual_rot_x_deg,
                    msg.actual_rot_y_deg,
                    msg.actual_rot_z_deg,
                    msg.actual_tcp_offset_x_mm,
                    msg.actual_tcp_offset_y_mm,
                    msg.actual_tcp_offset_z_mm,
                    msg.actual_tcp_offset_rx_deg,
                    msg.actual_tcp_offset_ry_deg,
                    msg.actual_tcp_offset_rz_deg,
                ]
            )

    def tcp_force_callback(self, msg: Float64MultiArray) -> None:
        if self.record_state != 'RECORDING':
            return

        if self.writer is None:
            self.get_logger().warn(
                'tcp_force_callback called while record_state=RECORDING but writer is None; '
                'ignoring this message.'
            )
            return

        if len(msg.data) < 6:
            return

        elapsed_time = self.get_elapsed_time_now()

        self.writer.write_row(
            'tcp_wrench_actual.csv',
            [
                elapsed_time,
                float(msg.data[0]),
                float(msg.data[1]),
                float(msg.data[2]),
                float(msg.data[3]),
                float(msg.data[4]),
                float(msg.data[5]),
            ]
        )

    def compliant_metrics_callback(self, msg: Float64MultiArray) -> None:
        if self.record_state != 'RECORDING':
            return

        if self.writer is None:
            self.get_logger().warn(
                'compliant_metrics_callback called while record_state=RECORDING but writer is None; '
                'ignoring this message.'
            )
            return

        if len(msg.data) < 2:
            return

        elapsed_time = self.get_elapsed_time_now()

        self.writer.write_row(
            'compliant_placement_metrics.csv',
            [
                elapsed_time,
                float(msg.data[0]),
                float(msg.data[1]),
            ]
        )

    def destroy_node(self):
        try:
            if self.writer is not None:
                self.writer.close_all()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StateCSVLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

