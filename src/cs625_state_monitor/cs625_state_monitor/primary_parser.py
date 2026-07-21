import math
import struct
from typing import Dict, List, Optional


class PrimaryStateParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> List[Dict]:
        results = []
        if data:
            self.buffer.extend(data)

        while len(self.buffer) >= 4:
            if len(self.buffer) < 4:
                break

            data_length = struct.unpack(">i", self.buffer[0:4])[0]

            if data_length <= 0:
                self.buffer.clear()
                break

            if len(self.buffer) < data_length:
                break

            packet = bytes(self.buffer[:data_length])
            self.buffer = self.buffer[data_length:]

            parsed = self.parse_robot_state_packet(packet)
            if parsed is not None:
                results.append(parsed)

        return results

    def parse_robot_state_packet(self, packet: bytes) -> Optional[Dict]:
        if len(packet) < 5:
            return None

        data_type = struct.unpack("B", packet[4:5])[0]
        if data_type != 16:
            return None

        result = {
            "parser_ok": True,
            "has_joint_data": False,
            "has_actual_tool_data": False,
            "has_target_tool_data": False,
            "joint_data": None,
            "actual_tool_data": None,
            "target_tool_data": None,
        }

        sub_data = packet[5:]
        sub_offset = 0
        cartesian_packets = []

        while sub_offset < len(sub_data):
            if len(sub_data) - sub_offset < 5:
                break

            sub_length = struct.unpack(">i", sub_data[sub_offset:sub_offset + 4])[0]
            sub_type = struct.unpack("B", sub_data[sub_offset + 4:sub_offset + 5])[0]

            if sub_length <= 0:
                break

            if len(sub_data) - sub_offset < sub_length:
                break

            sub_packet = sub_data[sub_offset:sub_offset + sub_length]
            sub_offset += sub_length

            if sub_type == 1:
                joint_data = self.parse_joint_subpacket(sub_packet)
                if joint_data is not None:
                    result["joint_data"] = joint_data
                    result["has_joint_data"] = True

            elif sub_type == 4:
                cart_data = self.parse_cartesian_subpacket(sub_packet)
                if cart_data is not None:
                    cartesian_packets.append(cart_data)

        if len(cartesian_packets) >= 1:
            result["actual_tool_data"] = cartesian_packets[0]
            result["has_actual_tool_data"] = True

        if len(cartesian_packets) >= 2:
            result["target_tool_data"] = cartesian_packets[1]
            result["has_target_tool_data"] = True

        return result

    def parse_joint_subpacket(self, data: bytes) -> Optional[Dict]:
        try:
            curr_data_add = 5

            actual_joint_deg = [0.0] * 6
            target_joint_deg = [0.0] * 6
            actual_velocity_deg = [0.0] * 6
            joint_current = [0.0] * 6
            joint_voltage = [0.0] * 6
            joint_temperature = [0.0] * 6
            joint_torques = [0.0] * 6

            for i in range(6):
                actual_joint_rad = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
                actual_joint_deg[i] = round(actual_joint_rad * 180.0 / math.pi, 4)
                curr_data_add += 8

                target_joint_rad = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
                target_joint_deg[i] = round(target_joint_rad * 180.0 / math.pi, 4)
                curr_data_add += 8

                actual_velocity_rad_s = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
                actual_velocity_deg[i] = round(actual_velocity_rad_s * 180.0 / math.pi, 4)
                curr_data_add += 8

                curr_data_add += 12

                current = struct.unpack(">f", data[curr_data_add:curr_data_add + 4])[0]
                joint_current[i] = round(current, 4)
                curr_data_add += 4

                voltage = struct.unpack(">f", data[curr_data_add:curr_data_add + 4])[0]
                joint_voltage[i] = round(voltage, 4)
                curr_data_add += 4

                temperature = struct.unpack(">f", data[curr_data_add:curr_data_add + 4])[0]
                joint_temperature[i] = round(temperature, 4)
                curr_data_add += 4

                torques = struct.unpack(">f", data[curr_data_add:curr_data_add + 4])[0]
                joint_torques[i] = round(torques, 4)
                curr_data_add += 4

                curr_data_add += 5

            return {
                "actual_joint_deg": actual_joint_deg,
                "target_joint_deg": target_joint_deg,
                "actual_velocity_deg": actual_velocity_deg,
                "joint_current": joint_current,
                "joint_voltage": joint_voltage,
                "joint_temperature": joint_temperature,
                "joint_torques": joint_torques,
            }
        except Exception:
            return None

    def parse_cartesian_subpacket(self, data: bytes) -> Optional[Dict]:
        try:
            curr_data_add = 5

            tcp_x_m = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
            curr_data_add += 8
            tcp_y_m = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
            curr_data_add += 8
            tcp_z_m = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
            curr_data_add += 8

            rot_x_rad = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
            curr_data_add += 8
            rot_y_rad = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
            curr_data_add += 8
            rot_z_rad = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
            curr_data_add += 8

            tcp_offset_x_m = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
            curr_data_add += 8
            tcp_offset_y_m = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
            curr_data_add += 8
            tcp_offset_z_m = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
            curr_data_add += 8

            tcp_offset_rx_rad = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
            curr_data_add += 8
            tcp_offset_ry_rad = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]
            curr_data_add += 8
            tcp_offset_rz_rad = struct.unpack(">d", data[curr_data_add:curr_data_add + 8])[0]

            return {
                "tcp_x_mm": round(tcp_x_m * 1000.0, 3),
                "tcp_y_mm": round(tcp_y_m * 1000.0, 3),
                "tcp_z_mm": round(tcp_z_m * 1000.0, 3),
                "rot_x_deg": round(rot_x_rad * 180.0 / math.pi, 3),
                "rot_y_deg": round(rot_y_rad * 180.0 / math.pi, 3),
                "rot_z_deg": round(rot_z_rad * 180.0 / math.pi, 3),
                "tcp_offset_x_mm": round(tcp_offset_x_m * 1000.0, 3),
                "tcp_offset_y_mm": round(tcp_offset_y_m * 1000.0, 3),
                "tcp_offset_z_mm": round(tcp_offset_z_m * 1000.0, 3),
                "tcp_offset_rx_deg": round(tcp_offset_rx_rad * 180.0 / math.pi, 3),
                "tcp_offset_ry_deg": round(tcp_offset_ry_rad * 180.0 / math.pi, 3),
                "tcp_offset_rz_deg": round(tcp_offset_rz_rad * 180.0 / math.pi, 3),
            }
        except Exception:
            return None
