import csv
import datetime
import os
from typing import Dict, List


class CSVWriterManager:
    def __init__(self, output_dir: str) -> None:
        self.base_output_dir = output_dir
        self.output_folder = self._create_output_folder()
        self.csv_files: Dict[str, object] = {}
        self.csv_writers: Dict[str, csv.writer] = {}

    def _create_output_folder(self) -> str:
        timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        folder_name = f"CS625data_{timestamp}"
        folder_path = os.path.join(self.base_output_dir, folder_name)
        os.makedirs(folder_path, exist_ok=True)
        return folder_path

    def create_all(self) -> None:
        joint_header = ['Time(s)', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        tool_header = [
            'Time(s)',
            'tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm',
            'rot_x_deg', 'rot_y_deg', 'rot_z_deg',
            'tcp_offset_x_mm', 'tcp_offset_y_mm', 'tcp_offset_z_mm',
            'tcp_offset_rx_deg', 'tcp_offset_ry_deg', 'tcp_offset_rz_deg'
        ]

        tcp_wrench_header = [
            'Time(s)',
            'force_x', 'force_y', 'force_z',
            'torque_x', 'torque_y', 'torque_z'
        ]

        compliant_metrics_header = [
            'Time(s)',
            'tau_metric',
            'filtered_tau_metric'
        ]

        file_headers = {
            'actual_joint_deg.csv': joint_header,
            'target_joint_deg.csv': joint_header,
            'actual_velocity_deg.csv': joint_header,
            'joint_current.csv': joint_header,
            'joint_voltage.csv': joint_header,
            'joint_temperature.csv': joint_header,
            'joint_torques.csv': joint_header,
            'tool_data_actual.csv': tool_header,
            'tcp_wrench_actual.csv': tcp_wrench_header,
            'compliant_placement_metrics.csv': compliant_metrics_header,
        }

        for filename, header in file_headers.items():
            path = os.path.join(self.output_folder, filename)
            file_obj = open(path, 'w', newline='')
            writer = csv.writer(file_obj)
            writer.writerow(header)
            file_obj.flush()

            self.csv_files[filename] = file_obj
            self.csv_writers[filename] = writer

    def write_row(self, filename: str, row: List) -> None:
        if filename in self.csv_writers:
            self.csv_writers[filename].writerow(row)
            self.csv_files[filename].flush()

    def close_all(self) -> None:
        for file_obj in self.csv_files.values():
            try:
                file_obj.close()
            except Exception:
                pass
