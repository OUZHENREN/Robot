#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QPlainTextEdit>
#include <QListWidget>
#include <QTimer>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include "pointcloud_workflow_interfaces/srv/start_sampling.hpp"
#include "pointcloud_workflow_interfaces/srv/stop_sampling.hpp"
#include "pointcloud_workflow_interfaces/srv/preview_sampling_path.hpp"
#include "pointcloud_workflow_interfaces/srv/get_sampling_status.hpp"

#include "pointcloud_workflow_interfaces/srv/list_available_pcds.hpp"
#include "pointcloud_workflow_interfaces/srv/set_enabled_pcds.hpp"
#include "pointcloud_workflow_interfaces/srv/rebuild_merged_cloud.hpp"
#include "pointcloud_workflow_interfaces/srv/get_publisher_status.hpp"

#include "cs625_trajectory_tools/srv/set_named_target_from_current.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "cs625_trajectory_tools/srv/get_named_pose.hpp"

namespace elite_dashboard_rviz_plugin
{

class PointCloudWorkflowPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PointCloudWorkflowPanel(QWidget * parent = nullptr);
  ~PointCloudWorkflowPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onRefreshSamplerStatusClicked();
  void onPreviewSamplingPathClicked();
  void onStartSamplingClicked();
  void onStopSamplingClicked();

  void onSetBoxRoughCaptureClicked();
  void onSetSlotRoughCaptureClicked();

  void onRefreshPublisherListClicked();
  void onRefreshPublisherStatusClicked();
  void onRebuildMergedCloudClicked();
  void onApplyEnabledPcdsClicked();
  void onRebuildEnvironmentCollisionClicked();

private:
  void initUI();
  void initRosNode();
  void appendLog(const QString & text);

  void updateSamplerStatusLabels(
    const QString & state,
    int current_index,
    int total_count,
    bool running,
    bool stop_requested);

  void updatePublisherStatusLabels(
    int available_file_count,
    int enabled_file_count,
    qint64 raw_points,
    qint64 processed_points,
    int reload_count);

  std::vector<std::string> collectCheckedTimestampDirs() const;
  void refreshTimestampListWidget(
    const std::vector<std::string> & dirs,
    const std::vector<bool> & enabled);

  void triggerInitialRefresh();
  void callSetNamedTargetFromCurrent(const std::string & pose_name);

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<pointcloud_workflow_interfaces::srv::StartSampling>::SharedPtr
    start_sampling_client_;
  rclcpp::Client<pointcloud_workflow_interfaces::srv::StopSampling>::SharedPtr
    stop_sampling_client_;
  rclcpp::Client<pointcloud_workflow_interfaces::srv::PreviewSamplingPath>::SharedPtr
    preview_sampling_path_client_;
  rclcpp::Client<pointcloud_workflow_interfaces::srv::GetSamplingStatus>::SharedPtr
    get_sampling_status_client_;

  rclcpp::Client<cs625_trajectory_tools::srv::SetNamedTargetFromCurrent>::SharedPtr
    set_named_target_from_current_client_;

  rclcpp::Client<pointcloud_workflow_interfaces::srv::ListAvailablePcds>::SharedPtr
    list_available_pcds_client_;
  rclcpp::Client<pointcloud_workflow_interfaces::srv::SetEnabledPcds>::SharedPtr
    set_enabled_pcds_client_;
  rclcpp::Client<pointcloud_workflow_interfaces::srv::RebuildMergedCloud>::SharedPtr
    rebuild_merged_cloud_client_;
  rclcpp::Client<pointcloud_workflow_interfaces::srv::GetPublisherStatus>::SharedPtr
    get_publisher_status_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
    rebuild_environment_collision_client_;

  rclcpp::Client<cs625_trajectory_tools::srv::GetNamedPose>::SharedPtr
    get_named_pose_client_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    preview_marker_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
    preview_pose_array_pub_;

  QPushButton * refresh_sampler_status_button_{nullptr};
  QPushButton * preview_sampling_path_button_{nullptr};
  QPushButton * start_sampling_button_{nullptr};
  QPushButton * stop_sampling_button_{nullptr};
  QPushButton * set_box_rough_capture_button_{nullptr};
  QPushButton * set_slot_rough_capture_button_{nullptr};

  QPushButton * refresh_publisher_list_button_{nullptr};
  QPushButton * refresh_publisher_status_button_{nullptr};
  QPushButton * rebuild_merged_cloud_button_{nullptr};
  QPushButton * apply_enabled_pcds_button_{nullptr};
  QPushButton * rebuild_environment_collision_button_{nullptr};

  QLabel * sampler_state_value_{nullptr};
  QLabel * sampler_current_index_value_{nullptr};
  QLabel * sampler_total_count_value_{nullptr};
  QLabel * sampler_running_value_{nullptr};
  QLabel * sampler_stop_requested_value_{nullptr};
  QLabel * sampler_preview_count_value_{nullptr};

  QLabel * publisher_available_count_value_{nullptr};
  QLabel * publisher_enabled_count_value_{nullptr};
  QLabel * publisher_raw_points_value_{nullptr};
  QLabel * publisher_processed_points_value_{nullptr};
  QLabel * publisher_reload_count_value_{nullptr};

  QListWidget * timestamp_list_widget_{nullptr};
  QPlainTextEdit * log_text_{nullptr};

  QTimer * initial_refresh_timer_{nullptr};
};

}  // namespace elite_dashboard_rviz_plugin
