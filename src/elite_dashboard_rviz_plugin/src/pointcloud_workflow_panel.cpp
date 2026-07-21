#include "elite_dashboard_rviz_plugin/pointcloud_workflow_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QMetaObject>
#include <QThread>
#include <QScrollBar>
#include <QListWidgetItem>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_srvs/srv/trigger.hpp>

#include "cs625_trajectory_tools/srv/get_named_pose.hpp"

#include <chrono>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

namespace elite_dashboard_rviz_plugin
{

namespace
{

constexpr int kPreviewSamplingSegmentCount = 5;
constexpr const char * kPreviewMarkerTopic = "/workspace_pointcloud_sampler/preview_markers";
constexpr const char * kPreviewPoseArrayTopic = "/workspace_pointcloud_sampler/preview_pose_array";
constexpr const char * kPreviewFrameId = "base_link";

std::vector<geometry_msgs::msg::PoseStamped> generateInterpolatedSamplesLocal(
  const geometry_msgs::msg::PoseStamped & start_pose,
  const geometry_msgs::msg::PoseStamped & end_pose,
  int segment_count,
  const rclcpp::Time & stamp)
{
  std::vector<geometry_msgs::msg::PoseStamped> result;

  if (segment_count <= 0) {
    auto s = start_pose;
    auto e = end_pose;
    s.header.stamp = stamp;
    e.header.stamp = stamp;
    result.push_back(s);
    result.push_back(e);
    return result;
  }

  tf2::Quaternion q0, q1;
  tf2::fromMsg(start_pose.pose.orientation, q0);
  tf2::fromMsg(end_pose.pose.orientation, q1);
  q0.normalize();
  q1.normalize();

  for (int i = 0; i <= segment_count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(segment_count);

    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = start_pose.header.frame_id.empty() ? kPreviewFrameId : start_pose.header.frame_id;
    p.header.stamp = stamp;

    p.pose.position.x =
      start_pose.pose.position.x +
      t * (end_pose.pose.position.x - start_pose.pose.position.x);
    p.pose.position.y =
      start_pose.pose.position.y +
      t * (end_pose.pose.position.y - start_pose.pose.position.y);
    p.pose.position.z =
      start_pose.pose.position.z +
      t * (end_pose.pose.position.z - start_pose.pose.position.z);

    tf2::Quaternion qi = q0.slerp(q1, t);
    qi.normalize();
    p.pose.orientation = tf2::toMsg(qi);

    result.push_back(p);
  }

  return result;
}

QString poseToQString(const geometry_msgs::msg::PoseStamped & pose, int index)
{
  return QString::fromUtf8(
    "[Sampler][Preview] 点 %1: "
    "frame=%2 "
    "pos=(%3, %4, %5) "
    "quat=(%6, %7, %8, %9)")
    .arg(index)
    .arg(QString::fromStdString(pose.header.frame_id))
    .arg(pose.pose.position.x, 0, 'f', 6)
    .arg(pose.pose.position.y, 0, 'f', 6)
    .arg(pose.pose.position.z, 0, 'f', 6)
    .arg(pose.pose.orientation.x, 0, 'f', 6)
    .arg(pose.pose.orientation.y, 0, 'f', 6)
    .arg(pose.pose.orientation.z, 0, 'f', 6)
    .arg(pose.pose.orientation.w, 0, 'f', 6);
}

visualization_msgs::msg::Marker makeDeleteAllMarker(const std::string & frame_id, const rclcpp::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "sampling_preview";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

}  // namespace

PointCloudWorkflowPanel::PointCloudWorkflowPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  initUI();
}

PointCloudWorkflowPanel::~PointCloudWorkflowPanel() = default;

void PointCloudWorkflowPanel::onInitialize()
{
  initRosNode();
  appendLog(QString::fromUtf8("PointCloudWorkflowPanel 已初始化"));
  triggerInitialRefresh();
}

void PointCloudWorkflowPanel::initRosNode()
{
  auto context = getDisplayContext();
  if (context) {
    auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
    if (ros_node_abstraction) {
      node_ = ros_node_abstraction->get_raw_node();
    }
  }

  if (!node_) {
    node_ = rclcpp::Node::make_shared("pointcloud_workflow_panel_node");
  }

  start_sampling_client_ =
    node_->create_client<pointcloud_workflow_interfaces::srv::StartSampling>(
      "/workspace_pointcloud_sampler/start_sampling");
  stop_sampling_client_ =
    node_->create_client<pointcloud_workflow_interfaces::srv::StopSampling>(
      "/workspace_pointcloud_sampler/stop_sampling");
  preview_sampling_path_client_ =
    node_->create_client<pointcloud_workflow_interfaces::srv::PreviewSamplingPath>(
      "/workspace_pointcloud_sampler/preview_sampling_path");
  get_sampling_status_client_ =
    node_->create_client<pointcloud_workflow_interfaces::srv::GetSamplingStatus>(
      "/workspace_pointcloud_sampler/get_sampling_status");

  set_named_target_from_current_client_ =
    node_->create_client<cs625_trajectory_tools::srv::SetNamedTargetFromCurrent>(
      "/cs625/set_named_target_from_current");

  get_named_pose_client_ =
    node_->create_client<cs625_trajectory_tools::srv::GetNamedPose>(
      "/cs625/get_named_pose");

  list_available_pcds_client_ =
    node_->create_client<pointcloud_workflow_interfaces::srv::ListAvailablePcds>(
      "/environment_point_cloud_publisher/list_available_pcds");
  set_enabled_pcds_client_ =
    node_->create_client<pointcloud_workflow_interfaces::srv::SetEnabledPcds>(
      "/environment_point_cloud_publisher/set_enabled_pcds");
  rebuild_merged_cloud_client_ =
    node_->create_client<pointcloud_workflow_interfaces::srv::RebuildMergedCloud>(
      "/environment_point_cloud_publisher/rebuild_merged_cloud");
  get_publisher_status_client_ =
    node_->create_client<pointcloud_workflow_interfaces::srv::GetPublisherStatus>(
      "/environment_point_cloud_publisher/get_publisher_status");
  rebuild_environment_collision_client_ =
    node_->create_client<std_srvs::srv::Trigger>(
      "/environment_collision_generator/rebuild_environment_collision");

  preview_marker_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(kPreviewMarkerTopic, 10);

  preview_pose_array_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseArray>(kPreviewPoseArrayTopic, 10);
}

void PointCloudWorkflowPanel::triggerInitialRefresh()
{
  if (!initial_refresh_timer_) {
    initial_refresh_timer_ = new QTimer(this);
    initial_refresh_timer_->setSingleShot(true);
    connect(initial_refresh_timer_, &QTimer::timeout, this, [this]() {
      onRefreshPublisherListClicked();
      onRefreshPublisherStatusClicked();
      onRefreshSamplerStatusClicked();
    });
  }

  initial_refresh_timer_->start(1500);
}

void PointCloudWorkflowPanel::initUI()
{
  auto * main_layout = new QVBoxLayout;

  auto * sampler_group = new QGroupBox(QString::fromUtf8("采样器控制（Workspace Sampler）"));
  auto * sampler_layout = new QVBoxLayout;

  auto * sampler_status_layout = new QGridLayout;
  sampler_state_value_ = new QLabel("IDLE");
  sampler_current_index_value_ = new QLabel("-1");
  sampler_total_count_value_ = new QLabel("0");
  sampler_running_value_ = new QLabel("false");
  sampler_stop_requested_value_ = new QLabel("false");
  sampler_preview_count_value_ = new QLabel("0");

  sampler_status_layout->addWidget(new QLabel(QString::fromUtf8("状态：")), 0, 0);
  sampler_status_layout->addWidget(sampler_state_value_, 0, 1);
  sampler_status_layout->addWidget(new QLabel(QString::fromUtf8("当前索引：")), 0, 2);
  sampler_status_layout->addWidget(sampler_current_index_value_, 0, 3);

  sampler_status_layout->addWidget(new QLabel(QString::fromUtf8("总点数：")), 1, 0);
  sampler_status_layout->addWidget(sampler_total_count_value_, 1, 1);
  sampler_status_layout->addWidget(new QLabel(QString::fromUtf8("运行中：")), 1, 2);
  sampler_status_layout->addWidget(sampler_running_value_, 1, 3);

  sampler_status_layout->addWidget(new QLabel(QString::fromUtf8("停止请求：")), 2, 0);
  sampler_status_layout->addWidget(sampler_stop_requested_value_, 2, 1);
  sampler_status_layout->addWidget(new QLabel(QString::fromUtf8("预览点数量：")), 2, 2);
  sampler_status_layout->addWidget(sampler_preview_count_value_, 2, 3);

  sampler_layout->addLayout(sampler_status_layout);

  auto * sampler_button_layout_1 = new QHBoxLayout;
  refresh_sampler_status_button_ = new QPushButton(QString::fromUtf8("刷新采样状态"));
  preview_sampling_path_button_ = new QPushButton(QString::fromUtf8("预览采样路径"));
  start_sampling_button_ = new QPushButton(QString::fromUtf8("开始采样"));
  stop_sampling_button_ = new QPushButton(QString::fromUtf8("停止采样"));

  sampler_button_layout_1->addWidget(refresh_sampler_status_button_);
  sampler_button_layout_1->addWidget(preview_sampling_path_button_);
  sampler_button_layout_1->addWidget(start_sampling_button_);
  sampler_button_layout_1->addWidget(stop_sampling_button_);

  sampler_layout->addLayout(sampler_button_layout_1);

  auto * sampler_button_layout_2 = new QHBoxLayout;
  set_box_rough_capture_button_ =
    new QPushButton(QString::fromUtf8("记录当前为 box_rough_capture_tcp"));
  set_slot_rough_capture_button_ =
    new QPushButton(QString::fromUtf8("记录当前为 slot_rough_capture_tcp"));

  sampler_button_layout_2->addWidget(set_box_rough_capture_button_);
  sampler_button_layout_2->addWidget(set_slot_rough_capture_button_);

  sampler_layout->addLayout(sampler_button_layout_2);

  sampler_group->setLayout(sampler_layout);
  main_layout->addWidget(sampler_group);

  auto * publisher_group = new QGroupBox(QString::fromUtf8("点云发布器控制（Environment Publisher）"));
  auto * publisher_layout = new QVBoxLayout;

  auto * publisher_status_layout = new QGridLayout;
  publisher_available_count_value_ = new QLabel("0");
  publisher_enabled_count_value_ = new QLabel("0");
  publisher_raw_points_value_ = new QLabel("0");
  publisher_processed_points_value_ = new QLabel("0");
  publisher_reload_count_value_ = new QLabel("0");

  publisher_status_layout->addWidget(new QLabel(QString::fromUtf8("可用文件数：")), 0, 0);
  publisher_status_layout->addWidget(publisher_available_count_value_, 0, 1);
  publisher_status_layout->addWidget(new QLabel(QString::fromUtf8("启用文件数：")), 0, 2);
  publisher_status_layout->addWidget(publisher_enabled_count_value_, 0, 3);

  publisher_status_layout->addWidget(new QLabel(QString::fromUtf8("原始点数：")), 1, 0);
  publisher_status_layout->addWidget(publisher_raw_points_value_, 1, 1);
  publisher_status_layout->addWidget(new QLabel(QString::fromUtf8("处理后点数：")), 1, 2);
  publisher_status_layout->addWidget(publisher_processed_points_value_, 1, 3);

  publisher_status_layout->addWidget(new QLabel(QString::fromUtf8("重建次数：")), 2, 0);
  publisher_status_layout->addWidget(publisher_reload_count_value_, 2, 1);

  publisher_layout->addLayout(publisher_status_layout);

  auto * publisher_button_layout = new QHBoxLayout;
  refresh_publisher_list_button_ = new QPushButton(QString::fromUtf8("刷新目录列表"));
  refresh_publisher_status_button_ = new QPushButton(QString::fromUtf8("刷新发布器状态"));
  rebuild_merged_cloud_button_ = new QPushButton(QString::fromUtf8("重建融合点云"));
  apply_enabled_pcds_button_ = new QPushButton(QString::fromUtf8("应用勾选目录"));
  rebuild_environment_collision_button_ = new QPushButton(QString::fromUtf8("生成环境碰撞体"));

  publisher_button_layout->addWidget(refresh_publisher_list_button_);
  publisher_button_layout->addWidget(refresh_publisher_status_button_);
  publisher_button_layout->addWidget(rebuild_merged_cloud_button_);
  publisher_button_layout->addWidget(apply_enabled_pcds_button_);
  publisher_button_layout->addWidget(rebuild_environment_collision_button_);

  publisher_layout->addLayout(publisher_button_layout);

  auto * list_group = new QGroupBox(QString::fromUtf8("可用 PCD 时间戳目录"));
  auto * list_layout = new QVBoxLayout;
  timestamp_list_widget_ = new QListWidget;
  list_layout->addWidget(timestamp_list_widget_);
  list_group->setLayout(list_layout);

  publisher_layout->addWidget(list_group);
  publisher_group->setLayout(publisher_layout);
  main_layout->addWidget(publisher_group);

  auto * log_group = new QGroupBox(QString::fromUtf8("日志"));
  auto * log_layout = new QVBoxLayout;
  log_text_ = new QPlainTextEdit;
  log_text_->setReadOnly(true);
  log_layout->addWidget(log_text_);
  log_group->setLayout(log_layout);
  main_layout->addWidget(log_group);

  setLayout(main_layout);

  connect(
    refresh_sampler_status_button_, &QPushButton::clicked,
    this, &PointCloudWorkflowPanel::onRefreshSamplerStatusClicked);
  connect(
    preview_sampling_path_button_, &QPushButton::clicked,
    this, &PointCloudWorkflowPanel::onPreviewSamplingPathClicked);
  connect(
    start_sampling_button_, &QPushButton::clicked,
    this, &PointCloudWorkflowPanel::onStartSamplingClicked);
  connect(
    stop_sampling_button_, &QPushButton::clicked,
    this, &PointCloudWorkflowPanel::onStopSamplingClicked);
  connect(
    set_box_rough_capture_button_, &QPushButton::clicked,
    this, &PointCloudWorkflowPanel::onSetBoxRoughCaptureClicked);
  connect(
    set_slot_rough_capture_button_, &QPushButton::clicked,
    this, &PointCloudWorkflowPanel::onSetSlotRoughCaptureClicked);

  connect(
    refresh_publisher_list_button_, &QPushButton::clicked,
    this, &PointCloudWorkflowPanel::onRefreshPublisherListClicked);
  connect(
    refresh_publisher_status_button_, &QPushButton::clicked,
    this, &PointCloudWorkflowPanel::onRefreshPublisherStatusClicked);
  connect(
    rebuild_merged_cloud_button_, &QPushButton::clicked,
    this, &PointCloudWorkflowPanel::onRebuildMergedCloudClicked);
  connect(
    apply_enabled_pcds_button_, &QPushButton::clicked,
    this, &PointCloudWorkflowPanel::onApplyEnabledPcdsClicked);
  connect(
    rebuild_environment_collision_button_, &QPushButton::clicked,
    this, &PointCloudWorkflowPanel::onRebuildEnvironmentCollisionClicked);
}

void PointCloudWorkflowPanel::appendLog(const QString & text)
{
  if (!log_text_) {
    return;
  }

  auto append_fn = [this, text]() {
    if (!log_text_) {
      return;
    }
    log_text_->appendPlainText(text);
    auto * bar = log_text_->verticalScrollBar();
    if (bar) {
      bar->setValue(bar->maximum());
    }
  };

  if (QThread::currentThread() == this->thread()) {
    append_fn();
  } else {
    QMetaObject::invokeMethod(this, append_fn, Qt::QueuedConnection);
  }
}

void PointCloudWorkflowPanel::updateSamplerStatusLabels(
  const QString & state,
  int current_index,
  int total_count,
  bool running,
  bool stop_requested)
{
  if (sampler_state_value_) {
    sampler_state_value_->setText(state);
  }
  if (sampler_current_index_value_) {
    sampler_current_index_value_->setText(QString::number(current_index));
  }
  if (sampler_total_count_value_) {
    sampler_total_count_value_->setText(QString::number(total_count));
  }
  if (sampler_running_value_) {
    sampler_running_value_->setText(running ? "true" : "false");
  }
  if (sampler_stop_requested_value_) {
    sampler_stop_requested_value_->setText(stop_requested ? "true" : "false");
  }
}

void PointCloudWorkflowPanel::updatePublisherStatusLabels(
  int available_file_count,
  int enabled_file_count,
  qint64 raw_points,
  qint64 processed_points,
  int reload_count)
{
  if (publisher_available_count_value_) {
    publisher_available_count_value_->setText(QString::number(available_file_count));
  }
  if (publisher_enabled_count_value_) {
    publisher_enabled_count_value_->setText(QString::number(enabled_file_count));
  }
  if (publisher_raw_points_value_) {
    publisher_raw_points_value_->setText(QString::number(raw_points));
  }
  if (publisher_processed_points_value_) {
    publisher_processed_points_value_->setText(QString::number(processed_points));
  }
  if (publisher_reload_count_value_) {
    publisher_reload_count_value_->setText(QString::number(reload_count));
  }
}

std::vector<std::string> PointCloudWorkflowPanel::collectCheckedTimestampDirs() const
{
  std::vector<std::string> dirs;
  if (!timestamp_list_widget_) {
    return dirs;
  }

  for (int i = 0; i < timestamp_list_widget_->count(); ++i) {
    auto * item = timestamp_list_widget_->item(i);
    if (!item) {
      continue;
    }
    if (item->checkState() == Qt::Checked) {
      dirs.push_back(item->text().toStdString());
    }
  }

  return dirs;
}

void PointCloudWorkflowPanel::refreshTimestampListWidget(
  const std::vector<std::string> & dirs,
  const std::vector<bool> & enabled)
{
  if (!timestamp_list_widget_) {
    return;
  }

  timestamp_list_widget_->clear();

  const std::size_t n = std::min(dirs.size(), enabled.size());
  for (std::size_t i = 0; i < n; ++i) {
    auto * item = new QListWidgetItem(QString::fromStdString(dirs[i]));
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(enabled[i] ? Qt::Checked : Qt::Unchecked);
    timestamp_list_widget_->addItem(item);
  }
}

void PointCloudWorkflowPanel::callSetNamedTargetFromCurrent(const std::string & pose_name)
{
  if (!set_named_target_from_current_client_) {
    appendLog(QString::fromUtf8("[Sampler] 错误：set_named_target client 未初始化"));
    return;
  }

  if (!set_named_target_from_current_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[Sampler] 错误：服务 /cs625/set_named_target_from_current 不可用"));
    return;
  }

  auto request =
    std::make_shared<cs625_trajectory_tools::srv::SetNamedTargetFromCurrent::Request>();
  request->name = pose_name;

  appendLog(
    QString::fromUtf8("[Sampler] 正在记录当前命名目标为：%1")
    .arg(QString::fromStdString(pose_name)));

  set_named_target_from_current_client_->async_send_request(
    request,
    [this, pose_name](
      rclcpp::Client<cs625_trajectory_tools::srv::SetNamedTargetFromCurrent>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[Sampler] 记录命名目标响应为空"));
          return;
        }

        appendLog(
          QString::fromUtf8("[Sampler] 记录命名目标：name=%1 success=%2 message=%3")
          .arg(QString::fromStdString(pose_name))
          .arg(response->success ? "true" : "false")
          .arg(QString::fromStdString(response->message)));
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[Sampler] 记录命名目标异常：%1").arg(e.what()));
      }
    });
}

void PointCloudWorkflowPanel::onSetBoxRoughCaptureClicked()
{
  callSetNamedTargetFromCurrent("box_rough_capture_tcp");
}

void PointCloudWorkflowPanel::onSetSlotRoughCaptureClicked()
{
  callSetNamedTargetFromCurrent("slot_rough_capture_tcp");
}

void PointCloudWorkflowPanel::onRefreshSamplerStatusClicked()
{
  if (!get_sampling_status_client_) {
    appendLog(QString::fromUtf8("[Sampler] 错误：状态 client 未初始化"));
    return;
  }

  if (!get_sampling_status_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[Sampler] 错误：服务 /workspace_pointcloud_sampler/get_sampling_status 不可用"));
    return;
  }

  auto request =
    std::make_shared<pointcloud_workflow_interfaces::srv::GetSamplingStatus::Request>();

  get_sampling_status_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<pointcloud_workflow_interfaces::srv::GetSamplingStatus>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[Sampler] 状态响应为空"));
          return;
        }

        QMetaObject::invokeMethod(
          this,
          [this, response]() {
            updateSamplerStatusLabels(
              QString::fromStdString(response->state),
              response->current_index,
              response->total_count,
              response->running,
              response->stop_requested);
          },
          Qt::QueuedConnection);

        appendLog(
          QString::fromUtf8("[Sampler] 状态刷新成功：state=%1 current=%2 total=%3 running=%4 stop_requested=%5")
          .arg(QString::fromStdString(response->state))
          .arg(response->current_index)
          .arg(response->total_count)
          .arg(response->running ? "true" : "false")
          .arg(response->stop_requested ? "true" : "false"));
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[Sampler] 状态查询异常：%1").arg(e.what()));
      }
    });
}

void PointCloudWorkflowPanel::onPreviewSamplingPathClicked()
{
  if (!preview_sampling_path_client_) {
    appendLog(QString::fromUtf8("[Sampler] 错误：preview client 未初始化"));
    return;
  }

  if (!preview_marker_pub_) {
    appendLog(QString::fromUtf8("[Sampler] 错误：preview marker publisher 未初始化"));
    return;
  }

  if (!preview_pose_array_pub_) {
    appendLog(QString::fromUtf8("[Sampler] 错误：preview pose array publisher 未初始化"));
    return;
  }

  if (!preview_sampling_path_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[Sampler] 错误：服务 /workspace_pointcloud_sampler/preview_sampling_path 不可用"));
    return;
  }

  appendLog(QString::fromUtf8("[Sampler] 开始请求后台预览采样路径"));

  auto request =
    std::make_shared<pointcloud_workflow_interfaces::srv::PreviewSamplingPath::Request>();

  preview_sampling_path_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<pointcloud_workflow_interfaces::srv::PreviewSamplingPath>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[Sampler] 预览失败：响应为空"));
          return;
        }

        if (!response->success) {
          appendLog(
            QString::fromUtf8("[Sampler] 预览失败：%1")
            .arg(QString::fromStdString(response->message)));
          return;
        }

        const auto & samples = response->poses;
        if (samples.empty()) {
          appendLog(QString::fromUtf8("[Sampler] 预览失败：返回的采样位姿为空"));
          return;
        }

        const auto stamp = node_->now();

        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = stamp;
        pose_array.header.frame_id =
          samples.front().header.frame_id.empty() ?
          kPreviewFrameId : samples.front().header.frame_id;

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(
          makeDeleteAllMarker(pose_array.header.frame_id, stamp));

        int marker_id = 1;

        for (std::size_t i = 0; i < samples.size(); ++i) {
          pose_array.poses.push_back(samples[i].pose);

          const bool is_start = (i == 0);
          const bool is_end = (i + 1 == samples.size());

          float r = 0.2f;
          float g = 0.8f;
          float b = 1.0f;

          if (is_start) {
            r = 0.1f; g = 1.0f; b = 0.1f;
          } else if (is_end) {
            r = 1.0f; g = 0.2f; b = 0.2f;
          }

          visualization_msgs::msg::Marker sphere;
          sphere.header = samples[i].header;
          sphere.ns = "sampling_preview_sphere";
          sphere.id = marker_id++;
          sphere.type = visualization_msgs::msg::Marker::SPHERE;
          sphere.action = visualization_msgs::msg::Marker::ADD;
          sphere.pose = samples[i].pose;
          sphere.scale.x = 0.03;
          sphere.scale.y = 0.03;
          sphere.scale.z = 0.03;
          sphere.color.a = 0.95f;
          sphere.color.r = r;
          sphere.color.g = g;
          sphere.color.b = b;
          marker_array.markers.push_back(sphere);

          visualization_msgs::msg::Marker arrow;
          arrow.header = samples[i].header;
          arrow.ns = "sampling_preview_arrow";
          arrow.id = marker_id++;
          arrow.type = visualization_msgs::msg::Marker::ARROW;
          arrow.action = visualization_msgs::msg::Marker::ADD;
          arrow.pose = samples[i].pose;
          arrow.scale.x = 0.12;
          arrow.scale.y = 0.01;
          arrow.scale.z = 0.01;
          arrow.color.a = 0.95f;
          arrow.color.r = r;
          arrow.color.g = g;
          arrow.color.b = b;
          marker_array.markers.push_back(arrow);

          visualization_msgs::msg::Marker text;
          text.header = samples[i].header;
          text.ns = "sampling_preview_text";
          text.id = marker_id++;
          text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          text.action = visualization_msgs::msg::Marker::ADD;
          text.pose = samples[i].pose;
          text.pose.position.z += 0.05;
          text.scale.z = 0.035;
          text.color.a = 1.0f;
          text.color.r = 1.0f;
          text.color.g = 1.0f;
          text.color.b = 0.2f;

          std::ostringstream oss;
          oss << i;
          if (is_start) {
            oss << " (start)";
          } else if (is_end) {
            oss << " (end)";
          }
          text.text = oss.str();
          marker_array.markers.push_back(text);
        }

        preview_pose_array_pub_->publish(pose_array);
        preview_marker_pub_->publish(marker_array);

        QMetaObject::invokeMethod(
          this,
          [this, count = static_cast<int>(samples.size())]() {
            if (sampler_preview_count_value_) {
              sampler_preview_count_value_->setText(QString::number(count));
            }
          },
          Qt::QueuedConnection);

        appendLog(
          QString::fromUtf8("[Sampler] 预览成功：共生成 %1 个采样位姿点，已发布到 RViz")
          .arg(samples.size()));

        appendLog(
          QString::fromUtf8("[Sampler] Marker topic: %1")
          .arg(kPreviewMarkerTopic));
        appendLog(
          QString::fromUtf8("[Sampler] PoseArray topic: %1")
          .arg(kPreviewPoseArrayTopic));

        for (std::size_t i = 0; i < samples.size(); ++i) {
          appendLog(poseToQString(samples[i], static_cast<int>(i)));
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[Sampler] 预览请求异常：%1").arg(e.what()));
      }
    });
}

void PointCloudWorkflowPanel::onStartSamplingClicked()
{
  if (!start_sampling_client_) {
    appendLog(QString::fromUtf8("[Sampler] 错误：start client 未初始化"));
    return;
  }

  if (!start_sampling_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[Sampler] 错误：服务 /workspace_pointcloud_sampler/start_sampling 不可用"));
    return;
  }

  auto request =
    std::make_shared<pointcloud_workflow_interfaces::srv::StartSampling::Request>();

  start_sampling_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<pointcloud_workflow_interfaces::srv::StartSampling>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[Sampler] Start 响应为空"));
          return;
        }

        appendLog(
          QString::fromUtf8("[Sampler] Start：accepted=%1 success=%2 message=%3")
          .arg(response->accepted ? "true" : "false")
          .arg(response->success ? "true" : "false")
          .arg(QString::fromStdString(response->message)));

        if (response->success) {
          QMetaObject::invokeMethod(
            this,
            [this]() {
              onRefreshSamplerStatusClicked();
            },
            Qt::QueuedConnection);
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[Sampler] Start 异常：%1").arg(e.what()));
      }
    });
}

void PointCloudWorkflowPanel::onStopSamplingClicked()
{
  if (!stop_sampling_client_) {
    appendLog(QString::fromUtf8("[Sampler] 错误：stop client 未初始化"));
    return;
  }

  if (!stop_sampling_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[Sampler] 错误：服务 /workspace_pointcloud_sampler/stop_sampling 不可用"));
    return;
  }

  auto request =
    std::make_shared<pointcloud_workflow_interfaces::srv::StopSampling::Request>();

  stop_sampling_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<pointcloud_workflow_interfaces::srv::StopSampling>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[Sampler] Stop 响应为空"));
          return;
        }

        appendLog(
          QString::fromUtf8("[Sampler] Stop：accepted=%1 success=%2 message=%3")
          .arg(response->accepted ? "true" : "false")
          .arg(response->success ? "true" : "false")
          .arg(QString::fromStdString(response->message)));

        if (response->success) {
          QMetaObject::invokeMethod(
            this,
            [this]() {
              onRefreshSamplerStatusClicked();
            },
            Qt::QueuedConnection);
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[Sampler] Stop 异常：%1").arg(e.what()));
      }
    });
}

void PointCloudWorkflowPanel::onRefreshPublisherListClicked()
{
  if (!list_available_pcds_client_) {
    appendLog(QString::fromUtf8("[Publisher] 错误：list client 未初始化"));
    return;
  }

  if (!list_available_pcds_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[Publisher] 错误：服务 /environment_point_cloud_publisher/list_available_pcds 不可用"));
    return;
  }

  auto request =
    std::make_shared<pointcloud_workflow_interfaces::srv::ListAvailablePcds::Request>();

  list_available_pcds_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<pointcloud_workflow_interfaces::srv::ListAvailablePcds>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[Publisher] 列表响应为空"));
          return;
        }

        QMetaObject::invokeMethod(
          this,
          [this, response]() {
            refreshTimestampListWidget(response->timestamp_dirs, response->enabled);
          },
          Qt::QueuedConnection);

        appendLog(
          QString::fromUtf8("[Publisher] 列表刷新：success=%1 dirs=%2 message=%3")
          .arg(response->success ? "true" : "false")
          .arg(response->timestamp_dirs.size())
          .arg(QString::fromStdString(response->message)));
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[Publisher] 列表刷新异常：%1").arg(e.what()));
      }
    });
}

void PointCloudWorkflowPanel::onRefreshPublisherStatusClicked()
{
  if (!get_publisher_status_client_) {
    appendLog(QString::fromUtf8("[Publisher] 错误：status client 未初始化"));
    return;
  }

  if (!get_publisher_status_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[Publisher] 错误：服务 /environment_point_cloud_publisher/get_publisher_status 不可用"));
    return;
  }

  auto request =
    std::make_shared<pointcloud_workflow_interfaces::srv::GetPublisherStatus::Request>();

  get_publisher_status_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<pointcloud_workflow_interfaces::srv::GetPublisherStatus>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[Publisher] 状态响应为空"));
          return;
        }

        QMetaObject::invokeMethod(
          this,
          [this, response]() {
            updatePublisherStatusLabels(
              response->available_file_count,
              response->enabled_file_count,
              response->raw_points,
              response->processed_points,
              response->reload_count);
          },
          Qt::QueuedConnection);

        appendLog(
          QString::fromUtf8("[Publisher] 状态刷新：success=%1 available=%2 enabled=%3 raw=%4 processed=%5 reload=%6")
          .arg(response->success ? "true" : "false")
          .arg(response->available_file_count)
          .arg(response->enabled_file_count)
          .arg(response->raw_points)
          .arg(response->processed_points)
          .arg(response->reload_count));
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[Publisher] 状态刷新异常：%1").arg(e.what()));
      }
    });
}

void PointCloudWorkflowPanel::onRebuildMergedCloudClicked()
{
  if (!rebuild_merged_cloud_client_) {
    appendLog(QString::fromUtf8("[Publisher] 错误：rebuild client 未初始化"));
    return;
  }

  if (!rebuild_merged_cloud_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[Publisher] 错误：服务 /environment_point_cloud_publisher/rebuild_merged_cloud 不可用"));
    return;
  }

  auto request =
    std::make_shared<pointcloud_workflow_interfaces::srv::RebuildMergedCloud::Request>();

  rebuild_merged_cloud_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<pointcloud_workflow_interfaces::srv::RebuildMergedCloud>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[Publisher] 重建响应为空"));
          return;
        }

        appendLog(
          QString::fromUtf8("[Publisher] 重建：success=%1 files=%2 raw=%3 processed=%4 message=%5")
          .arg(response->success ? "true" : "false")
          .arg(response->file_count)
          .arg(response->raw_points)
          .arg(response->processed_points)
          .arg(QString::fromStdString(response->message)));

        if (response->success) {
          QMetaObject::invokeMethod(
            this,
            [this]() {
              onRefreshPublisherStatusClicked();
            },
            Qt::QueuedConnection);
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[Publisher] 重建异常：%1").arg(e.what()));
      }
    });
}

void PointCloudWorkflowPanel::onRebuildEnvironmentCollisionClicked()
{
  if (!rebuild_environment_collision_client_) {
    appendLog(QString::fromUtf8("[EnvironmentCollision] 错误：rebuild client 未初始化"));
    return;
  }

  if (!rebuild_environment_collision_button_) {
    appendLog(QString::fromUtf8("[EnvironmentCollision] 错误：按钮未初始化"));
    return;
  }

  if (!rebuild_environment_collision_client_->wait_for_service(1s)) {
    appendLog(
      QString::fromUtf8(
        "[EnvironmentCollision] 错误：服务 "
        "/environment_collision_generator/rebuild_environment_collision 不可用"));
    return;
  }

  rebuild_environment_collision_button_->setEnabled(false);
  appendLog(QString::fromUtf8("[EnvironmentCollision] 正在请求生成环境碰撞体..."));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  rebuild_environment_collision_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        auto response = future.get();

        QMetaObject::invokeMethod(
          this,
          [this]() {
            if (rebuild_environment_collision_button_) {
              rebuild_environment_collision_button_->setEnabled(true);
            }
          },
          Qt::QueuedConnection);

        if (!response) {
          appendLog(QString::fromUtf8("[EnvironmentCollision] 响应为空"));
          return;
        }

        appendLog(
          QString::fromUtf8("[EnvironmentCollision] 生成完成：success=%1 message=%2")
            .arg(response->success ? "true" : "false")
            .arg(QString::fromStdString(response->message)));

        if (response->success) {
          QMetaObject::invokeMethod(
            this,
            [this]() {
              onRefreshPublisherStatusClicked();
            },
            Qt::QueuedConnection);
        }
      } catch (const std::exception & e) {
        QMetaObject::invokeMethod(
          this,
          [this]() {
            if (rebuild_environment_collision_button_) {
              rebuild_environment_collision_button_->setEnabled(true);
            }
          },
          Qt::QueuedConnection);

        appendLog(QString::fromUtf8("[EnvironmentCollision] 请求异常：%1").arg(e.what()));
      }
    });
}

void PointCloudWorkflowPanel::onApplyEnabledPcdsClicked()
{
  if (!set_enabled_pcds_client_) {
    appendLog(QString::fromUtf8("[Publisher] 错误：set_enabled client 未初始化"));
    return;
  }

  if (!set_enabled_pcds_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[Publisher] 错误：服务 /environment_point_cloud_publisher/set_enabled_pcds 不可用"));
    return;
  }

  auto dirs = collectCheckedTimestampDirs();

  auto request =
    std::make_shared<pointcloud_workflow_interfaces::srv::SetEnabledPcds::Request>();
  request->timestamp_dirs = dirs;

  set_enabled_pcds_client_->async_send_request(
    request,
    [this, dirs](
      rclcpp::Client<pointcloud_workflow_interfaces::srv::SetEnabledPcds>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[Publisher] 应用勾选响应为空"));
          return;
        }

        appendLog(
          QString::fromUtf8("[Publisher] 应用勾选：selected=%1 success=%2 enabled_count=%3 message=%4")
          .arg(dirs.size())
          .arg(response->success ? "true" : "false")
          .arg(response->enabled_count)
          .arg(QString::fromStdString(response->message)));

        if (response->success) {
          QMetaObject::invokeMethod(
            this,
            [this]() {
              onRefreshPublisherListClicked();
              onRefreshPublisherStatusClicked();
            },
            Qt::QueuedConnection);
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[Publisher] 应用勾选异常：%1").arg(e.what()));
      }
    });
}

void PointCloudWorkflowPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void PointCloudWorkflowPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace elite_dashboard_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  elite_dashboard_rviz_plugin::PointCloudWorkflowPanel,
  rviz_common::Panel)
