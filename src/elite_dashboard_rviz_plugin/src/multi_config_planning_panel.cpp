#include "elite_dashboard_rviz_plugin/multi_config_planning_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QMetaObject>
#include <QThread>
#include <QScrollBar>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <sstream>
#include <iomanip>

#include <QGridLayout>

using namespace std::chrono_literals;

namespace elite_dashboard_rviz_plugin
{

namespace
{

constexpr const char * kSelectedJointStateTopic =
  "/cs625/selected_multi_config_target_joint_state";
  
constexpr const char * kCurrentJointStateTopic = "/joint_states";
constexpr const char * kSelectedTargetMarkerTopic =
  "/cs625/selected_multi_config_target_marker";
constexpr const char * kPreviewPathMarkerTopic =
  "/cs625/multi_config_preview_markers";
constexpr const char * kPreviewPathPoseArrayTopic =
  "/cs625/multi_config_preview_pose_array";
constexpr const char * kPlannedTcpPathTopic =
  "/cs625/planned_tcp_path";
constexpr const char * kDefaultFrameId = "base_link";

visualization_msgs::msg::Marker makeDeleteAllMarker(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & ns)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

std::vector<std::string> defaultJointNames()
{
  return {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };
}

}  // namespace

MultiConfigPlanningPanel::MultiConfigPlanningPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  initUI();
}

MultiConfigPlanningPanel::~MultiConfigPlanningPanel() = default;

void MultiConfigPlanningPanel::onInitialize()
{
  initRosNode();
  appendLog(QString::fromUtf8("MultiConfigPlanningPanel 已初始化"));
}

void MultiConfigPlanningPanel::initRosNode()
{
  auto context = getDisplayContext();
  if (context) {
    auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
    if (ros_node_abstraction) {
      node_ = ros_node_abstraction->get_raw_node();
    }
  }

  if (!node_) {
    node_ = rclcpp::Node::make_shared("multi_config_planning_panel_node");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  get_named_pose_client_ =
    node_->create_client<cs625_trajectory_tools::srv::GetNamedPose>(
      "/cs625/get_named_pose");

  solve_ik_all_client_ =
    node_->create_client<cs625_kinematics::srv::SolveIKAll>(
      "/solve_ik_all");

  set_named_target_client_ =
    node_->create_client<cs625_trajectory_tools::srv::SetNamedTarget>(
      "/cs625/set_named_target");

  get_named_target_client_ =
    node_->create_client<cs625_trajectory_tools::srv::GetNamedTarget>(
      "/cs625/get_named_target");

  plan_to_target_client_ =
    node_->create_client<cs625_trajectory_tools::srv::PlanToTarget>(
      "/cs625/plan_to_target_srv");

  execute_last_plan_client_ =
    node_->create_client<cs625_trajectory_tools::srv::ExecuteLastPlan>(
      "/cs625/execute_last_plan");

  selected_joint_state_pub_ =
    node_->create_publisher<sensor_msgs::msg::JointState>(
      kSelectedJointStateTopic, 10);

  selected_target_marker_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      kSelectedTargetMarkerTopic, 10);

  preview_path_marker_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      kPreviewPathMarkerTopic, 10);

  preview_path_pose_array_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseArray>(
      kPreviewPathPoseArrayTopic, 10);

  planned_tcp_path_sub_ =
    node_->create_subscription<nav_msgs::msg::Path>(
      kPlannedTcpPathTopic,
      10,
      [this](const nav_msgs::msg::Path::SharedPtr msg)
      {
        if (!msg) {
          appendLog(QString::fromUtf8("[路径预览] 收到空 Path 消息"));
          return;
        }

        publishPreviewPathMarkers(*msg);

        appendLog(
          QString::fromUtf8("[路径预览] 已接收并发布 TCP Path，可视化点数=%1")
          .arg(msg->poses.size()));

        for (std::size_t i = 0; i < msg->poses.size(); ++i) {
          appendLog(poseToQString(msg->poses[i], static_cast<int>(i)));
        }
      });
      
  current_joint_state_sub_ =
    node_->create_subscription<sensor_msgs::msg::JointState>(
      kCurrentJointStateTopic,
      20,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg)
      {
        if (!msg) {
          return;
        }

        const auto expected_names = defaultJointNames();
        const bool was_complete = has_complete_current_joint_state_;

        std::array<double, 6> new_positions{};
        std::array<bool, 6> new_valid{{false, false, false, false, false, false}};

        for (std::size_t expected_idx = 0; expected_idx < expected_names.size(); ++expected_idx) {
          const auto & expected_name = expected_names[expected_idx];

          for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
            if (msg->name[i] == expected_name) {
              new_positions[expected_idx] = msg->position[i];
              new_valid[expected_idx] = true;
              break;
            }
          }
        }

        current_joint_positions_ = new_positions;
        current_joint_position_valid_ = new_valid;

        has_complete_current_joint_state_ = true;
        for (bool valid : current_joint_position_valid_) {
          if (!valid) {
            has_complete_current_joint_state_ = false;
            break;
          }
        }

        if (!was_complete && has_complete_current_joint_state_) {
          appendLog(QString::fromUtf8("[JointState] 已接收到完整当前关节状态：%1")
            .arg(currentSeedToQString()));
        }
      });
}

void MultiConfigPlanningPanel::populateTargetCombo()
{
  if (!target_combo_) {
    return;
  }

  target_combo_->clear();

  target_combo_->addItem("box_rough_capture_tcp");
  target_combo_->addItem("slot_rough_capture_tcp");
  target_combo_->addItem("slot_origin");
  target_combo_->addItem("slot_precision_view_tcp");
  target_combo_->addItem("slot_insert_tcp");
  target_combo_->addItem("slot_pre_insert_rotated_tcp");
  target_combo_->addItem("slot_pre_insert_tcp");
  target_combo_->addItem("transit_tcp");
  target_combo_->addItem("retreat_tcp");
  target_combo_->addItem("box_origin");
  target_combo_->addItem("box_precision_view_tcp");
  target_combo_->addItem("box_grasp_tcp");
  target_combo_->addItem("box_pre_grasp_tcp");
  target_combo_->addItem("box_place_tcp");
  target_combo_->addItem("home_tcp");
}

void MultiConfigPlanningPanel::initUI()
{
  auto * main_layout = new QVBoxLayout;

  auto * target_group = new QGroupBox(QString::fromUtf8("目标选择"));
  auto * target_layout = new QGridLayout;

  target_combo_ = new QComboBox;
  populateTargetCombo();

  load_target_pose_button_ = new QPushButton(QString::fromUtf8("读取目标 Pose"));

  current_pose_frame_value_ = new QLabel("-");
  current_pose_xyz_value_ = new QLabel("-");
  current_pose_quat_value_ = new QLabel("-");

  target_layout->addWidget(new QLabel(QString::fromUtf8("目标名称：")), 0, 0);
  target_layout->addWidget(target_combo_, 0, 1);
  target_layout->addWidget(load_target_pose_button_, 0, 2);

  target_layout->addWidget(new QLabel(QString::fromUtf8("frame_id：")), 1, 0);
  target_layout->addWidget(current_pose_frame_value_, 1, 1, 1, 2);

  target_layout->addWidget(new QLabel(QString::fromUtf8("位置 xyz：")), 2, 0);
  target_layout->addWidget(current_pose_xyz_value_, 2, 1, 1, 2);

  target_layout->addWidget(new QLabel(QString::fromUtf8("姿态 qx qy qz qw：")), 3, 0);
  target_layout->addWidget(current_pose_quat_value_, 3, 1, 1, 2);

  target_group->setLayout(target_layout);
  main_layout->addWidget(target_group);

  auto * ik_group = new QGroupBox(QString::fromUtf8("多构型 IK"));
  auto * ik_layout = new QVBoxLayout;

  auto * ik_button_layout = new QHBoxLayout;
  solve_all_ik_button_ = new QPushButton(QString::fromUtf8("求全部 IK 解"));
  recommend_solution_button_ = new QPushButton(QString::fromUtf8("推荐最近解"));

  ik_button_layout->addWidget(solve_all_ik_button_);
  ik_button_layout->addWidget(recommend_solution_button_);
  ik_layout->addLayout(ik_button_layout);

  auto * filter_layout = new QGridLayout;
  shoulder_filter_combo_ = new QComboBox;
  elbow_filter_combo_ = new QComboBox;
  wrist_filter_combo_ = new QComboBox;

  shoulder_filter_combo_->addItem(QString::fromUtf8("都要"));
  shoulder_filter_combo_->addItem("Front");
  shoulder_filter_combo_->addItem("Rear");

  elbow_filter_combo_->addItem(QString::fromUtf8("都要"));
  elbow_filter_combo_->addItem("Up");
  elbow_filter_combo_->addItem("Down");

  wrist_filter_combo_->addItem(QString::fromUtf8("都要"));
  wrist_filter_combo_->addItem("NonFlip");
  wrist_filter_combo_->addItem("Flip");

  filter_layout->addWidget(new QLabel(QString::fromUtf8("肩部构型：")), 0, 0);
  filter_layout->addWidget(shoulder_filter_combo_, 0, 1);
  filter_layout->addWidget(new QLabel(QString::fromUtf8("肘部构型：")), 0, 2);
  filter_layout->addWidget(elbow_filter_combo_, 0, 3);
  filter_layout->addWidget(new QLabel(QString::fromUtf8("腕部构型：")), 0, 4);
  filter_layout->addWidget(wrist_filter_combo_, 0, 5);

  ik_layout->addLayout(filter_layout);

  solution_table_ = new QTableWidget;
  solution_table_->setColumnCount(11);
  solution_table_->setHorizontalHeaderLabels(
    QStringList()
      << "ID" << "Shoulder" << "Elbow" << "Wrist" << "Cost"
      << "J1" << "J2" << "J3" << "J4" << "J5" << "J6");
  solution_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  solution_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  solution_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  solution_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  ik_layout->addWidget(solution_table_);
  ik_group->setLayout(ik_layout);
  main_layout->addWidget(ik_group);

  auto * action_group = new QGroupBox(QString::fromUtf8("选中解操作"));
  auto * action_layout = new QGridLayout;

  preview_selected_solution_button_ =
    new QPushButton(QString::fromUtf8("预览选中解"));
  save_selected_target_button_ =
    new QPushButton(QString::fromUtf8("保存为 selected_multi_config_target"));
  preview_path_button_ =
    new QPushButton(QString::fromUtf8("预览路径（PlanToTarget，不执行）"));
  execute_last_plan_button_ =
    new QPushButton(QString::fromUtf8("执行最近规划"));
  refresh_saved_target_button_ =
    new QPushButton(QString::fromUtf8("读取 selected_multi_config_target"));

  saved_target_pose_value_ = new QLabel("-");
  saved_target_joint_value_ = new QLabel("-");

  action_layout->addWidget(preview_selected_solution_button_, 0, 0);
  action_layout->addWidget(save_selected_target_button_, 0, 1);
  action_layout->addWidget(preview_path_button_, 1, 0);
  action_layout->addWidget(execute_last_plan_button_, 1, 1);
  action_layout->addWidget(refresh_saved_target_button_, 2, 0, 1, 2);

  action_layout->addWidget(new QLabel(QString::fromUtf8("已保存目标 Pose：")), 3, 0);
  action_layout->addWidget(saved_target_pose_value_, 3, 1);

  action_layout->addWidget(new QLabel(QString::fromUtf8("已保存目标关节：")), 4, 0);
  action_layout->addWidget(saved_target_joint_value_, 4, 1);

  action_group->setLayout(action_layout);
  main_layout->addWidget(action_group);

  auto * log_group = new QGroupBox(QString::fromUtf8("日志"));
  auto * log_layout = new QVBoxLayout;

  log_text_ = new QPlainTextEdit;
  log_text_->setReadOnly(true);

  log_layout->addWidget(log_text_);
  log_group->setLayout(log_layout);
  main_layout->addWidget(log_group);

  setLayout(main_layout);

  connect(
    load_target_pose_button_, &QPushButton::clicked,
    this, &MultiConfigPlanningPanel::onLoadTargetPoseClicked);

  connect(
    solve_all_ik_button_, &QPushButton::clicked,
    this, &MultiConfigPlanningPanel::onSolveAllIkClicked);

  connect(
    recommend_solution_button_, &QPushButton::clicked,
    this, &MultiConfigPlanningPanel::onRecommendSolutionClicked);

  connect(
    preview_selected_solution_button_, &QPushButton::clicked,
    this, &MultiConfigPlanningPanel::onPreviewSelectedSolutionClicked);

  connect(
    save_selected_target_button_, &QPushButton::clicked,
    this, &MultiConfigPlanningPanel::onSaveSelectedTargetClicked);

  connect(
    preview_path_button_, &QPushButton::clicked,
    this, &MultiConfigPlanningPanel::onPreviewPathClicked);

  connect(
    execute_last_plan_button_, &QPushButton::clicked,
    this, &MultiConfigPlanningPanel::onExecuteLastPlanClicked);

  connect(
    refresh_saved_target_button_, &QPushButton::clicked,
    this, &MultiConfigPlanningPanel::onRefreshSavedTargetClicked);

  connect(
    shoulder_filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &MultiConfigPlanningPanel::onFilterChanged);
  connect(
    elbow_filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &MultiConfigPlanningPanel::onFilterChanged);
  connect(
    wrist_filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &MultiConfigPlanningPanel::onFilterChanged);

  connect(
    solution_table_, &QTableWidget::itemSelectionChanged,
    this, &MultiConfigPlanningPanel::onSolutionSelectionChanged);

  updateCurrentTargetPoseLabels();
  updateSavedTargetLabels();
}

void MultiConfigPlanningPanel::appendLog(const QString & text)
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

void MultiConfigPlanningPanel::updateCurrentTargetPoseLabels()
{
  if (!has_current_target_pose_) {
    if (current_pose_frame_value_) current_pose_frame_value_->setText("-");
    if (current_pose_xyz_value_) current_pose_xyz_value_->setText("-");
    if (current_pose_quat_value_) current_pose_quat_value_->setText("-");
    return;
  }

  if (current_pose_frame_value_) {
    current_pose_frame_value_->setText(
      QString::fromStdString(current_target_pose_.header.frame_id));
  }

  if (current_pose_xyz_value_) {
    current_pose_xyz_value_->setText(
      QString("(%1, %2, %3)")
      .arg(current_target_pose_.pose.position.x, 0, 'f', 6)
      .arg(current_target_pose_.pose.position.y, 0, 'f', 6)
      .arg(current_target_pose_.pose.position.z, 0, 'f', 6));
  }

  if (current_pose_quat_value_) {
    current_pose_quat_value_->setText(
      QString("(%1, %2, %3, %4)")
      .arg(current_target_pose_.pose.orientation.x, 0, 'f', 6)
      .arg(current_target_pose_.pose.orientation.y, 0, 'f', 6)
      .arg(current_target_pose_.pose.orientation.z, 0, 'f', 6)
      .arg(current_target_pose_.pose.orientation.w, 0, 'f', 6));
  }
}

void MultiConfigPlanningPanel::updateSavedTargetLabels()
{
  if (!has_saved_target_) {
    if (saved_target_pose_value_) saved_target_pose_value_->setText("-");
    if (saved_target_joint_value_) saved_target_joint_value_->setText("-");
    return;
  }

  if (saved_target_pose_value_) {
    saved_target_pose_value_->setText(
      QString("xyz=(%1, %2, %3), quat=(%4, %5, %6, %7)")
      .arg(saved_target_pose_.pose.position.x, 0, 'f', 4)
      .arg(saved_target_pose_.pose.position.y, 0, 'f', 4)
      .arg(saved_target_pose_.pose.position.z, 0, 'f', 4)
      .arg(saved_target_pose_.pose.orientation.x, 0, 'f', 4)
      .arg(saved_target_pose_.pose.orientation.y, 0, 'f', 4)
      .arg(saved_target_pose_.pose.orientation.z, 0, 'f', 4)
      .arg(saved_target_pose_.pose.orientation.w, 0, 'f', 4));
  }

  if (saved_target_joint_value_) {
    QStringList parts;
    for (std::size_t i = 0; i < saved_target_joint_state_.position.size(); ++i) {
      parts << QString("J%1=%2").arg(i + 1).arg(saved_target_joint_state_.position[i], 0, 'f', 4);
    }
    saved_target_joint_value_->setText(parts.join(", "));
  }
}

QString MultiConfigPlanningPanel::shoulderToQString(int8_t v) const
{
  if (v == 0) return "Front";
  if (v == 1) return "Rear";
  return "Unknown";
}

QString MultiConfigPlanningPanel::elbowToQString(int8_t v) const
{
  if (v == 0) return "Up";
  if (v == 1) return "Down";
  return "Unknown";
}

QString MultiConfigPlanningPanel::wristToQString(int8_t v) const
{
  if (v == 0) return "NonFlip";
  if (v == 1) return "Flip";
  return "Unknown";
}

bool MultiConfigPlanningPanel::solutionPassesCurrentFilters(
  const cs625_kinematics::msg::IkSolution & sol) const
{
  if (shoulder_filter_combo_) {
    const int idx = shoulder_filter_combo_->currentIndex();
    if (idx == 1 && sol.shoulder != 0) return false;
    if (idx == 2 && sol.shoulder != 1) return false;
  }

  if (elbow_filter_combo_) {
    const int idx = elbow_filter_combo_->currentIndex();
    if (idx == 1 && sol.elbow != 0) return false;
    if (idx == 2 && sol.elbow != 1) return false;
  }

  if (wrist_filter_combo_) {
    const int idx = wrist_filter_combo_->currentIndex();
    if (idx == 1 && sol.wrist != 0) return false;
    if (idx == 2 && sol.wrist != 1) return false;
  }

  return true;
}

void MultiConfigPlanningPanel::refreshSolutionTable()
{
  if (!solution_table_) {
    return;
  }

  solution_table_->clearContents();
  solution_table_->setRowCount(0);
  filtered_solution_indices_.clear();

  for (std::size_t i = 0; i < all_solutions_.size(); ++i) {
    if (!solutionPassesCurrentFilters(all_solutions_[i])) {
      continue;
    }
    filtered_solution_indices_.push_back(static_cast<int>(i));
  }

  solution_table_->setRowCount(static_cast<int>(filtered_solution_indices_.size()));

  for (int row = 0; row < static_cast<int>(filtered_solution_indices_.size()); ++row) {
    const auto & sol = all_solutions_[filtered_solution_indices_[row]];

    solution_table_->setItem(row, 0, new QTableWidgetItem(QString::number(filtered_solution_indices_[row])));
    solution_table_->setItem(row, 1, new QTableWidgetItem(shoulderToQString(sol.shoulder)));
    solution_table_->setItem(row, 2, new QTableWidgetItem(elbowToQString(sol.elbow)));
    solution_table_->setItem(row, 3, new QTableWidgetItem(wristToQString(sol.wrist)));
    solution_table_->setItem(row, 4, new QTableWidgetItem(QString::number(sol.cost, 'f', 6)));

    for (int j = 0; j < 6; ++j) {
      solution_table_->setItem(
        row, 5 + j,
        new QTableWidgetItem(QString::number(sol.joints[j], 'f', 6)));
    }
  }

  appendLog(
    QString::fromUtf8("[IK] 表格刷新完成：总解数=%1，筛选后=%2")
    .arg(all_solutions_.size())
    .arg(filtered_solution_indices_.size()));
}

bool MultiConfigPlanningPanel::getSelectedSolution(
  cs625_kinematics::msg::IkSolution & sol,
  int & filtered_row_index) const
{
  filtered_row_index = -1;

  if (!solution_table_) {
    return false;
  }

  filtered_row_index = solution_table_->currentRow();

  if (filtered_row_index < 0) {
    const auto ranges = solution_table_->selectedRanges();
    if (ranges.isEmpty()) {
      return false;
    }
    filtered_row_index = ranges.first().topRow();
  }

  if (filtered_row_index < 0 ||
      filtered_row_index >= static_cast<int>(filtered_solution_indices_.size())) {
    return false;
  }

  const int original_index = filtered_solution_indices_[filtered_row_index];
  if (original_index < 0 || original_index >= static_cast<int>(all_solutions_.size())) {
    return false;
  }

  sol = all_solutions_[original_index];
  return true;
}

sensor_msgs::msg::JointState MultiConfigPlanningPanel::makeJointStateFromSolution(
  const cs625_kinematics::msg::IkSolution & sol) const
{
  sensor_msgs::msg::JointState js;
  js.header.stamp = node_ ? node_->now() : rclcpp::Clock().now();
  js.name = defaultJointNames();
  js.position.resize(6);

  for (int i = 0; i < 6; ++i) {
    js.position[i] = sol.joints[i];
  }

  return js;
}

bool MultiConfigPlanningPanel::tryBuildSeedJointState(
  sensor_msgs::msg::JointState & seed_joint_state) const
{
  seed_joint_state = sensor_msgs::msg::JointState{};
  seed_joint_state.header.stamp = node_ ? node_->now() : rclcpp::Clock().now();
  seed_joint_state.name = defaultJointNames();

  if (!has_complete_current_joint_state_) {
    return false;
  }

  seed_joint_state.position.resize(6);
  for (int i = 0; i < 6; ++i) {
    if (!current_joint_position_valid_[i]) {
      return false;
    }
    seed_joint_state.position[i] = current_joint_positions_[i];
  }

  return true;
}

QString MultiConfigPlanningPanel::currentSeedToQString() const
{
  if (!has_complete_current_joint_state_) {
    return QString::fromUtf8("当前关节种子不可用");
  }

  QStringList parts;
  for (int i = 0; i < 6; ++i) {
    parts << QString("J%1=%2").arg(i + 1).arg(current_joint_positions_[i], 0, 'f', 6);
  }
  return parts.join(", ");
}

void MultiConfigPlanningPanel::publishTargetMarker(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & text)
{
  if (!selected_target_marker_pub_ || !node_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  const auto stamp = node_->now();
  const std::string frame_id =
    pose.header.frame_id.empty() ? std::string(kDefaultFrameId) : pose.header.frame_id;

  array.markers.push_back(makeDeleteAllMarker(frame_id, stamp, "selected_target"));

  visualization_msgs::msg::Marker sphere;
  sphere.header = pose.header;
  sphere.header.stamp = stamp;
  sphere.ns = "selected_target_sphere";
  sphere.id = 1;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.pose = pose.pose;
  sphere.scale.x = 0.035;
  sphere.scale.y = 0.035;
  sphere.scale.z = 0.035;
  sphere.color.a = 0.95f;
  sphere.color.r = 0.1f;
  sphere.color.g = 0.9f;
  sphere.color.b = 0.2f;
  array.markers.push_back(sphere);

  visualization_msgs::msg::Marker arrow;
  arrow.header = pose.header;
  arrow.header.stamp = stamp;
  arrow.ns = "selected_target_arrow";
  arrow.id = 2;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;
  arrow.pose = pose.pose;
  arrow.scale.x = 0.12;
  arrow.scale.y = 0.01;
  arrow.scale.z = 0.01;
  arrow.color.a = 0.95f;
  arrow.color.r = 0.1f;
  arrow.color.g = 0.7f;
  arrow.color.b = 1.0f;
  array.markers.push_back(arrow);

  visualization_msgs::msg::Marker label;
  label.header = pose.header;
  label.header.stamp = stamp;
  label.ns = "selected_target_text";
  label.id = 3;
  label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label.action = visualization_msgs::msg::Marker::ADD;
  label.pose = pose.pose;
  label.pose.position.z += 0.06;
  label.scale.z = 0.035;
  label.color.a = 1.0f;
  label.color.r = 1.0f;
  label.color.g = 1.0f;
  label.color.b = 0.1f;
  label.text = text;
  array.markers.push_back(label);

  selected_target_marker_pub_->publish(array);
}

void MultiConfigPlanningPanel::publishSelectedSolutionVisual(
  const cs625_kinematics::msg::IkSolution & sol)
{
  if (!selected_joint_state_pub_) {
    appendLog(QString::fromUtf8("[预览解] JointState publisher 未初始化"));
    return;
  }

  if (!has_current_target_pose_) {
    appendLog(QString::fromUtf8("[预览解] 当前还没有目标 Pose"));
    return;
  }

  auto js = makeJointStateFromSolution(sol);
  selected_joint_state_pub_->publish(js);

  publishTargetMarker(current_target_pose_, "selected_multi_config_target");

  appendLog(QString::fromUtf8("[预览解] 已发布目标 JointState 到 %1")
    .arg(kSelectedJointStateTopic));
  appendLog(QString::fromUtf8("[预览解] 已发布目标 Marker 到 %1")
    .arg(kSelectedTargetMarkerTopic));
}

QString MultiConfigPlanningPanel::poseToQString(
  const geometry_msgs::msg::PoseStamped & pose,
  int index) const
{
  return QString::fromUtf8(
    "[Path] 点 %1: frame=%2 pos=(%3, %4, %5) quat=(%6, %7, %8, %9)")
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

void MultiConfigPlanningPanel::publishPreviewPathMarkers(const nav_msgs::msg::Path & path)
{
  if (!preview_path_marker_pub_ || !preview_path_pose_array_pub_ || !node_) {
    return;
  }

  const auto stamp = node_->now();
  const std::string frame_id =
    path.header.frame_id.empty() ? std::string(kDefaultFrameId) : path.header.frame_id;

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = stamp;
  pose_array.header.frame_id = frame_id;

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(makeDeleteAllMarker(frame_id, stamp, "multi_config_preview"));

  visualization_msgs::msg::Marker line;
  line.header.stamp = stamp;
  line.header.frame_id = frame_id;
  line.ns = "multi_config_preview_line";
  line.id = 1;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.008;
  line.color.a = 0.95f;
  line.color.r = 0.2f;
  line.color.g = 0.9f;
  line.color.b = 1.0f;

  int marker_id = 10;

  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    pose_array.poses.push_back(path.poses[i].pose);

    geometry_msgs::msg::Point p;
    p.x = path.poses[i].pose.position.x;
    p.y = path.poses[i].pose.position.y;
    p.z = path.poses[i].pose.position.z;
    line.points.push_back(p);

    const bool is_start = (i == 0);
    const bool is_end = (i + 1 == path.poses.size());

    float r = 0.2f;
    float g = 0.8f;
    float b = 1.0f;
    if (is_start) {
      r = 0.1f; g = 1.0f; b = 0.1f;
    } else if (is_end) {
      r = 1.0f; g = 0.2f; b = 0.2f;
    }

    visualization_msgs::msg::Marker sphere;
    sphere.header = path.poses[i].header;
    sphere.header.stamp = stamp;
    sphere.ns = "multi_config_preview_sphere";
    sphere.id = marker_id++;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = path.poses[i].pose;
    sphere.scale.x = 0.025;
    sphere.scale.y = 0.025;
    sphere.scale.z = 0.025;
    sphere.color.a = 0.95f;
    sphere.color.r = r;
    sphere.color.g = g;
    sphere.color.b = b;
    marker_array.markers.push_back(sphere);
  }

  marker_array.markers.push_back(line);

  preview_path_pose_array_pub_->publish(pose_array);
  preview_path_marker_pub_->publish(marker_array);
}

void MultiConfigPlanningPanel::onLoadTargetPoseClicked()
{
  if (!tf_buffer_) {
    appendLog(QString::fromUtf8("[目标] TF buffer 未初始化"));
    return;
  }

  const QString target_name = target_combo_ ? target_combo_->currentText().trimmed() : "";
  if (target_name.isEmpty()) {
    appendLog(QString::fromUtf8("[目标] 目标名称为空"));
    return;
  }

  const std::string base_frame = kDefaultFrameId;
  const std::string target_frame = target_name.toStdString();

  appendLog(QString::fromUtf8("[目标] 正在通过 TF 读取目标 Pose：%1（%2 -> %3）")
    .arg(target_name)
    .arg(QString::fromStdString(base_frame))
    .arg(target_name));

  try {
    const auto transform = tf_buffer_->lookupTransform(
      base_frame,
      target_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(1.0));

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = base_frame;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    current_target_pose_ = pose;
    has_current_target_pose_ = true;

    QMetaObject::invokeMethod(
      this,
      [this]() {
        updateCurrentTargetPoseLabels();
      },
      Qt::QueuedConnection);

    appendLog(QString::fromUtf8("[目标] 成功通过 TF 读取 Pose：%1").arg(target_name));
    appendLog(poseToQString(current_target_pose_, 0));
  } catch (const tf2::TransformException & ex) {
    has_current_target_pose_ = false;

    QMetaObject::invokeMethod(
      this,
      [this]() {
        updateCurrentTargetPoseLabels();
      },
      Qt::QueuedConnection);

    appendLog(QString::fromUtf8("[目标] TF 读取失败：%1").arg(ex.what()));
  } catch (const std::exception & e) {
    has_current_target_pose_ = false;

    QMetaObject::invokeMethod(
      this,
      [this]() {
        updateCurrentTargetPoseLabels();
      },
      Qt::QueuedConnection);

    appendLog(QString::fromUtf8("[目标] 读取异常：%1").arg(e.what()));
  }
}
void MultiConfigPlanningPanel::onSolveAllIkClicked()
{
  if (!has_current_target_pose_) {
    appendLog(QString::fromUtf8("[IK] 请先读取目标 Pose"));
    return;
  }

  if (!solve_ik_all_client_) {
    appendLog(QString::fromUtf8("[IK] solve_ik_all client 未初始化"));
    return;
  }

  if (!solve_ik_all_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[IK] 服务 /solve_ik_all 不可用"));
    return;
  }

  auto request = std::make_shared<cs625_kinematics::srv::SolveIKAll::Request>();
  request->target_pose = current_target_pose_;
  request->max_solutions = 8;

  sensor_msgs::msg::JointState seed_joint_state;
  if (tryBuildSeedJointState(seed_joint_state)) {
    request->seed = seed_joint_state;
    appendLog(QString::fromUtf8("[IK] 开始求解全部 IK 解，使用当前关节状态作为排序种子"));
    appendLog(QString::fromUtf8("[IK] 当前 seed: %1").arg(currentSeedToQString()));
  } else {
    request->seed.name = defaultJointNames();
    request->seed.position.clear();
    appendLog(QString::fromUtf8("[IK] 开始求解全部 IK 解，但当前关节状态不可用，将退化为无有效种子排序"));
  }

  solve_ik_all_client_->async_send_request(
    request,
    [this](rclcpp::Client<cs625_kinematics::srv::SolveIKAll>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[IK] 响应为空"));
          return;
        }

        if (!response->success) {
          appendLog(QString::fromUtf8("[IK] 求解失败"));
          return;
        }

        all_solutions_ = response->solutions;

        QMetaObject::invokeMethod(
          this,
          [this]() {
            refreshSolutionTable();
          },
          Qt::QueuedConnection);

        appendLog(QString::fromUtf8("[IK] 求解成功，返回解数=%1")
          .arg(response->solutions.size()));
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[IK] 求解异常：%1").arg(e.what()));
      }
    });
}

void MultiConfigPlanningPanel::onRecommendSolutionClicked()
{
  if (!solution_table_) {
    return;
  }

  if (filtered_solution_indices_.empty()) {
    appendLog(QString::fromUtf8("[IK] 当前筛选结果为空，无法推荐"));
    return;
  }

  if (solution_table_->rowCount() <= 0 || solution_table_->columnCount() <= 0) {
    appendLog(QString::fromUtf8("[IK] 解表为空，无法推荐"));
    return;
  }

  QTableWidgetItem * item = solution_table_->item(0, 0);
  if (!item) {
    appendLog(QString::fromUtf8("[IK] 第 0 行项目不存在，无法自动选中"));
    return;
  }

  solution_table_->setCurrentCell(0, 0);
  solution_table_->selectRow(0);
  solution_table_->scrollToItem(item, QAbstractItemView::PositionAtCenter);

  cs625_kinematics::msg::IkSolution sol;
  int row = -1;
  if (getSelectedSolution(sol, row)) {
    if (has_complete_current_joint_state_) {
      appendLog(QString::fromUtf8(
        "[IK] 已自动选中当前筛选结果中的排序第一解（基于当前关节状态排序，第 %1 行）")
        .arg(row));
    } else {
      appendLog(QString::fromUtf8(
        "[IK] 已自动选中当前筛选结果中的排序第一解（未获得完整当前关节状态，第 %1 行）")
        .arg(row));
    }
  } else {
    appendLog(QString::fromUtf8("[IK] 已执行自动选中操作，但当前仍未检测到有效表格选中项"));
  }
}

void MultiConfigPlanningPanel::onPreviewSelectedSolutionClicked()
{
  cs625_kinematics::msg::IkSolution sol;
  int row = -1;
  if (!getSelectedSolution(sol, row)) {
    appendLog(QString::fromUtf8("[预览解] 请先在表格中选中一个解"));
    return;
  }

  publishSelectedSolutionVisual(sol);

  QStringList joints;
  for (int i = 0; i < 6; ++i) {
    joints << QString("J%1=%2").arg(i + 1).arg(sol.joints[i], 0, 'f', 6);
  }

  appendLog(QString::fromUtf8("[预览解] 已预览表格第 %1 行解：cost=%2")
    .arg(row)
    .arg(sol.cost, 0, 'f', 6));
  appendLog(QString::fromUtf8("[预览解] %1").arg(joints.join(", ")));
}

void MultiConfigPlanningPanel::onSaveSelectedTargetClicked()
{
  if (!has_current_target_pose_) {
    appendLog(QString::fromUtf8("[保存命名目标] 请先读取目标 Pose"));
    return;
  }

  if (!set_named_target_client_) {
    appendLog(QString::fromUtf8("[保存命名目标] set_named_target client 未初始化"));
    return;
  }

  cs625_kinematics::msg::IkSolution sol;
  int row = -1;
  if (!getSelectedSolution(sol, row)) {
    appendLog(QString::fromUtf8("[保存命名目标] 请先在表格中选中一个解"));
    return;
  }

  if (!set_named_target_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[保存命名目标] 服务 /cs625/set_named_target 不可用"));
    return;
  }

  auto request = std::make_shared<cs625_trajectory_tools::srv::SetNamedTarget::Request>();
  request->name = kSelectedTargetName;
  request->pose = current_target_pose_;
  request->joint_state = makeJointStateFromSolution(sol);

  appendLog(QString::fromUtf8("[保存命名目标] 正在保存为 %1").arg(kSelectedTargetName));

  set_named_target_client_->async_send_request(
    request,
    [this](rclcpp::Client<cs625_trajectory_tools::srv::SetNamedTarget>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[保存命名目标] 响应为空"));
          return;
        }

        appendLog(QString::fromUtf8("[保存命名目标] success=%1 message=%2")
          .arg(response->success ? "true" : "false")
          .arg(QString::fromStdString(response->message)));

        if (response->success) {
          QMetaObject::invokeMethod(
            this,
            [this]() {
              onRefreshSavedTargetClicked();
            },
            Qt::QueuedConnection);
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[保存命名目标] 异常：%1").arg(e.what()));
      }
    });
}

void MultiConfigPlanningPanel::onPreviewPathClicked()
{
  if (!has_current_target_pose_) {
    appendLog(QString::fromUtf8("[路径预览] 请先读取目标 Pose"));
    return;
  }

  if (!plan_to_target_client_) {
    appendLog(QString::fromUtf8("[路径预览] plan_to_target client 未初始化"));
    return;
  }

  cs625_kinematics::msg::IkSolution sol;
  int row = -1;
  if (!getSelectedSolution(sol, row)) {
    appendLog(QString::fromUtf8("[路径预览] 请先在表格中选中一个解"));
    return;
  }

  if (!plan_to_target_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[路径预览] 服务 /cs625/plan_to_target_srv 不可用"));
    return;
  }

  auto request = std::make_shared<cs625_trajectory_tools::srv::PlanToTarget::Request>();
  request->target_pose = current_target_pose_;
  request->target_joint_state = makeJointStateFromSolution(sol);
  request->use_joint_target = true;

  appendLog(QString::fromUtf8(
    "[路径预览] 正在调用 PlanToTarget（use_joint_target=true），仅规划不执行"));

  plan_to_target_client_->async_send_request(
    request,
    [this](rclcpp::Client<cs625_trajectory_tools::srv::PlanToTarget>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[路径预览] 响应为空"));
          return;
        }

        if (!response->success) {
          appendLog(QString::fromUtf8("[路径预览] 规划失败：%1")
            .arg(QString::fromStdString(response->message)));
          return;
        }

        appendLog(QString::fromUtf8("[路径预览] 规划成功，planning_time=%1 s")
          .arg(response->planning_time, 0, 'f', 4));
        appendLog(QString::fromUtf8("[路径预览] 等待后端发布 /cs625/planned_tcp_path"));
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[路径预览] 异常：%1").arg(e.what()));
      }
    });
}

void MultiConfigPlanningPanel::onExecuteLastPlanClicked()
{
  if (!execute_last_plan_client_) {
    appendLog(QString::fromUtf8("[执行] execute_last_plan client 未初始化"));
    return;
  }

  if (!execute_last_plan_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[执行] 服务 /cs625/execute_last_plan 不可用"));
    return;
  }

  auto request =
    std::make_shared<cs625_trajectory_tools::srv::ExecuteLastPlan::Request>();

  appendLog(QString::fromUtf8("[执行] 正在执行最近一次规划"));

  execute_last_plan_client_->async_send_request(
    request,
    [this](rclcpp::Client<cs625_trajectory_tools::srv::ExecuteLastPlan>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[执行] 响应为空"));
          return;
        }

        appendLog(QString::fromUtf8("[执行] success=%1 message=%2")
          .arg(response->success ? "true" : "false")
          .arg(QString::fromStdString(response->message)));
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[执行] 异常：%1").arg(e.what()));
      }
    });
}

void MultiConfigPlanningPanel::onRefreshSavedTargetClicked()
{
  if (!get_named_target_client_) {
    appendLog(QString::fromUtf8("[读取命名目标] get_named_target client 未初始化"));
    return;
  }

  if (!get_named_target_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("[读取命名目标] 服务 /cs625/get_named_target 不可用"));

    has_saved_target_ = false;
    QMetaObject::invokeMethod(
      this,
      [this]() {
        updateSavedTargetLabels();
      },
      Qt::QueuedConnection);

    return;
  }

  auto request = std::make_shared<cs625_trajectory_tools::srv::GetNamedTarget::Request>();
  request->name = kSelectedTargetName;

  appendLog(QString::fromUtf8("[读取命名目标] 正在读取 %1").arg(kSelectedTargetName));

  get_named_target_client_->async_send_request(
    request,
    [this](rclcpp::Client<cs625_trajectory_tools::srv::GetNamedTarget>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (!response) {
          appendLog(QString::fromUtf8("[读取命名目标] 响应为空"));

          has_saved_target_ = false;
          QMetaObject::invokeMethod(
            this,
            [this]() {
              updateSavedTargetLabels();
            },
            Qt::QueuedConnection);
          return;
        }

        if (!response->success) {
          appendLog(QString::fromUtf8("[读取命名目标] 失败：%1")
            .arg(QString::fromStdString(response->message)));

          has_saved_target_ = false;
          QMetaObject::invokeMethod(
            this,
            [this]() {
              updateSavedTargetLabels();
            },
            Qt::QueuedConnection);
          return;
        }

        saved_target_pose_ = response->pose;
        saved_target_joint_state_ = response->joint_state;
        has_saved_target_ = true;

        QMetaObject::invokeMethod(
          this,
          [this]() {
            updateSavedTargetLabels();
          },
          Qt::QueuedConnection);

        appendLog(QString::fromUtf8("[读取命名目标] 成功读取 %1").arg(kSelectedTargetName));
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("[读取命名目标] 异常：%1").arg(e.what()));

        has_saved_target_ = false;
        QMetaObject::invokeMethod(
          this,
          [this]() {
            updateSavedTargetLabels();
          },
          Qt::QueuedConnection);
      }
    });
}
void MultiConfigPlanningPanel::onFilterChanged(int value)
{
  (void)value;
  refreshSolutionTable();
}

void MultiConfigPlanningPanel::onSolutionSelectionChanged()
{
  cs625_kinematics::msg::IkSolution sol;
  int row = -1;
  if (!getSelectedSolution(sol, row)) {
    return;
  }

  appendLog(QString::fromUtf8(
    "[IK] 当前选中第 %1 行：shoulder=%2 elbow=%3 wrist=%4 cost=%5")
    .arg(row)
    .arg(shoulderToQString(sol.shoulder))
    .arg(elbowToQString(sol.elbow))
    .arg(wristToQString(sol.wrist))
    .arg(sol.cost, 0, 'f', 6));
}

void MultiConfigPlanningPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void MultiConfigPlanningPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace elite_dashboard_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  elite_dashboard_rviz_plugin::MultiConfigPlanningPanel,
  rviz_common::Panel)
