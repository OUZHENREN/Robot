#include "elite_dashboard_rviz_plugin/goal_planner_panel.hpp"

#include <cmath>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QThread>
#include <QMetaObject>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

namespace elite_dashboard_rviz_plugin
{

GoalPlannerPanel::GoalPlannerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  std::cout << "[GoalPlannerPanel] ctor begin" << std::endl;
  initUI();
  std::cout << "[GoalPlannerPanel] ctor end" << std::endl;
}

GoalPlannerPanel::~GoalPlannerPanel()
{
  std::cout << "[GoalPlannerPanel] dtor" << std::endl;
}

void GoalPlannerPanel::onInitialize()
{
  std::cout << "[GoalPlannerPanel] onInitialize begin" << std::endl;
  initRosNode();
  updateTransitValueLabels();
  updateRetreatValueLabels();
  std::cout << "[GoalPlannerPanel] onInitialize end" << std::endl;
}

void GoalPlannerPanel::initRosNode()
{
  std::cout << "[GoalPlannerPanel] initRosNode: enter" << std::endl;

  auto context = getDisplayContext();
  if (!context) {
    std::cout << "[GoalPlannerPanel] initRosNode: context is null" << std::endl;
  } else {
    std::cout << "[GoalPlannerPanel] initRosNode: got context" << std::endl;
    auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
    if (!ros_node_abstraction) {
      std::cout << "[GoalPlannerPanel] initRosNode: ros_node_abstraction is null" << std::endl;
    } else {
      std::cout << "[GoalPlannerPanel] initRosNode: got ros_node_abstraction" << std::endl;
      node_ = ros_node_abstraction->get_raw_node();
    }
  }

  if (!node_) {
    std::cout << "[GoalPlannerPanel] initRosNode: node_ is null, creating own node" << std::endl;
    node_ = rclcpp::Node::make_shared("goal_planner_panel_node_simple");
  } else {
    std::cout << "[GoalPlannerPanel] initRosNode: using RViz node" << std::endl;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::cout << "[GoalPlannerPanel] initRosNode: creating clients..." << std::endl;

  plan_client_ =
    node_->create_client<cs625_trajectory_tools::srv::PlanToFrame>(
      "/cs625/plan_to_frame");

  execute_client_ =
    node_->create_client<cs625_trajectory_tools::srv::ExecuteLastPlan>(
      "/cs625/execute_last_plan");

  set_named_pose_from_current_client_ =
    node_->create_client<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent>(
      "/cs625/set_home_from_current");

  set_named_pose_client_ =
    node_->create_client<cs625_trajectory_tools::srv::SetNamedPose>(
      "/cs625/set_named_pose");

  std::cout << "[GoalPlannerPanel] initRosNode: done" << std::endl;
}

void GoalPlannerPanel::initUI()
{
  auto * main_layout = new QVBoxLayout;

  auto * frame_layout = new QHBoxLayout;
  frame_layout->addWidget(new QLabel(QString::fromUtf8("目标 TF frame：")));

  frame_combo_ = new QComboBox;

  frame_combo_->addItem("box_rough_capture_tcp");
  frame_combo_->addItem("slot_rough_capture_tcp");

  frame_combo_->addItem("slot_origin");
  frame_combo_->addItem("slot_precision_view_tcp");
  frame_combo_->addItem("slot_insert_tcp");
  frame_combo_->addItem("slot_pre_insert_rotated_tcp");
  frame_combo_->addItem("slot_pre_insert_tcp");
  frame_combo_->addItem("transit_tcp");
  frame_combo_->addItem("retreat_tcp");

  frame_combo_->addItem("box_origin");
  frame_combo_->addItem("box_precision_view_tcp");
  frame_combo_->addItem("box_grasp_tcp");
  frame_combo_->addItem("box_pre_grasp_tcp");

  frame_combo_->addItem("box_place_tcp");
  frame_combo_->addItem("home_tcp");

  frame_layout->addWidget(frame_combo_);
  main_layout->addLayout(frame_layout);

  auto * button_grid_layout = new QGridLayout;

  plan_button_ = new QPushButton(QString::fromUtf8("规划到该 frame"));
  execute_button_ = new QPushButton(QString::fromUtf8("执行规划"));
  set_box_rough_capture_button_ =
    new QPushButton(QString::fromUtf8("记录当前为 box_rough_capture_tcp"));
  set_slot_rough_capture_button_ =
    new QPushButton(QString::fromUtf8("记录当前为 slot_rough_capture_tcp"));
  set_home_button_ =
    new QPushButton(QString::fromUtf8("记录当前为 home_tcp"));
  set_box_place_button_ =
    new QPushButton(QString::fromUtf8("记录当前为 box_place_tcp"));

  button_grid_layout->addWidget(plan_button_, 0, 0);
  button_grid_layout->addWidget(execute_button_, 0, 1);
  button_grid_layout->addWidget(set_box_rough_capture_button_, 1, 0);
  button_grid_layout->addWidget(set_slot_rough_capture_button_, 1, 1);
  button_grid_layout->addWidget(set_home_button_, 2, 0);
  button_grid_layout->addWidget(set_box_place_button_, 2, 1);

  main_layout->addLayout(button_grid_layout);

  auto * transit_group = new QGroupBox(QString::fromUtf8("transit_tcp 调节"));
  auto * transit_layout = new QGridLayout;

  transit_x_slider_ = new QSlider(Qt::Horizontal);
  transit_neg_z_slider_ = new QSlider(Qt::Horizontal);
  transit_y_rot_slider_ = new QSlider(Qt::Horizontal);

  transit_x_slider_->setRange(0, 300);
  transit_neg_z_slider_->setRange(0, 300);
  transit_y_rot_slider_->setRange(-900, 900);

  transit_x_slider_->setValue(80);
  transit_neg_z_slider_->setValue(60);
  transit_y_rot_slider_->setValue(0);

  transit_x_value_label_ = new QLabel;
  transit_neg_z_value_label_ = new QLabel;
  transit_y_rot_value_label_ = new QLabel;

  update_transit_button_ = new QPushButton(QString::fromUtf8("更新 transit_tcp"));

  transit_layout->addWidget(
    new QLabel(QString::fromUtf8("沿 slot_pre_insert_tcp 局部 X 正方向平移：")), 0, 0);
  transit_layout->addWidget(transit_x_slider_, 0, 1);
  transit_layout->addWidget(transit_x_value_label_, 0, 2);

  transit_layout->addWidget(
    new QLabel(QString::fromUtf8("沿 slot_pre_insert_tcp 局部 Z 负方向平移：")), 1, 0);
  transit_layout->addWidget(transit_neg_z_slider_, 1, 1);
  transit_layout->addWidget(transit_neg_z_value_label_, 1, 2);

  transit_layout->addWidget(
    new QLabel(QString::fromUtf8("绕 slot_pre_insert_tcp 局部 Y 旋转：")), 2, 0);
  transit_layout->addWidget(transit_y_rot_slider_, 2, 1);
  transit_layout->addWidget(transit_y_rot_value_label_, 2, 2);

  transit_layout->addWidget(update_transit_button_, 3, 0, 1, 3);

  transit_group->setLayout(transit_layout);
  main_layout->addWidget(transit_group);

  auto * retreat_group = new QGroupBox(QString::fromUtf8("retreat_tcp 调节"));
  auto * retreat_layout = new QGridLayout;

  retreat_x_slider_ = new QSlider(Qt::Horizontal);
  retreat_neg_z_slider_ = new QSlider(Qt::Horizontal);
  retreat_y_rot_slider_ = new QSlider(Qt::Horizontal);

  retreat_x_slider_->setRange(0, 300);
  retreat_neg_z_slider_->setRange(0, 300);
  retreat_y_rot_slider_->setRange(-900, 900);

  retreat_x_slider_->setValue(80);
  retreat_neg_z_slider_->setValue(60);
  retreat_y_rot_slider_->setValue(0);

  retreat_x_value_label_ = new QLabel;
  retreat_neg_z_value_label_ = new QLabel;
  retreat_y_rot_value_label_ = new QLabel;

  update_retreat_button_ = new QPushButton(QString::fromUtf8("更新 retreat_tcp"));

  retreat_layout->addWidget(
    new QLabel(QString::fromUtf8("沿当前 TCP 局部 X 正方向平移：")), 0, 0);
  retreat_layout->addWidget(retreat_x_slider_, 0, 1);
  retreat_layout->addWidget(retreat_x_value_label_, 0, 2);

  retreat_layout->addWidget(
    new QLabel(QString::fromUtf8("沿当前 TCP 局部 Z 负方向平移：")), 1, 0);
  retreat_layout->addWidget(retreat_neg_z_slider_, 1, 1);
  retreat_layout->addWidget(retreat_neg_z_value_label_, 1, 2);

  retreat_layout->addWidget(
    new QLabel(QString::fromUtf8("绕当前 TCP 局部 Y 旋转：")), 2, 0);
  retreat_layout->addWidget(retreat_y_rot_slider_, 2, 1);
  retreat_layout->addWidget(retreat_y_rot_value_label_, 2, 2);

  retreat_layout->addWidget(update_retreat_button_, 3, 0, 1, 3);

  retreat_group->setLayout(retreat_layout);
  main_layout->addWidget(retreat_group);

  log_text_ = new QPlainTextEdit;
  log_text_->setReadOnly(true);

  main_layout->addWidget(new QLabel(QString::fromUtf8("规划日志：")));
  main_layout->addWidget(log_text_);

  setLayout(main_layout);

  connect(plan_button_, &QPushButton::clicked,
          this, &GoalPlannerPanel::onPlanClicked);

  connect(execute_button_, &QPushButton::clicked,
          this, &GoalPlannerPanel::onExecuteClicked);

  connect(set_box_rough_capture_button_, &QPushButton::clicked,
          this, &GoalPlannerPanel::onSetBoxRoughCaptureFromCurrentClicked);

  connect(set_slot_rough_capture_button_, &QPushButton::clicked,
          this, &GoalPlannerPanel::onSetSlotRoughCaptureFromCurrentClicked);
          
  connect(set_home_button_, &QPushButton::clicked,
          this, &GoalPlannerPanel::onSetHomeFromCurrentClicked);

  connect(set_box_place_button_, &QPushButton::clicked,
          this, &GoalPlannerPanel::onSetBoxPlaceFromCurrentClicked);

  connect(update_transit_button_, &QPushButton::clicked,
          this, &GoalPlannerPanel::onUpdateTransitClicked);

  connect(transit_x_slider_, &QSlider::valueChanged,
          this, &GoalPlannerPanel::onTransitSliderValueChanged);
  connect(transit_neg_z_slider_, &QSlider::valueChanged,
          this, &GoalPlannerPanel::onTransitSliderValueChanged);
  connect(transit_y_rot_slider_, &QSlider::valueChanged,
          this, &GoalPlannerPanel::onTransitSliderValueChanged);

  connect(update_retreat_button_, &QPushButton::clicked,
          this, &GoalPlannerPanel::onUpdateRetreatClicked);

  connect(retreat_x_slider_, &QSlider::valueChanged,
          this, &GoalPlannerPanel::onRetreatSliderValueChanged);
  connect(retreat_neg_z_slider_, &QSlider::valueChanged,
          this, &GoalPlannerPanel::onRetreatSliderValueChanged);
  connect(retreat_y_rot_slider_, &QSlider::valueChanged,
          this, &GoalPlannerPanel::onRetreatSliderValueChanged);

  updateTransitValueLabels();
  updateRetreatValueLabels();
}

double GoalPlannerPanel::transitXOffsetMeters() const
{
  if (!transit_x_slider_) {
    return 0.0;
  }
  return static_cast<double>(transit_x_slider_->value()) / 1000.0;
}

double GoalPlannerPanel::transitNegativeZOffsetMeters() const
{
  if (!transit_neg_z_slider_) {
    return 0.0;
  }
  return static_cast<double>(transit_neg_z_slider_->value()) / 1000.0;
}

double GoalPlannerPanel::transitYRotationDegrees() const
{
  if (!transit_y_rot_slider_) {
    return 0.0;
  }
  return static_cast<double>(transit_y_rot_slider_->value()) / 10.0;
}

double GoalPlannerPanel::retreatXOffsetMeters() const
{
  if (!retreat_x_slider_) {
    return 0.0;
  }
  return static_cast<double>(retreat_x_slider_->value()) / 1000.0;
}

double GoalPlannerPanel::retreatNegativeZOffsetMeters() const
{
  if (!retreat_neg_z_slider_) {
    return 0.0;
  }
  return static_cast<double>(retreat_neg_z_slider_->value()) / 1000.0;
}

double GoalPlannerPanel::retreatYRotationDegrees() const
{
  if (!retreat_y_rot_slider_) {
    return 0.0;
  }
  return static_cast<double>(retreat_y_rot_slider_->value()) / 10.0;
}

void GoalPlannerPanel::updateTransitValueLabels()
{
  if (transit_x_value_label_) {
    transit_x_value_label_->setText(
      QString("%1 m").arg(transitXOffsetMeters(), 0, 'f', 3));
  }

  if (transit_neg_z_value_label_) {
    transit_neg_z_value_label_->setText(
      QString("-%1 m").arg(transitNegativeZOffsetMeters(), 0, 'f', 3));
  }

  if (transit_y_rot_value_label_) {
    transit_y_rot_value_label_->setText(
      QString("%1 deg").arg(transitYRotationDegrees(), 0, 'f', 1));
  }
}

void GoalPlannerPanel::updateRetreatValueLabels()
{
  if (retreat_x_value_label_) {
    retreat_x_value_label_->setText(
      QString("%1 m").arg(retreatXOffsetMeters(), 0, 'f', 3));
  }

  if (retreat_neg_z_value_label_) {
    retreat_neg_z_value_label_->setText(
      QString("-%1 m").arg(retreatNegativeZOffsetMeters(), 0, 'f', 3));
  }

  if (retreat_y_rot_value_label_) {
    retreat_y_rot_value_label_->setText(
      QString("%1 deg").arg(retreatYRotationDegrees(), 0, 'f', 1));
  }
}

void GoalPlannerPanel::onTransitSliderValueChanged(int value)
{
  (void)value;
  updateTransitValueLabels();
}

void GoalPlannerPanel::onRetreatSliderValueChanged(int value)
{
  (void)value;
  updateRetreatValueLabels();
}

bool GoalPlannerPanel::computeTransitPose(
  geometry_msgs::msg::PoseStamped & transit_pose,
  QString & error_text)
{
  if (!node_) {
    error_text = QString::fromUtf8("节点未初始化");
    return false;
  }

  if (!tf_buffer_) {
    error_text = QString::fromUtf8("TF buffer 未初始化");
    return false;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(
      "base_link",
      "slot_pre_insert_tcp",
      tf2::TimePointZero,
      tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException & ex) {
    error_text = QString::fromUtf8("查询 slot_pre_insert_tcp TF 失败：%1").arg(ex.what());
    return false;
  }

  tf2::Quaternion base_q;
  tf2::fromMsg(tf_msg.transform.rotation, base_q);

  tf2::Matrix3x3 basis(base_q);
  tf2::Vector3 x_axis = basis.getColumn(0);
  tf2::Vector3 y_axis = basis.getColumn(1);
  tf2::Vector3 z_axis = basis.getColumn(2);

  const double x_offset = transitXOffsetMeters();
  const double neg_z_offset = transitNegativeZOffsetMeters();
  const double y_rot_deg = transitYRotationDegrees();
  const double y_rot_rad = y_rot_deg * M_PI / 180.0;

  tf2::Vector3 base_origin(
    tf_msg.transform.translation.x,
    tf_msg.transform.translation.y,
    tf_msg.transform.translation.z);

  tf2::Vector3 transit_origin =
    base_origin +
    x_axis * x_offset -
    z_axis * neg_z_offset;

  tf2::Quaternion delta_q;
  delta_q.setRPY(0.0, y_rot_rad, 0.0);

  tf2::Quaternion final_q = base_q * delta_q;
  final_q.normalize();

  transit_pose.header.stamp = node_->now();
  transit_pose.header.frame_id = "base_link";
  transit_pose.pose.position.x = transit_origin.x();
  transit_pose.pose.position.y = transit_origin.y();
  transit_pose.pose.position.z = transit_origin.z();
  transit_pose.pose.orientation = tf2::toMsg(final_q);

  (void)y_axis;
  return true;
}

bool GoalPlannerPanel::computeRetreatPose(
  geometry_msgs::msg::PoseStamped & retreat_pose,
  QString & error_text)
{
  if (!node_) {
    error_text = QString::fromUtf8("节点未初始化");
    return false;
  }

  if (!tf_buffer_) {
    error_text = QString::fromUtf8("TF buffer 未初始化");
    return false;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
	tf_msg = tf_buffer_->lookupTransform(
	  "base_link",
	  "my_end_effector_link",
	  tf2::TimePointZero,
	  tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException & ex) {
	error_text = QString::fromUtf8("查询当前 TCP（my_end_effector_link）TF 失败：%1").arg(ex.what());
    return false;
  }

  tf2::Quaternion base_q;
  tf2::fromMsg(tf_msg.transform.rotation, base_q);

  tf2::Matrix3x3 basis(base_q);
  tf2::Vector3 x_axis = basis.getColumn(0);
  tf2::Vector3 y_axis = basis.getColumn(1);
  tf2::Vector3 z_axis = basis.getColumn(2);

  const double x_offset = retreatXOffsetMeters();
  const double neg_z_offset = retreatNegativeZOffsetMeters();
  const double y_rot_deg = retreatYRotationDegrees();
  const double y_rot_rad = y_rot_deg * M_PI / 180.0;

  tf2::Vector3 base_origin(
    tf_msg.transform.translation.x,
    tf_msg.transform.translation.y,
    tf_msg.transform.translation.z);

  tf2::Vector3 retreat_origin =
    base_origin +
    x_axis * x_offset -
    z_axis * neg_z_offset;

  tf2::Quaternion delta_q;
  delta_q.setRPY(0.0, y_rot_rad, 0.0);

  tf2::Quaternion final_q = base_q * delta_q;
  final_q.normalize();

  retreat_pose.header.stamp = node_->now();
  retreat_pose.header.frame_id = "base_link";
  retreat_pose.pose.position.x = retreat_origin.x();
  retreat_pose.pose.position.y = retreat_origin.y();
  retreat_pose.pose.position.z = retreat_origin.z();
  retreat_pose.pose.orientation = tf2::toMsg(final_q);

  (void)y_axis;
  return true;
}

void GoalPlannerPanel::onSetBoxRoughCaptureFromCurrentClicked()
{
  appendLog(QString::fromUtf8("[调试] onSetBoxRoughCaptureFromCurrentClicked 被调用"));

  if (!node_) {
    appendLog(QString::fromUtf8("节点未初始化，无法调用设置命名位姿服务"));
    return;
  }
  if (!set_named_pose_from_current_client_) {
    appendLog(QString::fromUtf8("set_home_from_current 服务 client 未初始化"));
    return;
  }

  appendLog(QString::fromUtf8("准备调用 wait_for_service(set_home_from_current)..."));

  if (!set_named_pose_from_current_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("服务 /cs625/set_home_from_current 不可用"));
    return;
  }

  auto req =
    std::make_shared<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent::Request>();
  req->name = "box_rough_capture_tcp";

  appendLog(QString::fromUtf8("发送设置当前位姿为 '%1' 的请求").arg("box_rough_capture_tcp"));

  set_named_pose_from_current_client_->async_send_request(
    req,
    [this](rclcpp::Client<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(QString::fromUtf8("设置 box_rough_capture_tcp 失败：%1").arg(msg));
        } else {
          appendLog(QString::fromUtf8("设置 box_rough_capture_tcp 成功：%1").arg(msg));
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("set_home_from_current 服务回调异常：") + e.what());
      }
    });
}

void GoalPlannerPanel::onSetSlotRoughCaptureFromCurrentClicked()
{
  appendLog(QString::fromUtf8("[调试] onSetSlotRoughCaptureFromCurrentClicked 被调用"));

  if (!node_) {
    appendLog(QString::fromUtf8("节点未初始化，无法调用设置命名位姿服务"));
    return;
  }
  if (!set_named_pose_from_current_client_) {
    appendLog(QString::fromUtf8("set_home_from_current 服务 client 未初始化"));
    return;
  }

  appendLog(QString::fromUtf8("准备调用 wait_for_service(set_home_from_current)..."));

  if (!set_named_pose_from_current_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("服务 /cs625/set_home_from_current 不可用"));
    return;
  }

  auto req =
    std::make_shared<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent::Request>();
  req->name = "slot_rough_capture_tcp";

  appendLog(QString::fromUtf8("发送设置当前位姿为 '%1' 的请求").arg("slot_rough_capture_tcp"));

  set_named_pose_from_current_client_->async_send_request(
    req,
    [this](rclcpp::Client<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(QString::fromUtf8("设置 slot_rough_capture_tcp 失败：%1").arg(msg));
        } else {
          appendLog(QString::fromUtf8("设置 slot_rough_capture_tcp 成功：%1").arg(msg));
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("set_home_from_current 服务回调异常：") + e.what());
      }
    });
}

void GoalPlannerPanel::onSetHomeFromCurrentClicked()
{
  appendLog(QString::fromUtf8("[调试] onSetHomeFromCurrentClicked 被调用"));

  if (!node_) {
    appendLog(QString::fromUtf8("节点未初始化，无法调用设置命名位姿服务"));
    return;
  }
  if (!set_named_pose_from_current_client_) {
    appendLog(QString::fromUtf8("set_home_from_current 服务 client 未初始化"));
    return;
  }

  appendLog(QString::fromUtf8("准备调用 wait_for_service(set_home_from_current)..."));

  if (!set_named_pose_from_current_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("服务 /cs625/set_home_from_current 不可用"));
    return;
  }

  auto req =
    std::make_shared<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent::Request>();
  req->name = "home_tcp";

  appendLog(QString::fromUtf8("发送设置当前位姿为 '%1' 的请求").arg("home_tcp"));

  set_named_pose_from_current_client_->async_send_request(
    req,
    [this](rclcpp::Client<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(QString::fromUtf8("设置 home_tcp 失败：%1").arg(msg));
        } else {
          appendLog(QString::fromUtf8("设置 home_tcp 成功：%1").arg(msg));
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("set_home_from_current 服务回调异常：") + e.what());
      }
    });
}

void GoalPlannerPanel::onSetBoxPlaceFromCurrentClicked()
{
  appendLog(QString::fromUtf8("[调试] onSetBoxPlaceFromCurrentClicked 被调用"));

  if (!node_) {
    appendLog(QString::fromUtf8("节点未初始化，无法调用设置命名位姿服务"));
    return;
  }
  if (!set_named_pose_from_current_client_) {
    appendLog(QString::fromUtf8("set_home_from_current 服务 client 未初始化"));
    return;
  }

  appendLog(QString::fromUtf8("准备调用 wait_for_service(set_home_from_current)..."));

  if (!set_named_pose_from_current_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("服务 /cs625/set_home_from_current 不可用"));
    return;
  }

  auto req =
    std::make_shared<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent::Request>();
  req->name = "box_place_tcp";

  appendLog(QString::fromUtf8("发送设置当前位姿为 '%1' 的请求").arg("box_place_tcp"));

  set_named_pose_from_current_client_->async_send_request(
    req,
    [this](rclcpp::Client<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(QString::fromUtf8("设置 box_place_tcp 失败：%1").arg(msg));
        } else {
          appendLog(QString::fromUtf8("设置 box_place_tcp 成功：%1").arg(msg));
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("set_home_from_current 服务回调异常：") + e.what());
      }
    });
}

void GoalPlannerPanel::onUpdateTransitClicked()
{
  appendLog(QString::fromUtf8("[调试] onUpdateTransitClicked 被调用"));

  if (!node_) {
    appendLog(QString::fromUtf8("节点未初始化，无法更新 transit_tcp"));
    return;
  }

  if (!set_named_pose_client_) {
    appendLog(QString::fromUtf8("set_named_pose 服务 client 未初始化"));
    return;
  }

  geometry_msgs::msg::PoseStamped transit_pose;
  QString error_text;
  if (!computeTransitPose(transit_pose, error_text)) {
    appendLog(QString::fromUtf8("计算 transit_tcp 失败：%1").arg(error_text));
    return;
  }

  appendLog(
    QString::fromUtf8(
      "已计算 transit_tcp：x_offset=%1 m, -z_offset=%2 m, y_rot=%3 deg")
    .arg(transitXOffsetMeters(), 0, 'f', 3)
    .arg(transitNegativeZOffsetMeters(), 0, 'f', 3)
    .arg(transitYRotationDegrees(), 0, 'f', 1));

  if (!set_named_pose_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("服务 /cs625/set_named_pose 不可用"));
    return;
  }

  auto req =
    std::make_shared<cs625_trajectory_tools::srv::SetNamedPose::Request>();
  req->name = "transit_tcp";
  req->pose = transit_pose;

  set_named_pose_client_->async_send_request(
    req,
    [this](rclcpp::Client<cs625_trajectory_tools::srv::SetNamedPose>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(QString::fromUtf8("更新 transit_tcp 失败：%1").arg(msg));
        } else {
          appendLog(QString::fromUtf8("更新 transit_tcp 成功：%1").arg(msg));
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("set_named_pose 服务回调异常：") + e.what());
      }
    });
}

void GoalPlannerPanel::onUpdateRetreatClicked()
{
  appendLog(QString::fromUtf8("[调试] onUpdateRetreatClicked 被调用"));

  if (!node_) {
    appendLog(QString::fromUtf8("节点未初始化，无法更新 retreat_tcp"));
    return;
  }

  if (!set_named_pose_client_) {
    appendLog(QString::fromUtf8("set_named_pose 服务 client 未初始化"));
    return;
  }

  geometry_msgs::msg::PoseStamped retreat_pose;
  QString error_text;
  if (!computeRetreatPose(retreat_pose, error_text)) {
    appendLog(QString::fromUtf8("计算 retreat_tcp 失败：%1").arg(error_text));
    return;
  }

  appendLog(
    QString::fromUtf8(
      "已基于当前 TCP 计算 retreat_tcp：x_offset=%1 m, -z_offset=%2 m, y_rot=%3 deg")
    .arg(retreatXOffsetMeters(), 0, 'f', 3)
    .arg(retreatNegativeZOffsetMeters(), 0, 'f', 3)
    .arg(retreatYRotationDegrees(), 0, 'f', 1));

  if (!set_named_pose_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("服务 /cs625/set_named_pose 不可用"));
    return;
  }

  auto req =
    std::make_shared<cs625_trajectory_tools::srv::SetNamedPose::Request>();
  req->name = "retreat_tcp";
  req->pose = retreat_pose;

  set_named_pose_client_->async_send_request(
    req,
    [this](rclcpp::Client<cs625_trajectory_tools::srv::SetNamedPose>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(QString::fromUtf8("更新 retreat_tcp 失败：%1").arg(msg));
        } else {
          appendLog(QString::fromUtf8("更新 retreat_tcp 成功：%1").arg(msg));
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("set_named_pose 服务回调异常：") + e.what());
      }
    });
}

void GoalPlannerPanel::appendLog(const QString & text)
{
  if (!log_text_) return;

  if (QThread::currentThread() == this->thread()) {
    log_text_->appendPlainText(text);
  } else {
    QString copy = text;
    QMetaObject::invokeMethod(
      this,
      [this, copy]() {
        if (log_text_) {
          log_text_->appendPlainText(copy);
        }
      },
      Qt::QueuedConnection);
  }
}

void GoalPlannerPanel::onPlanClicked()
{
  appendLog(QString::fromUtf8("[调试] onPlanClicked 被调用"));

  if (!node_) {
    appendLog(QString::fromUtf8("节点未初始化，无法调用规划服务"));
    return;
  }
  if (!plan_client_) {
    appendLog(QString::fromUtf8("规划服务 client 未初始化"));
    return;
  }

  QString frame_q = frame_combo_->currentText().trimmed();
  if (frame_q.isEmpty()) {
    appendLog(QString::fromUtf8("目标 frame 为空，未发送规划请求"));
    return;
  }

  appendLog(QString::fromUtf8("准备调用 wait_for_service(规划)..."));

  if (!plan_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("服务 /cs625/plan_to_frame 不可用"));
    return;
  }

  auto req = std::make_shared<cs625_trajectory_tools::srv::PlanToFrame::Request>();
  req->frame_id = frame_q.toStdString();

  appendLog(QString::fromUtf8("发送规划请求到 frame: %1").arg(frame_q));

  plan_client_->async_send_request(
    req,
    [this, frame_q](rclcpp::Client<cs625_trajectory_tools::srv::PlanToFrame>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(QString::fromUtf8("规划失败（%1）：%2")
                      .arg(frame_q).arg(msg));
        } else {
          appendLog(QString::fromUtf8("规划成功（%1），规划时间 = %2 s")
                      .arg(frame_q).arg(resp->planning_time));
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("规划服务回调异常：") + e.what());
      }
    });
}

void GoalPlannerPanel::onExecuteClicked()
{
  appendLog(QString::fromUtf8("[调试] onExecuteClicked 被调用"));

  if (!node_) {
    appendLog(QString::fromUtf8("节点未初始化，无法调用执行服务"));
    return;
  }
  if (!execute_client_) {
    appendLog(QString::fromUtf8("执行服务 client 未初始化"));
    return;
  }

  appendLog(QString::fromUtf8("准备调用 wait_for_service(执行)..."));

  if (!execute_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("服务 /cs625/execute_last_plan 不可用"));
    return;
  }

  auto req = std::make_shared<cs625_trajectory_tools::srv::ExecuteLastPlan::Request>();

  appendLog(QString::fromUtf8("发送执行最近规划的请求"));

  execute_client_->async_send_request(
    req,
    [this](rclcpp::Client<cs625_trajectory_tools::srv::ExecuteLastPlan>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(QString::fromUtf8("执行失败：%1").arg(msg));
        } else {
          appendLog(QString::fromUtf8("执行成功：%1").arg(msg));
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("执行服务回调异常：") + e.what());
      }
    });
}

void GoalPlannerPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void GoalPlannerPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace elite_dashboard_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  elite_dashboard_rviz_plugin::GoalPlannerPanel,
  rviz_common::Panel)
