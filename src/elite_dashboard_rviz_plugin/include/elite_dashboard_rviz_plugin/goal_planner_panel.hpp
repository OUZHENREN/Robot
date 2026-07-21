#pragma once

#include <memory>

#include <QWidget>
#include <QPushButton>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QLabel>
#include <QSlider>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "cs625_trajectory_tools/srv/plan_to_frame.hpp"
#include "cs625_trajectory_tools/srv/execute_last_plan.hpp"
#include "cs625_trajectory_tools/srv/set_named_pose.hpp"
#include "cs625_trajectory_tools/srv/set_named_pose_from_current.hpp"

namespace elite_dashboard_rviz_plugin
{

class GoalPlannerPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit GoalPlannerPanel(QWidget * parent = nullptr);
  ~GoalPlannerPanel() override;

  void onInitialize() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onPlanClicked();
  void onExecuteClicked();
  void onSetBoxRoughCaptureFromCurrentClicked();
  void onSetSlotRoughCaptureFromCurrentClicked();
  
  void onSetHomeFromCurrentClicked();
  void onSetBoxPlaceFromCurrentClicked();

  void onUpdateTransitClicked();
  void onTransitSliderValueChanged(int value);

  void onUpdateRetreatClicked();
  void onRetreatSliderValueChanged(int value);

private:
  void initRosNode();
  void initUI();
  void appendLog(const QString & text);

  bool computeTransitPose(
    geometry_msgs::msg::PoseStamped & transit_pose,
    QString & error_text);

  bool computeRetreatPose(
    geometry_msgs::msg::PoseStamped & retreat_pose,
    QString & error_text);

  void updateTransitValueLabels();
  void updateRetreatValueLabels();

  double transitXOffsetMeters() const;
  double transitNegativeZOffsetMeters() const;
  double transitYRotationDegrees() const;

  double retreatXOffsetMeters() const;
  double retreatNegativeZOffsetMeters() const;
  double retreatYRotationDegrees() const;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<cs625_trajectory_tools::srv::PlanToFrame>::SharedPtr plan_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::ExecuteLastPlan>::SharedPtr execute_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::SetNamedPoseFromCurrent>::SharedPtr
    set_named_pose_from_current_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::SetNamedPose>::SharedPtr
    set_named_pose_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  QComboBox * frame_combo_{nullptr};
  QPushButton * plan_button_{nullptr};
  QPushButton * execute_button_{nullptr};
  QPushButton * set_box_rough_capture_button_{nullptr};
  QPushButton * set_slot_rough_capture_button_{nullptr};
  
  QPushButton * set_home_button_{nullptr};
  QPushButton * set_box_place_button_{nullptr};

  QPushButton * update_transit_button_{nullptr};
  QPushButton * update_retreat_button_{nullptr};

  QSlider * transit_x_slider_{nullptr};
  QSlider * transit_neg_z_slider_{nullptr};
  QSlider * transit_y_rot_slider_{nullptr};

  QLabel * transit_x_value_label_{nullptr};
  QLabel * transit_neg_z_value_label_{nullptr};
  QLabel * transit_y_rot_value_label_{nullptr};

  QSlider * retreat_x_slider_{nullptr};
  QSlider * retreat_neg_z_slider_{nullptr};
  QSlider * retreat_y_rot_slider_{nullptr};

  QLabel * retreat_x_value_label_{nullptr};
  QLabel * retreat_neg_z_value_label_{nullptr};
  QLabel * retreat_y_rot_value_label_{nullptr};

  QPlainTextEdit * log_text_{nullptr};
};

}  // namespace elite_dashboard_rviz_plugin
