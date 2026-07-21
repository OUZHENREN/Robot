#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <QWidget>
#include <QPushButton>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QLabel>
#include <QTableWidget>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cs625_kinematics/msg/ik_solution.hpp"
#include "cs625_kinematics/srv/solve_ik_all.hpp"

#include "cs625_trajectory_tools/srv/get_named_pose.hpp"
#include "cs625_trajectory_tools/srv/get_named_target.hpp"
#include "cs625_trajectory_tools/srv/set_named_target.hpp"
#include "cs625_trajectory_tools/srv/plan_to_target.hpp"
#include "cs625_trajectory_tools/srv/execute_last_plan.hpp"

namespace elite_dashboard_rviz_plugin
{

class MultiConfigPlanningPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MultiConfigPlanningPanel(QWidget * parent = nullptr);
  ~MultiConfigPlanningPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onLoadTargetPoseClicked();
  void onSolveAllIkClicked();
  void onRecommendSolutionClicked();
  void onPreviewSelectedSolutionClicked();
  void onSaveSelectedTargetClicked();
  void onPreviewPathClicked();
  void onExecuteLastPlanClicked();
  void onRefreshSavedTargetClicked();

  void onFilterChanged(int value);
  void onSolutionSelectionChanged();

private:
  void initUI();
  void initRosNode();
  void appendLog(const QString & text);

  void populateTargetCombo();
  void updateCurrentTargetPoseLabels();
  void updateSavedTargetLabels();

  void refreshSolutionTable();
  bool solutionPassesCurrentFilters(const cs625_kinematics::msg::IkSolution & sol) const;
  bool getSelectedSolution(cs625_kinematics::msg::IkSolution & sol, int & filtered_row_index) const;

  sensor_msgs::msg::JointState makeJointStateFromSolution(
    const cs625_kinematics::msg::IkSolution & sol) const;

  void publishSelectedSolutionVisual(
    const cs625_kinematics::msg::IkSolution & sol);

  void publishTargetMarker(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & text);

  void publishPreviewPathMarkers(const nav_msgs::msg::Path & path);
  QString poseToQString(const geometry_msgs::msg::PoseStamped & pose, int index) const;

  QString shoulderToQString(int8_t v) const;
  QString elbowToQString(int8_t v) const;
  QString wristToQString(int8_t v) const;

  bool tryBuildSeedJointState(sensor_msgs::msg::JointState & seed_joint_state) const;
  QString currentSeedToQString() const;

private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Client<cs625_trajectory_tools::srv::GetNamedPose>::SharedPtr
    get_named_pose_client_;
  rclcpp::Client<cs625_kinematics::srv::SolveIKAll>::SharedPtr
    solve_ik_all_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::SetNamedTarget>::SharedPtr
    set_named_target_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::GetNamedTarget>::SharedPtr
    get_named_target_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::PlanToTarget>::SharedPtr
    plan_to_target_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::ExecuteLastPlan>::SharedPtr
    execute_last_plan_client_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    selected_joint_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    selected_target_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    preview_path_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
    preview_path_pose_array_pub_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
    planned_tcp_path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    current_joint_state_sub_;

  geometry_msgs::msg::PoseStamped current_target_pose_;
  bool has_current_target_pose_{false};

  geometry_msgs::msg::PoseStamped saved_target_pose_;
  sensor_msgs::msg::JointState saved_target_joint_state_;
  bool has_saved_target_{false};

  std::array<double, 6> current_joint_positions_{};
  std::array<bool, 6> current_joint_position_valid_{{false, false, false, false, false, false}};
  bool has_complete_current_joint_state_{false};

  std::vector<cs625_kinematics::msg::IkSolution> all_solutions_;
  std::vector<int> filtered_solution_indices_;

  static constexpr const char * kSelectedTargetName = "selected_multi_config_target";

  QComboBox * target_combo_{nullptr};
  QPushButton * load_target_pose_button_{nullptr};

  QLabel * current_pose_frame_value_{nullptr};
  QLabel * current_pose_xyz_value_{nullptr};
  QLabel * current_pose_quat_value_{nullptr};

  QPushButton * solve_all_ik_button_{nullptr};
  QPushButton * recommend_solution_button_{nullptr};

  QComboBox * shoulder_filter_combo_{nullptr};
  QComboBox * elbow_filter_combo_{nullptr};
  QComboBox * wrist_filter_combo_{nullptr};

  QTableWidget * solution_table_{nullptr};

  QPushButton * preview_selected_solution_button_{nullptr};
  QPushButton * save_selected_target_button_{nullptr};
  QPushButton * preview_path_button_{nullptr};
  QPushButton * execute_last_plan_button_{nullptr};
  QPushButton * refresh_saved_target_button_{nullptr};

  QLabel * saved_target_pose_value_{nullptr};
  QLabel * saved_target_joint_value_{nullptr};

  QPlainTextEdit * log_text_{nullptr};
};

}  // namespace elite_dashboard_rviz_plugin
