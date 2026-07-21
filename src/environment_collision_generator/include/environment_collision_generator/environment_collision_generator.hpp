#ifndef ENVIRONMENT_COLLISION_GENERATOR__ENVIRONMENT_COLLISION_GENERATOR_HPP_
#define ENVIRONMENT_COLLISION_GENERATOR__ENVIRONMENT_COLLISION_GENERATOR_HPP_

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class EnvironmentCollisionGenerator : public rclcpp::Node
{
public:
  EnvironmentCollisionGenerator();

private:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  struct AxisAlignedBox
  {
    double min_x{0.0};
    double min_y{0.0};
    double min_z{0.0};
    double max_x{0.0};
    double max_y{0.0};
    double max_z{0.0};

    bool valid{false};
  };

  struct VoxelAccumulator
  {
    double sum_x{0.0};
    double sum_y{0.0};
    double sum_z{0.0};
    std::size_t count{0};
  };

  void declare_and_load_parameters();
  void sanitize_parameters();
  void log_configuration() const;

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void slot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void box_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void handle_rebuild_environment_collision(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool rebuild_environment_collision(std::string & message);

  bool transform_cloud_to_world(
    const sensor_msgs::msg::PointCloud2 & cloud_msg,
    CloudT::Ptr & world_cloud,
    std::string & message) const;

  bool load_ascii_stl_vertices(
    const std::string & path,
    std::vector<Eigen::Vector3d> & vertices,
    std::string & message) const;

  AxisAlignedBox compute_world_aabb(
    const std::vector<Eigen::Vector3d> & local_vertices,
    const geometry_msgs::msg::PoseStamped & pose_msg) const;

  bool point_inside_aabb(const PointT & point, const AxisAlignedBox & box) const;

  CloudT::Ptr subtract_known_regions(
    const CloudT::Ptr & input_cloud,
    const std::vector<AxisAlignedBox> & subtraction_boxes,
    CloudT::Ptr & removed_cloud,
    std::int64_t & removed_points) const;

  std::vector<moveit_msgs::msg::CollisionObject> build_environment_collision_objects(
    const CloudT::Ptr & remaining_cloud,
    std::size_t & generated_box_count,
    std::size_t & truncated_box_count) const;

  moveit_msgs::msg::CollisionObject make_remove_collision_object(const std::string & object_id) const;

  moveit_msgs::msg::CollisionObject make_box_collision_object(
    const std::string & object_id,
    double center_x,
    double center_y,
    double center_z,
    double size_x,
    double size_y,
    double size_z) const;

  void publish_planning_scene_diff(
    const std::vector<moveit_msgs::msg::CollisionObject> & world_objects) const;

  void publish_debug_cloud(
    const CloudT::Ptr & cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const std::string & frame_id) const;

  bool lookup_pose_from_tf(
    const std::string & source_frame,
    geometry_msgs::msg::PoseStamped & pose_msg,
    std::string & error_message) const;

  std::string pose_to_string(const geometry_msgs::msg::PoseStamped & pose_msg) const;

  std::string input_cloud_topic_;
  std::string slot_pose_topic_;
  std::string box_pose_topic_;
  std::string planning_scene_topic_;
  std::string world_frame_;

  std::string slot_mesh_path_;
  std::string box_mesh_path_;

  double slot_mesh_scale_x_;
  double slot_mesh_scale_y_;
  double slot_mesh_scale_z_;
  double box_mesh_scale_x_;
  double box_mesh_scale_y_;
  double box_mesh_scale_z_;

  bool enable_slot_subtraction_;
  bool enable_box_subtraction_;
  bool enable_tf_fallback_;

  std::string slot_tf_frame_;
  std::string box_tf_frame_;

  double subtraction_padding_x_;
  double subtraction_padding_y_;
  double subtraction_padding_z_;

  double environment_voxel_size_;
  int max_environment_boxes_;
  int min_points_per_voxel_;

  std::string environment_object_prefix_;
  bool publish_remaining_cloud_;
  bool publish_removed_cloud_;

  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_msg_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_slot_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_box_pose_;

  std::vector<Eigen::Vector3d> slot_mesh_vertices_;
  std::vector<Eigen::Vector3d> box_mesh_vertices_;

  std::vector<std::string> current_environment_object_ids_;

  mutable std::mutex data_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slot_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr box_pose_sub_;

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr remaining_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr removed_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rebuild_service_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // ENVIRONMENT_COLLISION_GENERATOR__ENVIRONMENT_COLLISION_GENERATOR_HPP_
