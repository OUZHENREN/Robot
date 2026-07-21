#include "environment_collision_generator/environment_collision_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

EnvironmentCollisionGenerator::EnvironmentCollisionGenerator()
: Node("environment_collision_generator")
{
  declare_and_load_parameters();
  sanitize_parameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string message;
  bool slot_ok = load_ascii_stl_vertices(slot_mesh_path_, slot_mesh_vertices_, message);
  if (!slot_ok) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load slot mesh vertices: %s", message.c_str());
  }

  bool box_ok = load_ascii_stl_vertices(box_mesh_path_, box_mesh_vertices_, message);
  if (!box_ok) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load box mesh vertices: %s", message.c_str());
  }

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_,
    10,
    std::bind(&EnvironmentCollisionGenerator::cloud_callback, this, std::placeholders::_1));

  slot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    slot_pose_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&EnvironmentCollisionGenerator::slot_pose_callback, this, std::placeholders::_1));

  box_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    box_pose_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&EnvironmentCollisionGenerator::box_pose_callback, this, std::placeholders::_1));

  planning_scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
    planning_scene_topic_,
    rclcpp::QoS(1).transient_local());

  remaining_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/point_cloud_remaining",
    10);

  removed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/point_cloud_removed",
    10);

  status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "~/status",
    10);

  rebuild_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/rebuild_environment_collision",
    std::bind(
      &EnvironmentCollisionGenerator::handle_rebuild_environment_collision,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  log_configuration();
}

void EnvironmentCollisionGenerator::declare_and_load_parameters()
{
  input_cloud_topic_ = this->declare_parameter<std::string>(
    "input_cloud_topic", "/environment/point_cloud");
  slot_pose_topic_ = this->declare_parameter<std::string>(
    "slot_pose_topic", "/slot_target_pose");
  box_pose_topic_ = this->declare_parameter<std::string>(
    "box_pose_topic", "/box_target_pose");
  planning_scene_topic_ = this->declare_parameter<std::string>(
    "planning_scene_topic", "/planning_scene");
  world_frame_ = this->declare_parameter<std::string>(
    "world_frame", "world");

  slot_mesh_path_ = this->declare_parameter<std::string>(
    "slot_mesh_path", "/home/yff/elite_ros_ws/src/tcp_bridge/meshes/slot_collision.STL");
  box_mesh_path_ = this->declare_parameter<std::string>(
    "box_mesh_path", "/home/yff/elite_ros_ws/src/tcp_bridge/meshes/box_collision.STL");

  slot_mesh_scale_x_ = this->declare_parameter<double>("slot_mesh_scale.x", 0.001);
  slot_mesh_scale_y_ = this->declare_parameter<double>("slot_mesh_scale.y", 0.001);
  slot_mesh_scale_z_ = this->declare_parameter<double>("slot_mesh_scale.z", 0.001);
  box_mesh_scale_x_ = this->declare_parameter<double>("box_mesh_scale.x", 0.001);
  box_mesh_scale_y_ = this->declare_parameter<double>("box_mesh_scale.y", 0.001);
  box_mesh_scale_z_ = this->declare_parameter<double>("box_mesh_scale.z", 0.001);

  enable_slot_subtraction_ = this->declare_parameter<bool>("enable_slot_subtraction", true);
  enable_box_subtraction_ = this->declare_parameter<bool>("enable_box_subtraction", true);
  enable_tf_fallback_ = this->declare_parameter<bool>("enable_tf_fallback", true);

  slot_tf_frame_ = this->declare_parameter<std::string>("slot_tf_frame", "slot_origin");
  box_tf_frame_ = this->declare_parameter<std::string>("box_tf_frame", "box_origin");

  subtraction_padding_x_ = this->declare_parameter<double>("subtraction_padding_x", 0.015);
  subtraction_padding_y_ = this->declare_parameter<double>("subtraction_padding_y", 0.015);
  subtraction_padding_z_ = this->declare_parameter<double>("subtraction_padding_z", 0.015);

  environment_voxel_size_ = this->declare_parameter<double>("environment_voxel_size", 0.05);
  max_environment_boxes_ = this->declare_parameter<int>("max_environment_boxes", 500);
  min_points_per_voxel_ = this->declare_parameter<int>("min_points_per_voxel", 1);

  environment_object_prefix_ = this->declare_parameter<std::string>(
    "environment_object_prefix", "env_voxel_");
  publish_remaining_cloud_ = this->declare_parameter<bool>("publish_remaining_cloud", true);
  publish_removed_cloud_ = this->declare_parameter<bool>("publish_removed_cloud", true);
}

void EnvironmentCollisionGenerator::sanitize_parameters()
{
  if (environment_voxel_size_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Invalid environment_voxel_size, fallback to 0.05.");
    environment_voxel_size_ = 0.05;
  }

  if (max_environment_boxes_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "Invalid max_environment_boxes, fallback to 500.");
    max_environment_boxes_ = 500;
  }

  if (min_points_per_voxel_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "Invalid min_points_per_voxel, fallback to 1.");
    min_points_per_voxel_ = 1;
  }

  if (subtraction_padding_x_ < 0.0) {
    subtraction_padding_x_ = 0.0;
  }
  if (subtraction_padding_y_ < 0.0) {
    subtraction_padding_y_ = 0.0;
  }
  if (subtraction_padding_z_ < 0.0) {
    subtraction_padding_z_ = 0.0;
  }
}

void EnvironmentCollisionGenerator::log_configuration() const
{
  RCLCPP_INFO(
    this->get_logger(),
    "Configuration: input_cloud_topic='%s', slot_pose_topic='%s', box_pose_topic='%s', planning_scene_topic='%s', world_frame='%s', voxel_size=%.3f, max_environment_boxes=%d, enable_tf_fallback=%s, slot_tf_frame='%s', box_tf_frame='%s'",
    input_cloud_topic_.c_str(),
    slot_pose_topic_.c_str(),
    box_pose_topic_.c_str(),
    planning_scene_topic_.c_str(),
    world_frame_.c_str(),
    environment_voxel_size_,
    max_environment_boxes_,
    enable_tf_fallback_ ? "true" : "false",
    slot_tf_frame_.c_str(),
    box_tf_frame_.c_str());
}

void EnvironmentCollisionGenerator::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_cloud_msg_ = msg;
}

void EnvironmentCollisionGenerator::slot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_slot_pose_ = msg;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Received slot pose from topic '%s': %s",
    slot_pose_topic_.c_str(),
    pose_to_string(*msg).c_str());
}

void EnvironmentCollisionGenerator::box_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_box_pose_ = msg;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Received box pose from topic '%s': %s",
    box_pose_topic_.c_str(),
    pose_to_string(*msg).c_str());
}

void EnvironmentCollisionGenerator::handle_rebuild_environment_collision(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::string message;
  const bool ok = rebuild_environment_collision(message);
  response->success = ok;
  response->message = message;
}

bool EnvironmentCollisionGenerator::transform_cloud_to_world(
  const sensor_msgs::msg::PointCloud2 & cloud_msg,
  CloudT::Ptr & world_cloud,
  std::string & message) const
{
  CloudT::Ptr input_cloud = std::make_shared<CloudT>();
  pcl::fromROSMsg(cloud_msg, *input_cloud);

  if (cloud_msg.header.frame_id.empty() || cloud_msg.header.frame_id == world_frame_) {
    world_cloud = input_cloud;
    return true;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(
      world_frame_,
      cloud_msg.header.frame_id,
      tf2::TimePointZero);
  } catch (const std::exception & e) {
    message = std::string("Failed to lookup transform from '") +
      cloud_msg.header.frame_id + "' to '" + world_frame_ + "': " + e.what();
    return false;
  }

  Eigen::Quaterniond q(
    tf_msg.transform.rotation.w,
    tf_msg.transform.rotation.x,
    tf_msg.transform.rotation.y,
    tf_msg.transform.rotation.z);

  if (q.norm() == 0.0) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }

  Eigen::Isometry3d tf_eigen = Eigen::Isometry3d::Identity();
  tf_eigen.linear() = q.toRotationMatrix();
  tf_eigen.translation() = Eigen::Vector3d(
    tf_msg.transform.translation.x,
    tf_msg.transform.translation.y,
    tf_msg.transform.translation.z);

  world_cloud = std::make_shared<CloudT>();
  world_cloud->reserve(input_cloud->points.size());

  for (const auto & point : input_cloud->points) {
    Eigen::Vector3d p(point.x, point.y, point.z);
    const Eigen::Vector3d transformed = tf_eigen * p;

    PointT out_point;
    out_point.x = static_cast<float>(transformed.x());
    out_point.y = static_cast<float>(transformed.y());
    out_point.z = static_cast<float>(transformed.z());
    world_cloud->points.push_back(out_point);
  }

  world_cloud->width = static_cast<std::uint32_t>(world_cloud->points.size());
  world_cloud->height = 1;
  world_cloud->is_dense = false;
  return true;
}

bool EnvironmentCollisionGenerator::load_ascii_stl_vertices(
  const std::string & path,
  std::vector<Eigen::Vector3d> & vertices,
  std::string & message) const
{
  vertices.clear();

  std::ifstream file(path);
  if (!file.is_open()) {
    message = "Cannot open STL file: " + path;
    return false;
  }

  const bool is_slot = (path == slot_mesh_path_);
  const double scale_x = is_slot ? slot_mesh_scale_x_ : box_mesh_scale_x_;
  const double scale_y = is_slot ? slot_mesh_scale_y_ : box_mesh_scale_y_;
  const double scale_z = is_slot ? slot_mesh_scale_z_ : box_mesh_scale_z_;

  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string word;
    ss >> word;
    if (word == "vertex") {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      ss >> x >> y >> z;
      vertices.emplace_back(x * scale_x, y * scale_y, z * scale_z);
    }
  }

  if (vertices.empty()) {
    message = "No vertices parsed from ASCII STL file: " + path;
    return false;
  }

  message = "Loaded ASCII STL vertices successfully.";
  return true;
}

EnvironmentCollisionGenerator::AxisAlignedBox
EnvironmentCollisionGenerator::compute_world_aabb(
  const std::vector<Eigen::Vector3d> & local_vertices,
  const geometry_msgs::msg::PoseStamped & pose_msg) const
{
  AxisAlignedBox box;
  if (local_vertices.empty()) {
    return box;
  }

  Eigen::Quaterniond q(
    pose_msg.pose.orientation.w,
    pose_msg.pose.orientation.x,
    pose_msg.pose.orientation.y,
    pose_msg.pose.orientation.z);

  if (q.norm() == 0.0) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }

  const Eigen::Vector3d t(
    pose_msg.pose.position.x,
    pose_msg.pose.position.y,
    pose_msg.pose.position.z);

  box.min_x = std::numeric_limits<double>::max();
  box.min_y = std::numeric_limits<double>::max();
  box.min_z = std::numeric_limits<double>::max();
  box.max_x = std::numeric_limits<double>::lowest();
  box.max_y = std::numeric_limits<double>::lowest();
  box.max_z = std::numeric_limits<double>::lowest();

  for (const auto & vertex : local_vertices) {
    const Eigen::Vector3d world_v = q * vertex + t;
    box.min_x = std::min(box.min_x, world_v.x());
    box.min_y = std::min(box.min_y, world_v.y());
    box.min_z = std::min(box.min_z, world_v.z());
    box.max_x = std::max(box.max_x, world_v.x());
    box.max_y = std::max(box.max_y, world_v.y());
    box.max_z = std::max(box.max_z, world_v.z());
  }

  box.min_x -= subtraction_padding_x_;
  box.min_y -= subtraction_padding_y_;
  box.min_z -= subtraction_padding_z_;
  box.max_x += subtraction_padding_x_;
  box.max_y += subtraction_padding_y_;
  box.max_z += subtraction_padding_z_;
  box.valid = true;
  return box;
}

bool EnvironmentCollisionGenerator::point_inside_aabb(
  const PointT & point,
  const AxisAlignedBox & box) const
{
  if (!box.valid) {
    return false;
  }

  return
    point.x >= box.min_x && point.x <= box.max_x &&
    point.y >= box.min_y && point.y <= box.max_y &&
    point.z >= box.min_z && point.z <= box.max_z;
}

EnvironmentCollisionGenerator::CloudT::Ptr
EnvironmentCollisionGenerator::subtract_known_regions(
  const CloudT::Ptr & input_cloud,
  const std::vector<AxisAlignedBox> & subtraction_boxes,
  CloudT::Ptr & removed_cloud,
  std::int64_t & removed_points) const
{
  auto remaining_cloud = std::make_shared<CloudT>();
  removed_cloud = std::make_shared<CloudT>();
  removed_points = 0;

  if (!input_cloud) {
    return remaining_cloud;
  }

  remaining_cloud->reserve(input_cloud->points.size());
  removed_cloud->reserve(input_cloud->points.size());

  for (const auto & point : input_cloud->points) {
    bool inside_any = false;
    for (const auto & box : subtraction_boxes) {
      if (point_inside_aabb(point, box)) {
        inside_any = true;
        break;
      }
    }

    if (inside_any) {
      removed_cloud->points.push_back(point);
      ++removed_points;
    } else {
      remaining_cloud->points.push_back(point);
    }
  }

  remaining_cloud->width = static_cast<std::uint32_t>(remaining_cloud->points.size());
  remaining_cloud->height = 1;
  remaining_cloud->is_dense = false;

  removed_cloud->width = static_cast<std::uint32_t>(removed_cloud->points.size());
  removed_cloud->height = 1;
  removed_cloud->is_dense = false;

  return remaining_cloud;
}

std::vector<moveit_msgs::msg::CollisionObject>
EnvironmentCollisionGenerator::build_environment_collision_objects(
  const CloudT::Ptr & remaining_cloud,
  std::size_t & generated_box_count,
  std::size_t & truncated_box_count) const
{
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  generated_box_count = 0;
  truncated_box_count = 0;

  if (!remaining_cloud || remaining_cloud->empty()) {
    return objects;
  }

  std::unordered_map<std::string, VoxelAccumulator> voxel_map;

  for (const auto & point : remaining_cloud->points) {
    const int ix = static_cast<int>(std::floor(point.x / environment_voxel_size_));
    const int iy = static_cast<int>(std::floor(point.y / environment_voxel_size_));
    const int iz = static_cast<int>(std::floor(point.z / environment_voxel_size_));

    const std::string key =
      std::to_string(ix) + "_" + std::to_string(iy) + "_" + std::to_string(iz);

    auto & acc = voxel_map[key];
    acc.sum_x += point.x;
    acc.sum_y += point.y;
    acc.sum_z += point.z;
    acc.count += 1;
  }

  struct VoxelEntry
  {
    std::string key;
    VoxelAccumulator acc;
  };

  std::vector<VoxelEntry> entries;
  entries.reserve(voxel_map.size());

  for (const auto & kv : voxel_map) {
    if (static_cast<int>(kv.second.count) >= min_points_per_voxel_) {
      entries.push_back({kv.first, kv.second});
    }
  }

  std::sort(
    entries.begin(),
    entries.end(),
    [](const VoxelEntry & a, const VoxelEntry & b) {
      return a.acc.count > b.acc.count;
    });

  const std::size_t limited_size = std::min(
    entries.size(),
    static_cast<std::size_t>(max_environment_boxes_));

  truncated_box_count = entries.size() > limited_size ? (entries.size() - limited_size) : 0;

  objects.reserve(limited_size);

  for (std::size_t i = 0; i < limited_size; ++i) {
    const auto & entry = entries[i];
    const double cx = entry.acc.sum_x / static_cast<double>(entry.acc.count);
    const double cy = entry.acc.sum_y / static_cast<double>(entry.acc.count);
    const double cz = entry.acc.sum_z / static_cast<double>(entry.acc.count);

    std::ostringstream id_stream;
    id_stream << environment_object_prefix_ << i;

    objects.push_back(make_box_collision_object(
      id_stream.str(),
      cx,
      cy,
      cz,
      environment_voxel_size_,
      environment_voxel_size_,
      environment_voxel_size_));
  }

  generated_box_count = objects.size();
  return objects;
}

moveit_msgs::msg::CollisionObject
EnvironmentCollisionGenerator::make_remove_collision_object(const std::string & object_id) const
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = object_id;
  obj.header.frame_id = world_frame_;
  obj.header.stamp = this->now();
  obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  return obj;
}

moveit_msgs::msg::CollisionObject
EnvironmentCollisionGenerator::make_box_collision_object(
  const std::string & object_id,
  double center_x,
  double center_y,
  double center_z,
  double size_x,
  double size_y,
  double size_z) const
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = object_id;
  obj.header.frame_id = world_frame_;
  obj.header.stamp = this->now();
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size_x, size_y, size_z};

  geometry_msgs::msg::Pose pose;
  pose.position.x = center_x;
  pose.position.y = center_y;
  pose.position.z = center_z;
  pose.orientation.w = 1.0;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  return obj;
}

void EnvironmentCollisionGenerator::publish_planning_scene_diff(
  const std::vector<moveit_msgs::msg::CollisionObject> & world_objects) const
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  for (const auto & obj : world_objects) {
    scene.world.collision_objects.push_back(obj);
  }

  planning_scene_pub_->publish(scene);
}

void EnvironmentCollisionGenerator::publish_debug_cloud(
  const CloudT::Ptr & cloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
  const std::string & frame_id) const
{
  if (!cloud || !publisher) {
    return;
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id;
  publisher->publish(msg);
}

bool EnvironmentCollisionGenerator::lookup_pose_from_tf(
  const std::string & source_frame,
  geometry_msgs::msg::PoseStamped & pose_msg,
  std::string & error_message) const
{
  try {
    const auto tf_msg = tf_buffer_->lookupTransform(
      world_frame_,
      source_frame,
      tf2::TimePointZero);

    pose_msg.header.stamp = tf_msg.header.stamp;
    pose_msg.header.frame_id = world_frame_;
    pose_msg.pose.position.x = tf_msg.transform.translation.x;
    pose_msg.pose.position.y = tf_msg.transform.translation.y;
    pose_msg.pose.position.z = tf_msg.transform.translation.z;
    pose_msg.pose.orientation = tf_msg.transform.rotation;

    return true;
  } catch (const std::exception & e) {
    error_message = std::string("Failed TF lookup from '") +
      world_frame_ + "' to '" + source_frame + "': " + e.what();
    return false;
  }
}

std::string EnvironmentCollisionGenerator::pose_to_string(
  const geometry_msgs::msg::PoseStamped & pose_msg) const
{
  std::ostringstream ss;
  ss
    << "frame='" << pose_msg.header.frame_id << "', "
    << "pos=("
    << pose_msg.pose.position.x << ", "
    << pose_msg.pose.position.y << ", "
    << pose_msg.pose.position.z << "), "
    << "quat=("
    << pose_msg.pose.orientation.x << ", "
    << pose_msg.pose.orientation.y << ", "
    << pose_msg.pose.orientation.z << ", "
    << pose_msg.pose.orientation.w << ")";
  return ss.str();
}

bool EnvironmentCollisionGenerator::rebuild_environment_collision(std::string & message)
{
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;
  geometry_msgs::msg::PoseStamped::SharedPtr slot_pose_topic_msg;
  geometry_msgs::msg::PoseStamped::SharedPtr box_pose_topic_msg;
  std::vector<std::string> old_ids;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cloud_msg = latest_cloud_msg_;
    slot_pose_topic_msg = latest_slot_pose_;
    box_pose_topic_msg = latest_box_pose_;
    old_ids = current_environment_object_ids_;
  }

  if (!cloud_msg) {
    message = "No environment cloud received yet.";
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Rebuild requested: cloud_received=true, slot_topic_pose=%s, box_topic_pose=%s",
    slot_pose_topic_msg ? "true" : "false",
    box_pose_topic_msg ? "true" : "false");

  CloudT::Ptr world_cloud;
  std::string transform_message;
  if (!transform_cloud_to_world(*cloud_msg, world_cloud, transform_message)) {
    message = transform_message;
    return false;
  }

  std::shared_ptr<geometry_msgs::msg::PoseStamped> resolved_slot_pose;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> resolved_box_pose;
  std::string slot_pose_source = "none";
  std::string box_pose_source = "none";

  if (slot_pose_topic_msg) {
    resolved_slot_pose = slot_pose_topic_msg;
    slot_pose_source = "topic";
  } else if (enable_tf_fallback_) {
    geometry_msgs::msg::PoseStamped tf_pose;
    std::string tf_error;
    if (lookup_pose_from_tf(slot_tf_frame_, tf_pose, tf_error)) {
      resolved_slot_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(tf_pose);
      slot_pose_source = "tf";
      RCLCPP_INFO(
        this->get_logger(),
        "Slot pose resolved from TF frame '%s': %s",
        slot_tf_frame_.c_str(),
        pose_to_string(tf_pose).c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Slot pose unavailable from topic and TF fallback failed: %s",
        tf_error.c_str());
    }
  }

  if (box_pose_topic_msg) {
    resolved_box_pose = box_pose_topic_msg;
    box_pose_source = "topic";
  } else if (enable_tf_fallback_) {
    geometry_msgs::msg::PoseStamped tf_pose;
    std::string tf_error;
    if (lookup_pose_from_tf(box_tf_frame_, tf_pose, tf_error)) {
      resolved_box_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(tf_pose);
      box_pose_source = "tf";
      RCLCPP_INFO(
        this->get_logger(),
        "Box pose resolved from TF frame '%s': %s",
        box_tf_frame_.c_str(),
        pose_to_string(tf_pose).c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Box pose unavailable from topic and TF fallback failed: %s",
        tf_error.c_str());
    }
  }

  std::vector<AxisAlignedBox> subtraction_boxes;
  std::int64_t slot_removed_estimate = 0;
  std::int64_t box_removed_estimate = 0;

  if (enable_slot_subtraction_) {
    if (!resolved_slot_pose) {
      RCLCPP_WARN(
        this->get_logger(),
        "Slot subtraction skipped: no slot pose available (topic and TF fallback unavailable).");
    } else if (slot_mesh_vertices_.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Slot subtraction skipped: slot mesh vertices are empty.");
    } else {
      subtraction_boxes.push_back(compute_world_aabb(slot_mesh_vertices_, *resolved_slot_pose));
      RCLCPP_INFO(
        this->get_logger(),
        "Slot subtraction enabled using %s pose: %s",
        slot_pose_source.c_str(),
        pose_to_string(*resolved_slot_pose).c_str());
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Slot subtraction disabled by parameter.");
  }

  if (enable_box_subtraction_) {
    if (!resolved_box_pose) {
      RCLCPP_WARN(
        this->get_logger(),
        "Box subtraction skipped: no box pose available (topic and TF fallback unavailable).");
    } else if (box_mesh_vertices_.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Box subtraction skipped: box mesh vertices are empty.");
    } else {
      subtraction_boxes.push_back(compute_world_aabb(box_mesh_vertices_, *resolved_box_pose));
      RCLCPP_INFO(
        this->get_logger(),
        "Box subtraction enabled using %s pose: %s",
        box_pose_source.c_str(),
        pose_to_string(*resolved_box_pose).c_str());
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Box subtraction disabled by parameter.");
  }

  if (subtraction_boxes.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "No subtraction boxes were generated. Environment cloud will be voxelized without slot/box removal.");
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Generated %zu subtraction box(es).",
      subtraction_boxes.size());
  }

  if (world_cloud && !world_cloud->empty()) {
    if (enable_slot_subtraction_ && resolved_slot_pose && !slot_mesh_vertices_.empty()) {
      AxisAlignedBox slot_box = compute_world_aabb(slot_mesh_vertices_, *resolved_slot_pose);
      for (const auto & point : world_cloud->points) {
        if (point_inside_aabb(point, slot_box)) {
          ++slot_removed_estimate;
        }
      }
    }

    if (enable_box_subtraction_ && resolved_box_pose && !box_mesh_vertices_.empty()) {
      AxisAlignedBox box_box = compute_world_aabb(box_mesh_vertices_, *resolved_box_pose);
      for (const auto & point : world_cloud->points) {
        if (point_inside_aabb(point, box_box)) {
          ++box_removed_estimate;
        }
      }
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Subtraction preview: slot_estimated_removed=%ld, box_estimated_removed=%ld",
    static_cast<long>(slot_removed_estimate),
    static_cast<long>(box_removed_estimate));

  CloudT::Ptr removed_cloud;
  std::int64_t removed_points = 0;
  CloudT::Ptr remaining_cloud = subtract_known_regions(
    world_cloud,
    subtraction_boxes,
    removed_cloud,
    removed_points);

  std::size_t generated_box_count = 0;
  std::size_t truncated_box_count = 0;
  auto new_objects = build_environment_collision_objects(
    remaining_cloud,
    generated_box_count,
    truncated_box_count);

  std::vector<moveit_msgs::msg::CollisionObject> diff_objects;
  diff_objects.reserve(old_ids.size() + new_objects.size());

  for (const auto & id : old_ids) {
    diff_objects.push_back(make_remove_collision_object(id));
  }

  std::vector<std::string> new_ids;
  new_ids.reserve(new_objects.size());

  for (const auto & obj : new_objects) {
    diff_objects.push_back(obj);
    new_ids.push_back(obj.id);
  }

  publish_planning_scene_diff(diff_objects);

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_environment_object_ids_ = new_ids;
  }

  if (publish_remaining_cloud_) {
    publish_debug_cloud(remaining_cloud, remaining_cloud_pub_, world_frame_);
  }

  if (publish_removed_cloud_) {
    publish_debug_cloud(removed_cloud, removed_cloud_pub_, world_frame_);
  }

  std_msgs::msg::String status_msg;
  std::ostringstream status_stream;
  status_stream
    << "Environment collision rebuilt. "
    << "input_points=" << world_cloud->points.size()
    << ", removed_points=" << removed_points
    << ", remaining_points=" << remaining_cloud->points.size()
    << ", boxes=" << generated_box_count
    << ", truncated=" << truncated_box_count
    << ", slot_pose_source=" << slot_pose_source
    << ", box_pose_source=" << box_pose_source
    << ", subtraction_boxes=" << subtraction_boxes.size();
  status_msg.data = status_stream.str();
  status_pub_->publish(status_msg);

  RCLCPP_INFO(this->get_logger(), "%s", status_msg.data.c_str());

  message = status_msg.data;
  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnvironmentCollisionGenerator>());
  rclcpp::shutdown();
  return 0;
}
