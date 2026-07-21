#include "environment_point_cloud_publisher/environment_point_cloud_publisher.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <utility>

#include <Eigen/Core>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

EnvironmentPointCloudPublisher::EnvironmentPointCloudPublisher()
: Node("environment_point_cloud_publisher"),
  merged_cloud_(std::make_shared<CloudT>()),
  reload_count_(0),
  last_raw_points_(0),
  last_processed_points_(0)
{
  declare_and_load_parameters();
  sanitize_parameters();

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, 10);

  list_available_pcds_service_ =
    this->create_service<pointcloud_workflow_interfaces::srv::ListAvailablePcds>(
      "~/list_available_pcds",
      std::bind(
        &EnvironmentPointCloudPublisher::handle_list_available_pcds,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  set_enabled_pcds_service_ =
    this->create_service<pointcloud_workflow_interfaces::srv::SetEnabledPcds>(
      "~/set_enabled_pcds",
      std::bind(
        &EnvironmentPointCloudPublisher::handle_set_enabled_pcds,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  rebuild_merged_cloud_service_ =
    this->create_service<pointcloud_workflow_interfaces::srv::RebuildMergedCloud>(
      "~/rebuild_merged_cloud",
      std::bind(
        &EnvironmentPointCloudPublisher::handle_rebuild_merged_cloud,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  get_publisher_status_service_ =
    this->create_service<pointcloud_workflow_interfaces::srv::GetPublisherStatus>(
      "~/get_publisher_status",
      std::bind(
        &EnvironmentPointCloudPublisher::handle_get_publisher_status,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  log_configuration();
  reload_clouds_if_needed(true);

  const auto publish_interval = std::chrono::duration<double>(publish_interval_sec_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(publish_interval),
    std::bind(&EnvironmentPointCloudPublisher::publish_merged_cloud, this));

  const auto rescan_interval = std::chrono::duration<double>(rescan_interval_sec_);
  rescan_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(rescan_interval),
    [this]() {
      reload_clouds_if_needed(false);
    });
}

void EnvironmentPointCloudPublisher::declare_and_load_parameters()
{
  directory_path_ = this->declare_parameter<std::string>(
    "directory_path", "/home/yff/environment_point_cloud");
  topic_name_ = this->declare_parameter<std::string>(
    "topic_name", "/environment/point_cloud");
  frame_id_ = this->declare_parameter<std::string>(
    "frame_id", "base_link");
  target_pcd_filename_ = this->declare_parameter<std::string>(
    "target_pcd_filename", "environment_point_cloud.pcd");
  output_directory_path_ = this->declare_parameter<std::string>(
    "output_directory_path", "/home/yff/environment_point_cloud_fusion");
  shared_output_directory_path_ = this->declare_parameter<std::string>(
    "shared_output_directory_path", "/mnt/hgfs/Model/environment_point_cloud_fusion");

  publish_interval_sec_ = this->declare_parameter<double>(
    "publish_interval_sec", 1.0);
  rescan_interval_sec_ = this->declare_parameter<double>(
    "rescan_interval_sec", 2.0);

  enable_crop_box_ = this->declare_parameter<bool>(
    "enable_crop_box", false);
  crop_min_x_ = this->declare_parameter<double>("crop_min_x", -2.0);
  crop_min_y_ = this->declare_parameter<double>("crop_min_y", -2.0);
  crop_min_z_ = this->declare_parameter<double>("crop_min_z", -2.0);
  crop_max_x_ = this->declare_parameter<double>("crop_max_x", 2.0);
  crop_max_y_ = this->declare_parameter<double>("crop_max_y", 2.0);
  crop_max_z_ = this->declare_parameter<double>("crop_max_z", 2.0);

  enable_voxel_downsample_ = this->declare_parameter<bool>(
    "enable_voxel_downsample", true);
  voxel_leaf_size_ = this->declare_parameter<double>(
    "voxel_leaf_size", 0.01);

  enable_statistical_outlier_removal_ = this->declare_parameter<bool>(
    "enable_statistical_outlier_removal", true);
  sor_mean_k_ = this->declare_parameter<int>(
    "sor_mean_k", 30);
  sor_stddev_mul_thresh_ = this->declare_parameter<double>(
    "sor_stddev_mul_thresh", 1.0);

  enable_radius_outlier_removal_ = this->declare_parameter<bool>(
    "enable_radius_outlier_removal", true);
  ror_radius_search_ = this->declare_parameter<double>(
    "ror_radius_search", 0.03);
  ror_min_neighbors_ = this->declare_parameter<int>(
    "ror_min_neighbors", 4);

  enable_smoothing_ = this->declare_parameter<bool>(
    "enable_smoothing", false);
}

void EnvironmentPointCloudPublisher::reload_runtime_parameters()
{
  this->get_parameter("directory_path", directory_path_);
  this->get_parameter("topic_name", topic_name_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("target_pcd_filename", target_pcd_filename_);
  this->get_parameter("output_directory_path", output_directory_path_);
  this->get_parameter("shared_output_directory_path", shared_output_directory_path_);
  this->get_parameter("publish_interval_sec", publish_interval_sec_);
  this->get_parameter("rescan_interval_sec", rescan_interval_sec_);

  this->get_parameter("enable_crop_box", enable_crop_box_);
  this->get_parameter("crop_min_x", crop_min_x_);
  this->get_parameter("crop_min_y", crop_min_y_);
  this->get_parameter("crop_min_z", crop_min_z_);
  this->get_parameter("crop_max_x", crop_max_x_);
  this->get_parameter("crop_max_y", crop_max_y_);
  this->get_parameter("crop_max_z", crop_max_z_);

  this->get_parameter("enable_voxel_downsample", enable_voxel_downsample_);
  this->get_parameter("voxel_leaf_size", voxel_leaf_size_);

  this->get_parameter("enable_statistical_outlier_removal", enable_statistical_outlier_removal_);
  this->get_parameter("sor_mean_k", sor_mean_k_);
  this->get_parameter("sor_stddev_mul_thresh", sor_stddev_mul_thresh_);

  this->get_parameter("enable_radius_outlier_removal", enable_radius_outlier_removal_);
  this->get_parameter("ror_radius_search", ror_radius_search_);
  this->get_parameter("ror_min_neighbors", ror_min_neighbors_);

  this->get_parameter("enable_smoothing", enable_smoothing_);

  sanitize_parameters();
}

void EnvironmentPointCloudPublisher::sanitize_parameters()
{
  if (publish_interval_sec_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid publish_interval_sec=%.3f, fallback to 1.0 second.",
      publish_interval_sec_);
    publish_interval_sec_ = 1.0;
  }

  if (rescan_interval_sec_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid rescan_interval_sec=%.3f, fallback to 2.0 seconds.",
      rescan_interval_sec_);
    rescan_interval_sec_ = 2.0;
  }

  if (voxel_leaf_size_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid voxel_leaf_size=%.6f, fallback to 0.01 meter.",
      voxel_leaf_size_);
    voxel_leaf_size_ = 0.01;
  }

  if (sor_mean_k_ <= 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid sor_mean_k=%d, fallback to 30.",
      sor_mean_k_);
    sor_mean_k_ = 30;
  }

  if (sor_stddev_mul_thresh_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid sor_stddev_mul_thresh=%.6f, fallback to 1.0.",
      sor_stddev_mul_thresh_);
    sor_stddev_mul_thresh_ = 1.0;
  }

  if (ror_radius_search_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid ror_radius_search=%.6f, fallback to 0.03 meter.",
      ror_radius_search_);
    ror_radius_search_ = 0.03;
  }

  if (ror_min_neighbors_ <= 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid ror_min_neighbors=%d, fallback to 4.",
      ror_min_neighbors_);
    ror_min_neighbors_ = 4;
  }

  if (enable_smoothing_) {
    RCLCPP_WARN(
      this->get_logger(),
      "enable_smoothing=true, but smoothing is currently reserved and not applied.");
  }
}

void EnvironmentPointCloudPublisher::log_configuration() const
{
  RCLCPP_INFO(
    this->get_logger(),
    "Configuration: directory_path='%s', topic_name='%s', frame_id='%s', "
    "target_pcd_filename='%s', output_directory_path='%s', shared_output_directory_path='%s', "
    "publish_interval_sec=%.3f, rescan_interval_sec=%.3f, "
    "enable_crop_box=%s, crop_min=(%.3f, %.3f, %.3f), crop_max=(%.3f, %.3f, %.3f), "
    "enable_voxel_downsample=%s, voxel_leaf_size=%.4f, "
    "enable_statistical_outlier_removal=%s, sor_mean_k=%d, sor_stddev_mul_thresh=%.3f, "
    "enable_radius_outlier_removal=%s, ror_radius_search=%.4f, ror_min_neighbors=%d, "
    "enable_smoothing=%s",
    directory_path_.c_str(),
    topic_name_.c_str(),
    frame_id_.c_str(),
    target_pcd_filename_.c_str(),
    output_directory_path_.c_str(),
    shared_output_directory_path_.c_str(),
    publish_interval_sec_,
    rescan_interval_sec_,
    enable_crop_box_ ? "true" : "false",
    crop_min_x_, crop_min_y_, crop_min_z_,
    crop_max_x_, crop_max_y_, crop_max_z_,
    enable_voxel_downsample_ ? "true" : "false",
    voxel_leaf_size_,
    enable_statistical_outlier_removal_ ? "true" : "false",
    sor_mean_k_,
    sor_stddev_mul_thresh_,
    enable_radius_outlier_removal_ ? "true" : "false",
    ror_radius_search_,
    ror_min_neighbors_,
    enable_smoothing_ ? "true" : "false");
}

bool EnvironmentPointCloudPublisher::is_timestamp_directory(
  const std::filesystem::path & path) const
{
  if (!std::filesystem::is_directory(path)) {
    return false;
  }

  const std::string name = path.filename().string();
  if (name.empty()) {
    return false;
  }

  return std::all_of(name.begin(), name.end(), [](unsigned char ch) {
    return std::isdigit(ch) != 0;
  });
}

std::filesystem::path EnvironmentPointCloudPublisher::make_pcd_path_from_timestamp_dir(
  const std::string & dir_name) const
{
  return std::filesystem::path(directory_path_) / dir_name / target_pcd_filename_;
}

std::vector<std::string> EnvironmentPointCloudPublisher::scan_timestamp_directory_names() const
{
  std::vector<std::string> dirs;
  const std::filesystem::path root_path(directory_path_);

  if (!std::filesystem::exists(root_path)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Directory does not exist: %s",
      directory_path_.c_str());
    return dirs;
  }

  if (!std::filesystem::is_directory(root_path)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Path is not a directory: %s",
      directory_path_.c_str());
    return dirs;
  }

  for (const auto & entry : std::filesystem::directory_iterator(root_path)) {
    if (!entry.is_directory()) {
      continue;
    }

    const auto dir_path = entry.path();
    if (is_timestamp_directory(dir_path)) {
      dirs.push_back(dir_path.filename().string());
    }
  }

  std::sort(dirs.begin(), dirs.end());
  return dirs;
}

std::vector<std::filesystem::path> EnvironmentPointCloudPublisher::scan_pcd_files() const
{
  std::vector<std::filesystem::path> files;
  const auto dirs = scan_timestamp_directory_names();

  for (const auto & dir_name : dirs) {
    const auto pcd_path = make_pcd_path_from_timestamp_dir(dir_name);
    if (std::filesystem::exists(pcd_path) && std::filesystem::is_regular_file(pcd_path)) {
      files.push_back(pcd_path);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Target PCD file not found in timestamp directory: %s",
        pcd_path.string().c_str());
    }
  }

  return files;
}

void EnvironmentPointCloudPublisher::sync_enabled_dirs_with_scan(
  const std::vector<std::string> & scanned_dirs)
{
  std::set<std::string> scanned_set(scanned_dirs.begin(), scanned_dirs.end());

  if (enabled_timestamp_dirs_.empty() && !user_has_manually_set_enabled_dirs_) {
    for (const auto & dir_name : scanned_dirs) {
      enabled_timestamp_dirs_.insert(dir_name);
    }
    return;
  }

  for (auto it = enabled_timestamp_dirs_.begin(); it != enabled_timestamp_dirs_.end();) {
    if (scanned_set.find(*it) == scanned_set.end()) {
      it = enabled_timestamp_dirs_.erase(it);
    } else {
      ++it;
    }
  }

  if (!user_has_manually_set_enabled_dirs_) {
    for (const auto & dir_name : scanned_dirs) {
      if (enabled_timestamp_dirs_.find(dir_name) == enabled_timestamp_dirs_.end()) {
        enabled_timestamp_dirs_.insert(dir_name);
      }
    }
  }
}

EnvironmentPointCloudPublisher::CloudT::Ptr
EnvironmentPointCloudPublisher::process_cloud(const CloudT::Ptr & input_cloud) const
{
  auto current_cloud = std::make_shared<CloudT>(*input_cloud);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Processing merged cloud: input_points=%zu",
    current_cloud->points.size());

  if (enable_crop_box_) {
    auto cropped_cloud = std::make_shared<CloudT>();
    pcl::CropBox<PointT> crop_box_filter;
    crop_box_filter.setInputCloud(current_cloud);
    crop_box_filter.setMin(Eigen::Vector4f(
      static_cast<float>(crop_min_x_),
      static_cast<float>(crop_min_y_),
      static_cast<float>(crop_min_z_),
      1.0f));
    crop_box_filter.setMax(Eigen::Vector4f(
      static_cast<float>(crop_max_x_),
      static_cast<float>(crop_max_y_),
      static_cast<float>(crop_max_z_),
      1.0f));
    crop_box_filter.filter(*cropped_cloud);

    RCLCPP_DEBUG(
      this->get_logger(),
      "After CropBox: points=%zu",
      cropped_cloud->points.size());

    current_cloud = cropped_cloud;
  }

  if (enable_voxel_downsample_) {
    auto voxel_cloud = std::make_shared<CloudT>();
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(current_cloud);
    voxel_filter.setLeafSize(
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_));
    voxel_filter.filter(*voxel_cloud);

    RCLCPP_DEBUG(
      this->get_logger(),
      "After VoxelGrid: points=%zu",
      voxel_cloud->points.size());

    current_cloud = voxel_cloud;
  }

  if (enable_statistical_outlier_removal_ && !current_cloud->empty()) {
    auto sor_cloud = std::make_shared<CloudT>();
    pcl::StatisticalOutlierRemoval<PointT> sor_filter;
    sor_filter.setInputCloud(current_cloud);
    sor_filter.setMeanK(sor_mean_k_);
    sor_filter.setStddevMulThresh(sor_stddev_mul_thresh_);
    sor_filter.filter(*sor_cloud);

    RCLCPP_DEBUG(
      this->get_logger(),
      "After StatisticalOutlierRemoval: points=%zu",
      sor_cloud->points.size());

    current_cloud = sor_cloud;
  }

  if (enable_radius_outlier_removal_ && !current_cloud->empty()) {
    auto ror_cloud = std::make_shared<CloudT>();
    pcl::RadiusOutlierRemoval<PointT> ror_filter;
    ror_filter.setInputCloud(current_cloud);
    ror_filter.setRadiusSearch(ror_radius_search_);
    ror_filter.setMinNeighborsInRadius(ror_min_neighbors_);
    ror_filter.filter(*ror_cloud);

    RCLCPP_DEBUG(
      this->get_logger(),
      "After RadiusOutlierRemoval: points=%zu",
      ror_cloud->points.size());

    current_cloud = ror_cloud;
  }

  if (enable_smoothing_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Smoothing is enabled in parameters but not implemented in this version. "
      "Publishing filtered cloud without smoothing.");
  }

  return current_cloud;
}

bool EnvironmentPointCloudPublisher::save_processed_merged_cloud_to_pcd(
  const CloudT::Ptr & cloud) const
{
  if (!cloud) {
    RCLCPP_ERROR(this->get_logger(), "Cannot save merged cloud: cloud pointer is null.");
    return false;
  }

  if (cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Merged cloud is empty. Skip saving PCD file.");
    return false;
  }

  const std::filesystem::path output_dir(output_directory_path_);
  const std::filesystem::path output_file = output_dir / "environment_point_cloud.pcd";

  try {
    if (!std::filesystem::exists(output_dir)) {
      std::filesystem::create_directories(output_dir);
      RCLCPP_INFO(
        this->get_logger(),
        "Created output directory for merged cloud PCD: %s",
        output_dir.string().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to create output directory '%s': %s",
      output_dir.string().c_str(),
      e.what());
    return false;
  }

  if (pcl::io::savePCDFileBinary(output_file.string(), *cloud) < 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to save merged cloud PCD file: %s",
      output_file.string().c_str());
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Saved merged cloud PCD file successfully: %s, points=%zu",
    output_file.string().c_str(),
    cloud->points.size());

  return true;
}

bool EnvironmentPointCloudPublisher::sync_processed_merged_cloud_to_shared_directory() const
{
  const std::filesystem::path source_file =
    std::filesystem::path(output_directory_path_) / "environment_point_cloud.pcd";
  const std::filesystem::path shared_dir(shared_output_directory_path_);
  const std::filesystem::path destination_file =
    shared_dir / "environment_point_cloud.pcd";

  try {
    if (!std::filesystem::exists(source_file) ||
      !std::filesystem::is_regular_file(source_file))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot sync merged cloud PCD because source file does not exist: %s",
        source_file.string().c_str());
      return false;
    }

    if (!std::filesystem::exists(shared_dir)) {
      std::filesystem::create_directories(shared_dir);
      RCLCPP_INFO(
        this->get_logger(),
        "Created shared output directory: %s",
        shared_dir.string().c_str());
    }

    std::filesystem::copy_file(
      source_file,
      destination_file,
      std::filesystem::copy_options::overwrite_existing);

    RCLCPP_INFO(
      this->get_logger(),
      "Synced merged cloud PCD to shared directory successfully: %s -> %s",
      source_file.string().c_str(),
      destination_file.string().c_str());

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to sync merged cloud PCD to shared directory. source='%s', destination='%s', error='%s'",
      source_file.string().c_str(),
      destination_file.string().c_str(),
      e.what());
    return false;
  }
}

bool EnvironmentPointCloudPublisher::rebuild_merged_cloud(
  const std::vector<std::filesystem::path> & files)
{
  auto raw_merged_cloud = std::make_shared<CloudT>();
  std::size_t loaded_file_count = 0;

  for (const auto & file_path : files) {
    CloudT cloud;
    if (pcl::io::loadPCDFile<PointT>(file_path.string(), cloud) < 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to load PCD file: %s",
        file_path.string().c_str());
      continue;
    }

    *raw_merged_cloud += cloud;
    ++loaded_file_count;

    RCLCPP_DEBUG(
      this->get_logger(),
      "Loaded PCD: %s, points=%zu",
      file_path.string().c_str(),
      cloud.points.size());
  }

  auto processed_cloud = process_cloud(raw_merged_cloud);
  const bool save_ok = save_processed_merged_cloud_to_pcd(processed_cloud);
  const bool sync_ok = save_ok ? sync_processed_merged_cloud_to_shared_directory() : false;

  std::size_t current_reload_count = 0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    merged_cloud_ = processed_cloud;
    pcd_files_ = files;
    ++reload_count_;
    current_reload_count = reload_count_;
    last_raw_points_ = static_cast<std::int64_t>(raw_merged_cloud->points.size());
    last_processed_points_ = static_cast<std::int64_t>(merged_cloud_->points.size());
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Rebuilt merged cloud: files=%zu, loaded_files=%zu, raw_points=%zu, processed_points=%zu, reload_count=%zu, pcd_saved=%s, pcd_synced=%s",
    files.size(),
    loaded_file_count,
    raw_merged_cloud->points.size(),
    processed_cloud->points.size(),
    current_reload_count,
    save_ok ? "true" : "false",
    sync_ok ? "true" : "false");

  return loaded_file_count > 0 && save_ok && sync_ok;
}

bool EnvironmentPointCloudPublisher::rebuild_merged_cloud_from_enabled_set()
{
  reload_runtime_parameters();

  std::vector<std::filesystem::path> enabled_files;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto & dir_name : enabled_timestamp_dirs_) {
      const auto pcd_path = make_pcd_path_from_timestamp_dir(dir_name);
      if (std::filesystem::exists(pcd_path) && std::filesystem::is_regular_file(pcd_path)) {
        enabled_files.push_back(pcd_path);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Enabled directory has no valid PCD file: %s",
          pcd_path.string().c_str());
      }
    }
  }

  std::sort(enabled_files.begin(), enabled_files.end());

  if (enabled_files.empty()) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pcd_files_.clear();
    merged_cloud_->clear();
    last_raw_points_ = 0;
    last_processed_points_ = 0;
    RCLCPP_WARN(this->get_logger(), "No enabled PCD files available to rebuild merged cloud.");
    return false;
  }

  return rebuild_merged_cloud(enabled_files);
}

bool EnvironmentPointCloudPublisher::reload_clouds_if_needed(bool force_log)
{
  reload_runtime_parameters();

  const auto scanned_dirs = scan_timestamp_directory_names();

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    sync_enabled_dirs_with_scan(scanned_dirs);
  }

  std::vector<std::filesystem::path> desired_files;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto & dir_name : enabled_timestamp_dirs_) {
      const auto pcd_path = make_pcd_path_from_timestamp_dir(dir_name);
      if (std::filesystem::exists(pcd_path) && std::filesystem::is_regular_file(pcd_path)) {
        desired_files.push_back(pcd_path);
      }
    }
  }

  std::sort(desired_files.begin(), desired_files.end());

  std::vector<std::filesystem::path> old_files;
  std::size_t old_point_count = 0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    old_files = pcd_files_;
    old_point_count = merged_cloud_ ? merged_cloud_->points.size() : 0;
  }

  if (desired_files == old_files) {
    if (force_log) {
      RCLCPP_INFO(
        this->get_logger(),
        "No directory change detected. Using existing merged cloud. files=%zu, points=%zu",
        old_files.size(),
        old_point_count);
    }
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Detected point cloud file set change. Old files=%zu, New files=%zu",
    old_files.size(),
    desired_files.size());

  if (desired_files.empty()) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pcd_files_.clear();
    merged_cloud_->clear();
    last_raw_points_ = 0;
    last_processed_points_ = 0;

    RCLCPP_WARN(
      this->get_logger(),
      "No valid enabled PCD files found under timestamp subdirectories of: %s",
      directory_path_.c_str());
    return true;
  }

  return rebuild_merged_cloud(desired_files);
}

void EnvironmentPointCloudPublisher::publish_merged_cloud()
{
  CloudT::Ptr cloud_to_publish;
  std::size_t file_count = 0;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cloud_to_publish = merged_cloud_;
    file_count = pcd_files_.size();
  }

  if (!cloud_to_publish || cloud_to_publish->empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Merged cloud is empty. Nothing to publish.");
    return;
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud_to_publish, msg);
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;

  publisher_->publish(msg);

  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    5000,
    "Published processed environment cloud: files=%zu, total_points=%zu",
    file_count,
    cloud_to_publish->points.size());
}

void EnvironmentPointCloudPublisher::handle_list_available_pcds(
  const std::shared_ptr<pointcloud_workflow_interfaces::srv::ListAvailablePcds::Request> /*request*/,
  std::shared_ptr<pointcloud_workflow_interfaces::srv::ListAvailablePcds::Response> response)
{
  const auto dirs = scan_timestamp_directory_names();

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    sync_enabled_dirs_with_scan(dirs);
  }

  response->success = true;
  response->message = "Available timestamp directories fetched successfully.";
  response->timestamp_dirs = dirs;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    response->enabled.reserve(dirs.size());
    for (const auto & dir_name : dirs) {
      response->enabled.push_back(enabled_timestamp_dirs_.find(dir_name) != enabled_timestamp_dirs_.end());
    }
  }
}

void EnvironmentPointCloudPublisher::handle_set_enabled_pcds(
  const std::shared_ptr<pointcloud_workflow_interfaces::srv::SetEnabledPcds::Request> request,
  std::shared_ptr<pointcloud_workflow_interfaces::srv::SetEnabledPcds::Response> response)
{
  const auto scanned_dirs = scan_timestamp_directory_names();
  const std::set<std::string> scanned_set(scanned_dirs.begin(), scanned_dirs.end());

  std::set<std::string> new_enabled_set;
  for (const auto & dir_name : request->timestamp_dirs) {
    if (scanned_set.find(dir_name) == scanned_set.end()) {
      response->success = false;
      response->message = "Unknown timestamp directory: " + dir_name;
      response->enabled_count = 0;
      return;
    }
    new_enabled_set.insert(dir_name);
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    enabled_timestamp_dirs_ = new_enabled_set;
    user_has_manually_set_enabled_dirs_ = true;
  }

  const bool rebuild_ok = rebuild_merged_cloud_from_enabled_set();

  response->success = rebuild_ok || new_enabled_set.empty();
  response->message = new_enabled_set.empty()
    ? "Enabled PCD set updated to empty. Merged cloud cleared."
    : (rebuild_ok ? "Enabled PCD set updated, merged cloud rebuilt, saved, and synced successfully."
                  : "Enabled PCD set updated, but merged cloud rebuild/save/sync failed or no valid PCD files were found.");
  response->enabled_count = static_cast<int32_t>(new_enabled_set.size());
}

void EnvironmentPointCloudPublisher::handle_rebuild_merged_cloud(
  const std::shared_ptr<pointcloud_workflow_interfaces::srv::RebuildMergedCloud::Request> /*request*/,
  std::shared_ptr<pointcloud_workflow_interfaces::srv::RebuildMergedCloud::Response> response)
{
  const bool ok = rebuild_merged_cloud_from_enabled_set();

  response->success = ok;
  response->message = ok
    ? "Merged cloud rebuilt successfully, saved locally, and synced to shared directory."
    : "Merged cloud rebuild failed, local save failed, shared sync failed, or no enabled PCD files were available.";

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    response->file_count = static_cast<int32_t>(pcd_files_.size());
    response->raw_points = last_raw_points_;
    response->processed_points = last_processed_points_;
  }
}

void EnvironmentPointCloudPublisher::handle_get_publisher_status(
  const std::shared_ptr<pointcloud_workflow_interfaces::srv::GetPublisherStatus::Request> /*request*/,
  std::shared_ptr<pointcloud_workflow_interfaces::srv::GetPublisherStatus::Response> response)
{
  const auto dirs = scan_timestamp_directory_names();

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    sync_enabled_dirs_with_scan(dirs);
  }

  response->success = true;
  response->message = "Publisher status fetched successfully.";
  response->available_file_count = static_cast<int32_t>(dirs.size());

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    response->enabled_file_count = static_cast<int32_t>(enabled_timestamp_dirs_.size());
    response->raw_points = last_raw_points_;
    response->processed_points = last_processed_points_;
    response->reload_count = static_cast<int32_t>(reload_count_);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnvironmentPointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
