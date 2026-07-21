#ifndef ENVIRONMENT_POINT_CLOUD_PUBLISHER__ENVIRONMENT_POINT_CLOUD_PUBLISHER_HPP_
#define ENVIRONMENT_POINT_CLOUD_PUBLISHER__ENVIRONMENT_POINT_CLOUD_PUBLISHER_HPP_

#include <chrono>
#include <filesystem>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pointcloud_workflow_interfaces/srv/get_publisher_status.hpp"
#include "pointcloud_workflow_interfaces/srv/list_available_pcds.hpp"
#include "pointcloud_workflow_interfaces/srv/rebuild_merged_cloud.hpp"
#include "pointcloud_workflow_interfaces/srv/set_enabled_pcds.hpp"

class EnvironmentPointCloudPublisher : public rclcpp::Node
{
public:
  EnvironmentPointCloudPublisher();

private:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  void declare_and_load_parameters();
  void sanitize_parameters();
  void reload_runtime_parameters();
  void log_configuration() const;

  std::vector<std::filesystem::path> scan_pcd_files() const;
  std::vector<std::string> scan_timestamp_directory_names() const;
  bool is_timestamp_directory(const std::filesystem::path & path) const;
  std::filesystem::path make_pcd_path_from_timestamp_dir(const std::string & dir_name) const;

  bool reload_clouds_if_needed(bool force_log);
  bool rebuild_merged_cloud_from_enabled_set();
  bool rebuild_merged_cloud(const std::vector<std::filesystem::path> & files);
  CloudT::Ptr process_cloud(const CloudT::Ptr & input_cloud) const;
  bool save_processed_merged_cloud_to_pcd(const CloudT::Ptr & cloud) const;
  bool sync_processed_merged_cloud_to_shared_directory() const;
  void publish_merged_cloud();

  void sync_enabled_dirs_with_scan(const std::vector<std::string> & scanned_dirs);

  void handle_list_available_pcds(
    const std::shared_ptr<pointcloud_workflow_interfaces::srv::ListAvailablePcds::Request> request,
    std::shared_ptr<pointcloud_workflow_interfaces::srv::ListAvailablePcds::Response> response);

  void handle_set_enabled_pcds(
    const std::shared_ptr<pointcloud_workflow_interfaces::srv::SetEnabledPcds::Request> request,
    std::shared_ptr<pointcloud_workflow_interfaces::srv::SetEnabledPcds::Response> response);

  void handle_rebuild_merged_cloud(
    const std::shared_ptr<pointcloud_workflow_interfaces::srv::RebuildMergedCloud::Request> request,
    std::shared_ptr<pointcloud_workflow_interfaces::srv::RebuildMergedCloud::Response> response);

  void handle_get_publisher_status(
    const std::shared_ptr<pointcloud_workflow_interfaces::srv::GetPublisherStatus::Request> request,
    std::shared_ptr<pointcloud_workflow_interfaces::srv::GetPublisherStatus::Response> response);

  std::string directory_path_;
  std::string topic_name_;
  std::string frame_id_;
  std::string target_pcd_filename_;
  std::string output_directory_path_;
  std::string shared_output_directory_path_;

  double publish_interval_sec_;
  double rescan_interval_sec_;

  bool enable_crop_box_;
  double crop_min_x_;
  double crop_min_y_;
  double crop_min_z_;
  double crop_max_x_;
  double crop_max_y_;
  double crop_max_z_;

  bool enable_voxel_downsample_;
  double voxel_leaf_size_;

  bool enable_statistical_outlier_removal_;
  int sor_mean_k_;
  double sor_stddev_mul_thresh_;

  bool enable_radius_outlier_removal_;
  double ror_radius_search_;
  int ror_min_neighbors_;

  bool enable_smoothing_;

  std::vector<std::filesystem::path> pcd_files_;
  std::set<std::string> enabled_timestamp_dirs_;
  bool user_has_manually_set_enabled_dirs_{false};

  CloudT::Ptr merged_cloud_;
  std::size_t reload_count_;
  std::int64_t last_raw_points_;
  std::int64_t last_processed_points_;

  mutable std::mutex data_mutex_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr rescan_timer_;

  rclcpp::Service<pointcloud_workflow_interfaces::srv::ListAvailablePcds>::SharedPtr
    list_available_pcds_service_;
  rclcpp::Service<pointcloud_workflow_interfaces::srv::SetEnabledPcds>::SharedPtr
    set_enabled_pcds_service_;
  rclcpp::Service<pointcloud_workflow_interfaces::srv::RebuildMergedCloud>::SharedPtr
    rebuild_merged_cloud_service_;
  rclcpp::Service<pointcloud_workflow_interfaces::srv::GetPublisherStatus>::SharedPtr
    get_publisher_status_service_;
};

#endif  // ENVIRONMENT_POINT_CLOUD_PUBLISHER__ENVIRONMENT_POINT_CLOUD_PUBLISHER_HPP_
