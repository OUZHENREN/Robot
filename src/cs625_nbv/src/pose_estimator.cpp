#include "cs625_nbv/pose_estimator.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

namespace cs625_nbv {

// PIMPL pattern to hide PCL internals
class PoseEstimator::Impl {
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud;
};

PoseEstimator::PoseEstimator()
    : impl_(std::make_unique<Impl>())
{
    impl_->model_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

PoseEstimator::~PoseEstimator() = default;

void PoseEstimator::set_model_cloud(const sensor_msgs::msg::PointCloud2& cloud)
{
    impl_->model_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud, *impl_->model_cloud);
}

void PoseEstimator::set_model_cloud(const Eigen::Matrix<double, 3, Eigen::Dynamic>& points)
{
    impl_->model_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    impl_->model_cloud->reserve(points.cols());
    for (Eigen::Index i = 0; i < points.cols(); ++i) {
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(points(0, i));
        pt.y = static_cast<float>(points(1, i));
        pt.z = static_cast<float>(points(2, i));
        impl_->model_cloud->push_back(pt);
    }
}

Eigen::Isometry3d PoseEstimator::estimate_pose(
    const sensor_msgs::msg::PointCloud2& scene_cloud,
    const Eigen::Isometry3d& initial_guess,
    int max_iterations,
    double max_correspondence_distance)
{
    // Convert scene cloud to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(scene_cloud, *scene);

    // Downsample both clouds for faster ICP
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(0.005f, 0.005f, 0.005f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    voxel.setInputCloud(impl_->model_cloud);
    voxel.filter(*model_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    voxel.setInputCloud(scene);
    voxel.filter(*scene_filtered);

    // ICP registration
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(max_iterations);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setRANSACOutlierRejectionThreshold(0.01);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-6);

    // Set initial guess
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    init_guess.block<3, 3>(0, 0) = initial_guess.rotation().cast<float>();
    init_guess.block<3, 1>(0, 3) = initial_guess.translation().cast<float>();

    icp.setInputSource(model_filtered);
    icp.setInputTarget(scene_filtered);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, init_guess);

    // Get the final transformation
    Eigen::Matrix4f result_matrix = icp.getFinalTransformation();
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = result_matrix.block<3, 1>(0, 3).cast<double>();
    result.linear() = result_matrix.block<3, 3>(0, 0).cast<double>();

    return result;
}

geometry_msgs::msg::PoseStamped PoseEstimator::to_pose_stamped(
    const Eigen::Isometry3d& transform,
    const std::string& frame_id)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;

    // Translation
    pose.pose.position.x = transform.translation().x();
    pose.pose.position.y = transform.translation().y();
    pose.pose.position.z = transform.translation().z();

    // Rotation as quaternion
    Eigen::Quaterniond q(transform.rotation());
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    return pose;
}

}  // namespace cs625_nbv
