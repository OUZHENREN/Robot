/**
 * @file synthetic_camera_publisher_node.cpp
 * @brief Pose-aware, z-buffered virtual RGB-D observation source.
 *
 * The same discrete projection model is used by the virtual sensor and the
 * P5 pre-observation predictor. This is synthetic software evidence only.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "cs625_nbv/virtual_observation_model.hpp"

namespace {
double occlusionFraction(const std::string& level)
{
    if (level == "light") return 0.20;
    if (level == "heavy") return 0.50;
    return 0.0;
}
}  // namespace

class SyntheticCameraPublisherNode : public rclcpp::Node {
public:
    SyntheticCameraPublisherNode() : Node("cs625_synthetic_camera")
    {
        this->declare_parameter("frame_id", "base_link");
        this->declare_parameter("publish_hz", 10.0);
        this->declare_parameter("target_x", 0.5);
        this->declare_parameter("target_y", 0.3);
        this->declare_parameter("target_z", 0.845);
        this->declare_parameter("box_size_x", 0.08);
        this->declare_parameter("box_size_y", 0.06);
        this->declare_parameter("box_size_z", 0.10);
        this->declare_parameter("surface_step", 0.01);
        this->declare_parameter("image_width", 160);
        this->declare_parameter("image_height", 120);
        this->declare_parameter("horizontal_fov", 1.047);
        this->declare_parameter("vertical_fov", 0.785);
        this->declare_parameter("min_range", 0.05);
        this->declare_parameter("max_range", 5.0);
        this->declare_parameter("view_dependent", false);
        this->declare_parameter("occlusion_level", "none");
        this->declare_parameter("depth_noise_std", 0.0);
        this->declare_parameter("random_seed", 625);

        // A matched-seed experiment changes random_seed before every episode.
        // Reset the view epoch on *every* update (including an update to the
        // same numeric value as the launch default), otherwise the noise key
        // would inherit a previous episode's view_index_ and no longer be
        // matched across strategies.
        parameter_callback_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto& parameter : parameters) {
                    if (parameter.get_name() == "random_seed") {
                        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                            result.successful = false;
                            result.reason = "random_seed must be an integer";
                            return result;
                        }
                        view_index_ = 0;
                        has_active_view_pose_ = false;
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Reset virtual sensor view epoch for random_seed=%ld",
                            parameter.as_int()
                        );
                    }
                }
                return result;
            }
        );

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/points", rclcpp::QoS(1).transient_local());
        model_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cs625_nbv/model_cloud", rclcpp::QoS(1).transient_local());
        visible_model_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cs625_nbv/visible_model_cloud", 10);
        virtual_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cs625_nbv/virtual_camera_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr pose) {
                active_view_pose_ = *pose;
                has_active_view_pose_ = true;
                ++view_index_;
            });

        buildSurfaceModel();
        publishModel();
        const auto period = std::chrono::duration<double>(
            1.0 / std::max(0.1, this->get_parameter("publish_hz").as_double()));
        timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
            [this]() { publishObservation(); });
        RCLCPP_WARN(this->get_logger(),
            "Publishing pose-aware z-buffered synthetic observations; virtual-only evidence.");
    }

private:
    void buildSurfaceModel()
    {
        target_center_ = Eigen::Vector3d(
            this->get_parameter("target_x").as_double(),
            this->get_parameter("target_y").as_double(),
            this->get_parameter("target_z").as_double());
        config_.cuboid_size = Eigen::Vector3d(
            this->get_parameter("box_size_x").as_double(),
            this->get_parameter("box_size_y").as_double(),
            this->get_parameter("box_size_z").as_double());
        config_.surface_step = this->get_parameter("surface_step").as_double();
        config_.image_width = this->get_parameter("image_width").as_int();
        config_.image_height = this->get_parameter("image_height").as_int();
        config_.horizontal_fov = this->get_parameter("horizontal_fov").as_double();
        config_.vertical_fov = this->get_parameter("vertical_fov").as_double();
        config_.min_range = this->get_parameter("min_range").as_double();
        config_.max_range = this->get_parameter("max_range").as_double();
        surface_ = cs625_nbv::VirtualObservationModel::build_cuboid_surface(target_center_, config_);
        model_cloud_ = toCloud(surface_);
        RCLCPP_INFO(this->get_logger(), "Built virtual cuboid model with %u points", model_cloud_.width);
    }

    sensor_msgs::msg::PointCloud2 toCloud(
        const std::vector<cs625_nbv::VirtualSurfacePoint>& points) const
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = this->get_parameter("frame_id").as_string();
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(points.size());
        cloud.is_dense = true;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
        for (const auto& point : points) {
            *x = static_cast<float>(point.position.x()); *y = static_cast<float>(point.position.y());
            *z = static_cast<float>(point.position.z()); ++x; ++y; ++z;
        }
        return cloud;
    }

    Eigen::Isometry3d activeCameraPose() const
    {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        if (has_active_view_pose_) {
            pose.translation() = Eigen::Vector3d(active_view_pose_.pose.position.x,
                active_view_pose_.pose.position.y, active_view_pose_.pose.position.z);
            Eigen::Quaterniond q(active_view_pose_.pose.orientation.w, active_view_pose_.pose.orientation.x,
                active_view_pose_.pose.orientation.y, active_view_pose_.pose.orientation.z);
            if (q.norm() > 1e-9) pose.linear() = q.normalized().toRotationMatrix();
            return pose;
        }
        pose.translation() = target_center_ + Eigen::Vector3d(-0.5, 0.0, 0.2);
        const Eigen::Vector3d z = (target_center_ - pose.translation()).normalized();
        Eigen::Vector3d x = Eigen::Vector3d::UnitY().cross(z);
        if (x.norm() < 1e-9) x = Eigen::Vector3d::UnitX().cross(z);
        x.normalize();
        pose.linear().col(0) = x; pose.linear().col(1) = z.cross(x); pose.linear().col(2) = z;
        return pose;
    }

    std::vector<cs625_nbv::VirtualSurfacePoint> buildObservation(bool applyNoise) const
    {
        if (!this->get_parameter("view_dependent").as_bool()) return surface_;
        const Eigen::Isometry3d camera = activeCameraPose();
        const auto visible = cs625_nbv::VirtualObservationModel::zbuffer_visible_indices(surface_, camera, config_);
        const double blocked = occlusionFraction(this->get_parameter("occlusion_level").as_string());
        const auto seed = static_cast<uint32_t>(this->get_parameter("random_seed").as_int()) + 7919U * view_index_;
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> keep(0.0, 1.0);
        std::vector<cs625_nbv::VirtualSurfacePoint> observation;
        observation.reserve(visible.size());
        for (const auto index : visible) if (keep(rng) >= blocked) observation.push_back(surface_[index]);
        const double visible_ratio = static_cast<double>(observation.size()) / std::max<std::size_t>(1, surface_.size());
        const double base_noise = std::max(0.0, this->get_parameter("depth_noise_std").as_double());
        std::normal_distribution<double> noise(0.0, base_noise * (1.0 + 4.0 * (1.0 - visible_ratio)));
        if (applyNoise && base_noise > 0.0) {
            for (auto& point : observation) {
                const Eigen::Vector3d ray = point.position - camera.translation();
                if (ray.norm() > 1e-9) point.position += noise(rng) * ray.normalized();
            }
        }
        return observation;
    }

    void publishModel() { model_cloud_.header.stamp = this->now(); model_pub_->publish(model_cloud_); }
    void publishObservation()
    {
        publishModel();
        auto visible = toCloud(buildObservation(false)); visible.header.stamp = this->now(); visible_model_pub_->publish(visible);
        auto cloud = toCloud(buildObservation(true)); cloud.header.stamp = this->now(); cloud_pub_->publish(cloud);
    }

    Eigen::Vector3d target_center_{Eigen::Vector3d::Zero()};
    cs625_nbv::VirtualObservationConfig config_;
    std::vector<cs625_nbv::VirtualSurfacePoint> surface_;
    sensor_msgs::msg::PointCloud2 model_cloud_;
    geometry_msgs::msg::PoseStamped active_view_pose_;
    bool has_active_view_pose_{false}; uint32_t view_index_{0};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_, model_pub_, visible_model_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr virtual_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyntheticCameraPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
