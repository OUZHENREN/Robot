/**
 * @file synthetic_camera_publisher_node.cpp
 * @brief Deterministic viewpoint-dependent virtual RGB-D observation source.
 *
 * This node publishes a complete cuboid model cloud and a partial observation
 * cloud. A pose received on /cs625_nbv/virtual_camera_pose changes the visible
 * faces, deterministic occlusion pattern, and optional depth noise. The output
 * remains software-only synthetic data and must be labelled accordingly.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace {

struct SurfacePoint {
    std::array<float, 3> position;
    std::array<float, 3> normal;
};

double occlusionFraction(const std::string& level)
{
    if (level == "none") return 0.0;
    if (level == "light") return 0.20;
    if (level == "heavy") return 0.50;
    return 0.0;
}

}  // namespace

class SyntheticCameraPublisherNode : public rclcpp::Node {
public:
    SyntheticCameraPublisherNode()
        : Node("cs625_synthetic_camera")
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
        this->declare_parameter("view_dependent", false);
        this->declare_parameter("occlusion_level", "none");
        this->declare_parameter("depth_noise_std", 0.0);
        this->declare_parameter("random_seed", 625);

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/points", rclcpp::QoS(1).transient_local()
        );
        model_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cs625_nbv/model_cloud", rclcpp::QoS(1).transient_local()
        );
        visible_model_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cs625_nbv/visible_model_cloud", 10
        );
        virtual_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cs625_nbv/virtual_camera_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr pose) {
                active_view_pose_ = *pose;
                has_active_view_pose_ = true;
                ++view_index_;
            }
        );

        build_surface_model();
        publish_model();
        const auto publish_hz = this->get_parameter("publish_hz").as_double();
        const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_hz));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            [this]() { publish_observation(); }
        );

        RCLCPP_WARN(
            this->get_logger(),
            "Publishing synthetic observation clouds. Set data_source to "
            "synthetic_view_dependent and retain simulation-only caveats."
        );
    }

private:
    void build_surface_model()
    {
        const double cx = this->get_parameter("target_x").as_double();
        const double cy = this->get_parameter("target_y").as_double();
        const double cz = this->get_parameter("target_z").as_double();
        const double sx = this->get_parameter("box_size_x").as_double() / 2.0;
        const double sy = this->get_parameter("box_size_y").as_double() / 2.0;
        const double sz = this->get_parameter("box_size_z").as_double() / 2.0;
        const double step = std::max(0.002, this->get_parameter("surface_step").as_double());

        target_center_ = {cx, cy, cz};
        const auto append = [this](double x, double y, double z,
                                   double nx, double ny, double nz) {
            surface_points_.push_back({
                {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)},
                {static_cast<float>(nx), static_cast<float>(ny), static_cast<float>(nz)}
            });
        };

        for (double x = -sx; x <= sx + 1e-9; x += step) {
            for (double y = -sy; y <= sy + 1e-9; y += step) {
                append(cx + x, cy + y, cz - sz, 0, 0, -1);
                append(cx + x, cy + y, cz + sz, 0, 0, 1);
            }
        }
        for (double x = -sx; x <= sx + 1e-9; x += step) {
            for (double z = -sz; z <= sz + 1e-9; z += step) {
                append(cx + x, cy - sy, cz + z, 0, -1, 0);
                append(cx + x, cy + sy, cz + z, 0, 1, 0);
            }
        }
        for (double y = -sy; y <= sy + 1e-9; y += step) {
            for (double z = -sz; z <= sz + 1e-9; z += step) {
                append(cx - sx, cy + y, cz + z, -1, 0, 0);
                append(cx + sx, cy + y, cz + z, 1, 0, 0);
            }
        }

        model_cloud_ = toCloud(surface_points_);
        RCLCPP_INFO(this->get_logger(), "Built complete virtual cuboid model with %u points", model_cloud_.width);
    }

    sensor_msgs::msg::PointCloud2 toCloud(const std::vector<SurfacePoint>& points) const
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = this->get_parameter("frame_id").as_string();
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(points.size());
        cloud.is_dense = true;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        for (const auto& point : points) {
            *iter_x = point.position[0];
            *iter_y = point.position[1];
            *iter_z = point.position[2];
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
        return cloud;
    }

    void publish_model()
    {
        model_cloud_.header.stamp = this->now();
        model_pub_->publish(model_cloud_);
    }

    std::array<double, 3> activeCameraPosition() const
    {
        if (has_active_view_pose_) {
            return {
                active_view_pose_.pose.position.x,
                active_view_pose_.pose.position.y,
                active_view_pose_.pose.position.z
            };
        }
        return {target_center_[0] - 0.5, target_center_[1], target_center_[2] + 0.2};
    }

    std::vector<SurfacePoint> buildObservation(bool apply_noise) const
    {
        if (!this->get_parameter("view_dependent").as_bool()) {
            return surface_points_;
        }

        const auto camera = activeCameraPosition();
        const double blocked_fraction = occlusionFraction(
            this->get_parameter("occlusion_level").as_string()
        );
        const double base_noise_std = std::max(
            0.0, this->get_parameter("depth_noise_std").as_double()
        );
        const auto seed = static_cast<uint32_t>(this->get_parameter("random_seed").as_int())
            + 7919U * view_index_;
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> keep_draw(0.0, 1.0);

        std::vector<SurfacePoint> observation;
        observation.reserve(surface_points_.size() / 2);
        for (const auto& point : surface_points_) {
            const double vx = camera[0] - target_center_[0];
            const double vy = camera[1] - target_center_[1];
            const double vz = camera[2] - target_center_[2];
            const double facing = point.normal[0] * vx + point.normal[1] * vy + point.normal[2] * vz;
            if (facing <= 1e-9 || keep_draw(rng) < blocked_fraction) {
                continue;
            }

            SurfacePoint noisy = point;
            observation.push_back(noisy);
        }

        // A partially occluded RGB-D observation is not merely smaller: it
        // has fewer geometric constraints and a less stable depth fit. Model
        // that explicitly in the virtual sensor so the pre-observation
        // visibility predictor has a measurable, documented consequence.
        const double visible_ratio = static_cast<double>(observation.size()) /
            std::max<size_t>(1U, surface_points_.size());
        const double effective_noise_std = base_noise_std *
            (1.0 + 4.0 * (1.0 - visible_ratio));
        std::normal_distribution<double> noise(0.0, effective_noise_std);
        if (apply_noise && effective_noise_std > 0.0) {
            for (auto& noisy : observation) {
                const auto& point = noisy;
                const double dx = point.position[0] - camera[0];
                const double dy = point.position[1] - camera[1];
                const double dz = point.position[2] - camera[2];
                const double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (norm <= 1e-9) continue;
                const double perturbation = noise(rng);
                noisy.position[0] += static_cast<float>(perturbation * dx / norm);
                noisy.position[1] += static_cast<float>(perturbation * dy / norm);
                noisy.position[2] += static_cast<float>(perturbation * dz / norm);
            }
        }
        return observation;
    }

    void publish_observation()
    {
        // Re-publish the durable model as well: CLI readiness checks may join
        // after construction, and this avoids a discovery-timing deadlock.
        publish_model();
        const auto visible_model = buildObservation(false);
        auto visible_model_cloud = toCloud(visible_model);
        visible_model_cloud.header.stamp = this->now();
        visible_model_pub_->publish(visible_model_cloud);
        const auto observation = buildObservation(true);
        auto cloud = toCloud(observation);
        cloud.header.stamp = this->now();
        cloud_pub_->publish(cloud);
    }

    std::array<double, 3> target_center_{};
    std::vector<SurfacePoint> surface_points_;
    sensor_msgs::msg::PointCloud2 model_cloud_;
    geometry_msgs::msg::PoseStamped active_view_pose_;
    bool has_active_view_pose_{false};
    uint32_t view_index_{0};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr model_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visible_model_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr virtual_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyntheticCameraPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
