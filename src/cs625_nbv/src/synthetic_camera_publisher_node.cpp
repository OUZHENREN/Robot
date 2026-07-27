/**
 * @file synthetic_camera_publisher_node.cpp
 * @brief GPU-independent synthetic point-cloud source for remote NBV development.
 *
 * Publishes a deterministic cuboid surface in base_link. It is intended only
 * for exercising the ROS, PCL and NBV software pipeline when Gazebo rendering
 * sensors are unavailable (for example, on a VMware host without 3D support).
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class SyntheticCameraPublisherNode : public rclcpp::Node {
public:
    SyntheticCameraPublisherNode()
        : Node("cs625_synthetic_camera")
    {
        this->declare_parameter("frame_id", "base_link");
        this->declare_parameter("publish_hz", 2.0);
        this->declare_parameter("target_x", 0.5);
        this->declare_parameter("target_y", 0.3);
        this->declare_parameter("target_z", 0.845);
        this->declare_parameter("box_size_x", 0.08);
        this->declare_parameter("box_size_y", 0.06);
        this->declare_parameter("box_size_z", 0.10);
        this->declare_parameter("surface_step", 0.01);

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/points", rclcpp::QoS(1).transient_local()
        );

        build_cloud();
        const auto publish_hz = this->get_parameter("publish_hz").as_double();
        const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_hz));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            [this]() { publish_cloud(); }
        );

        RCLCPP_WARN(
            this->get_logger(),
            "Publishing synthetic /camera/points for software-only NBV validation; "
            "do not use its metrics as real-camera experiment results"
        );
    }

private:
    void build_cloud()
    {
        const double cx = this->get_parameter("target_x").as_double();
        const double cy = this->get_parameter("target_y").as_double();
        const double cz = this->get_parameter("target_z").as_double();
        const double sx = this->get_parameter("box_size_x").as_double() / 2.0;
        const double sy = this->get_parameter("box_size_y").as_double() / 2.0;
        const double sz = this->get_parameter("box_size_z").as_double() / 2.0;
        const double step = std::max(0.002, this->get_parameter("surface_step").as_double());

        std::vector<std::array<float, 3>> points;
        const auto append_point = [&points](double x, double y, double z) {
            points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
        };

        for (double x = -sx; x <= sx + 1e-9; x += step) {
            for (double y = -sy; y <= sy + 1e-9; y += step) {
                append_point(cx + x, cy + y, cz - sz);
                append_point(cx + x, cy + y, cz + sz);
            }
        }
        for (double x = -sx; x <= sx + 1e-9; x += step) {
            for (double z = -sz; z <= sz + 1e-9; z += step) {
                append_point(cx + x, cy - sy, cz + z);
                append_point(cx + x, cy + sy, cz + z);
            }
        }
        for (double y = -sy; y <= sy + 1e-9; y += step) {
            for (double z = -sz; z <= sz + 1e-9; z += step) {
                append_point(cx - sx, cy + y, cz + z);
                append_point(cx + sx, cy + y, cz + z);
            }
        }

        cloud_.header.frame_id = this->get_parameter("frame_id").as_string();
        cloud_.height = 1;
        cloud_.width = static_cast<uint32_t>(points.size());
        cloud_.is_dense = true;
        sensor_msgs::PointCloud2Modifier modifier(cloud_);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_, "z");
        for (const auto& point : points) {
            *iter_x = point[0];
            *iter_y = point[1];
            *iter_z = point[2];
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        RCLCPP_INFO(this->get_logger(), "Built synthetic cuboid cloud with %u points", cloud_.width);
    }

    void publish_cloud()
    {
        cloud_.header.stamp = this->now();
        cloud_pub_->publish(cloud_);
    }

    sensor_msgs::msg::PointCloud2 cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyntheticCameraPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
