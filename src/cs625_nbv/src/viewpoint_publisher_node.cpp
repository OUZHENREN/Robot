/**
 * @file viewpoint_publisher_node.cpp
 * @brief Publishes NBV candidate viewpoints as RViz MarkerArray for visualization.
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "cs625_nbv/viewpoint_sampler.hpp"
#include "cs625_nbv/msg/viewpoint_candidate.hpp"

using namespace std::chrono_literals;

class ViewpointPublisherNode : public rclcpp::Node {
public:
    ViewpointPublisherNode()
        : Node("cs625_nbv_viewpoint_publisher")
    {
        this->declare_parameter("view_distance", 0.5);
        this->declare_parameter("sample_count", 42);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cs625_nbv/candidate_views", 10
        );

        timer_ = this->create_wall_timer(2s, [this]() { publish_viewpoints(); });

        RCLCPP_INFO(this->get_logger(), "Viewpoint publisher initialized");
    }

private:
    void publish_viewpoints()
    {
        double view_dist = this->get_parameter("view_distance").as_double();
        int count = this->get_parameter("sample_count").as_int();

        cs625_nbv::CameraModel camera;
        cs625_nbv::ViewpointSampler sampler(camera);

        // Publish candidates around a fixed target
        Eigen::Vector3d target(0.5, 0.3, 0.845);
        auto candidates = sampler.generate_candidates(target, view_dist, count);

        visualization_msgs::msg::MarkerArray markers;

        for (size_t i = 0; i < candidates.size(); ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "base_link";
            m.header.stamp = this->now();
            m.ns = "nbv_candidates";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.action = visualization_msgs::msg::Marker::ADD;

            // Position: camera location
            m.pose = candidates[i].pose;

            // Arrow scale: 5cm shaft, 2cm head
            m.scale.x = 0.08;
            m.scale.y = 0.015;
            m.scale.z = 0.015;

            // Color: green for candidate views
            m.color.r = 0.0;
            m.color.g = 0.8;
            m.color.b = 0.0;
            m.color.a = 0.7;

            markers.markers.push_back(m);
        }

        marker_pub_->publish(markers);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViewpointPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
