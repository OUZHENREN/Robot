#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <sstream>
#include <vector>
#include <cmath>

class TcpPoseReceiver : public rclcpp::Node
{
public:
    TcpPoseReceiver() : Node("tcp_pose_receiver_node")
    {
        // 1. 创建一个订阅者，监听从Python服务器转发过来的 "/tcp_data" 话题
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "tcp_data", 10, std::bind(&TcpPoseReceiver::topic_callback, this, std::placeholders::_1));

        // 2. 创建一个发布者，用于发布给RViz看的 "/target_pose" 话题
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);

        RCLCPP_INFO(this->get_logger(), "Pose Receiver is running and waiting for 6 values (x,y,z,Rx,Ry,Rz) on '/tcp_data' topic.");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received data: '%s'", msg->data.c_str());

        std::stringstream ss(msg->data);
        std::string item;
        std::vector<double> pose_data;
        
        while (std::getline(ss, item, ' '))
        {
            try
            {
                pose_data.push_back(std::stod(item));
            }
            catch (const std::invalid_argument& ia)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid number in data string: %s", item.c_str());
                return;
            }
        }

        // 修改：现在期望接收6个值 (x,y,z,Rx,Ry,Rz) 而不是7个
        if (pose_data.size() != 6)
        {
            RCLCPP_ERROR(this->get_logger(), "Received data does not contain 6 values (x,y,z,Rx,Ry,Rz). Received %zu values.", pose_data.size());
            return;
        }

        RCLCPP_INFO(this->get_logger(), 
            "Parsed: Position(%f mm, %f mm, %f mm), Rotation(%f°, %f°, %f°)", 
            pose_data[0], pose_data[1], pose_data[2], pose_data[3], pose_data[4], pose_data[5]);

        // 转换6个值为ROS位姿
        auto pose_stamped_msg = convert_to_pose_stamped(pose_data[0], pose_data[1], pose_data[2], 
                                                       pose_data[3], pose_data[4], pose_data[5]);

        // 发布消息
        pose_publisher_->publish(std::move(pose_stamped_msg));
        RCLCPP_INFO(this->get_logger(), "Pose message published successfully!");
    }

    geometry_msgs::msg::PoseStamped convert_to_pose_stamped(double x_mm, double y_mm, double z_mm, 
                                                           double Rx_deg, double Ry_deg, double Rz_deg)
    {
        auto message = geometry_msgs::msg::PoseStamped();

        // 关键：设置消息头
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "base_link"; // 或者 "base"，与机器人TF树保持一致

        // 单位转换：毫米 -> 米
        message.pose.position.x = x_mm / 1000.0;
        message.pose.position.y = y_mm / 1000.0;
        message.pose.position.z = z_mm / 1000.0;

        // 单位转换：度 -> 弧度
        double roll = Rx_deg * M_PI / 180.0;
        double pitch = Ry_deg * M_PI / 180.0;
        double yaw = Rz_deg * M_PI / 180.0;

        // 欧拉角转四元数 (ZYX顺序)
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        message.pose.orientation.w = cr * cp * cy + sr * sp * sy;
        message.pose.orientation.x = sr * cp * cy - cr * sp * sy;
        message.pose.orientation.y = cr * sp * cy + sr * cp * sy;
        message.pose.orientation.z = cr * cp * sy - sr * sp * cy;

        RCLCPP_INFO(this->get_logger(), 
            "Converted to: Position(%f m, %f m, %f m)", 
            message.pose.position.x, message.pose.position.y, message.pose.position.z);

        return message;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TcpPoseReceiver>());
    rclcpp::shutdown();
    return 0;
}

