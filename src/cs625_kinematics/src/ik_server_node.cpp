#include <memory>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "cs625_kinematics/ik_solver.hpp"
#include "cs625_kinematics/ros_conversions.hpp"

#include "cs625_kinematics/srv/solve_ik.hpp"
#include "cs625_kinematics/srv/solve_ik_all.hpp"
#include "cs625_kinematics/msg/ik_solution.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace cs625_kinematics {

class IkServerNode : public rclcpp::Node
{
public:
    IkServerNode()
    : Node("cs625_ik_server"),
      has_current_state_(false)
    {
        RCLCPP_INFO(this->get_logger(), "cs625_ik_server node started. Target frame is my_end_effector_link.");

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&IkServerNode::jointStateCallback, this, _1));

        solve_ik_srv_ = this->create_service<cs625_kinematics::srv::SolveIK>(
            "solve_ik",
            std::bind(&IkServerNode::solveIkCallback, this, _1, _2));

        solve_ik_all_srv_ = this->create_service<cs625_kinematics::srv::SolveIKAll>(
            "solve_ik_all",
            std::bind(&IkServerNode::solveIkAllCallback, this, _1, _2));
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        has_current_state_ = jointStateToArray(*msg, q_current_);
    }

    void solveIkCallback(
        const cs625_kinematics::srv::SolveIK::Request::SharedPtr req,
        cs625_kinematics::srv::SolveIK::Response::SharedPtr res)
    {
        const Eigen::Matrix4d T = poseStampedToEigen(req->target_pose);

        std::array<double,6> seed_arr{};
        std::array<double,6>* seed_ptr = nullptr;

        if (jointStateToArray(req->seed, seed_arr)) {
            seed_ptr = &seed_arr;
        } else if (has_current_state_) {
            seed_arr = q_current_;
            seed_ptr = &seed_arr;
        }

        std::array<double,6> q_solution{};
        const bool ok = ik_solver_.solveIK(T, seed_ptr, q_solution);

        res->success = ok;
        if (!ok) {
            RCLCPP_WARN(this->get_logger(), "solve_ik failed.");
            return;
        }

        if (!req->seed.name.empty()) {
            res->solution = arrayToJointState(
                q_solution,
                std::vector<std::string>(req->seed.name.begin(), req->seed.name.end()));
        } else {
            res->solution = arrayToJointState(q_solution, defaultJointNames());
        }
    }

    void solveIkAllCallback(
        const cs625_kinematics::srv::SolveIKAll::Request::SharedPtr req,
        cs625_kinematics::srv::SolveIKAll::Response::SharedPtr res)
    {
        const Eigen::Matrix4d T = poseStampedToEigen(req->target_pose);

        std::array<double,6> seed_arr{};
        std::array<double,6>* seed_ptr = nullptr;

        if (jointStateToArray(req->seed, seed_arr)) {
            seed_ptr = &seed_arr;
        } else if (has_current_state_) {
            seed_arr = q_current_;
            seed_ptr = &seed_arr;
        }

        std::vector<IkSolutionInternal> internal_solutions;
        const bool ok = ik_solver_.solveIKAll(T, req->max_solutions, seed_ptr, internal_solutions);

        res->success = ok;
        res->solutions.clear();

        if (!ok) {
            RCLCPP_WARN(this->get_logger(), "solve_ik_all failed.");
            return;
        }

        for (const auto& sol : internal_solutions) {
            cs625_kinematics::msg::IkSolution msg_sol;
            for (size_t i = 0; i < 6; ++i) {
                msg_sol.joints[i] = sol.joints[i];
            }
            msg_sol.shoulder = sol.shoulder;
            msg_sol.elbow = sol.elbow;
            msg_sol.wrist = sol.wrist;
            msg_sol.cost = sol.cost;
            res->solutions.push_back(msg_sol);
        }
    }

    IkSolver ik_solver_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Service<cs625_kinematics::srv::SolveIK>::SharedPtr solve_ik_srv_;
    rclcpp::Service<cs625_kinematics::srv::SolveIKAll>::SharedPtr solve_ik_all_srv_;

    std::array<double,6> q_current_{};
    bool has_current_state_;
};

}  // namespace cs625_kinematics

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cs625_kinematics::IkServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
