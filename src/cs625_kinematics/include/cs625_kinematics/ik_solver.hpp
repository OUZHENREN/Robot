#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

namespace cs625_kinematics {

struct IkSolutionInternal {
    std::array<double,6> joints{};
    double cost{0.0};
    int8_t shoulder{-1};
    int8_t elbow{-1};
    int8_t wrist{-1};
    bool valid{true};
};

class IkSolver {
public:
    IkSolver();

    bool solveIK(const Eigen::Matrix4d& target_T,
                 const std::array<double,6>* seed,
                 std::array<double,6>& q_solution);

    bool solveIKAll(const Eigen::Matrix4d& target_T,
                    int max_solutions,
                    const std::array<double,6>* seed,
                    std::vector<IkSolutionInternal>& solutions);

private:
    bool solveIKNumerical(const Eigen::Matrix4d& target_T,
                          const std::array<double,6>& seed,
                          std::array<double,6>& q_solution);

    bool validateSolution(const Eigen::Matrix4d& target_T,
                          const std::array<double,6>& q_solution) const;

    bool isWithinJointLimits(const std::array<double,6>& q) const;

    std::array<double,6> normalizeJoints(const std::array<double,6>& q) const;

    std::vector<std::array<double,6>> generateSeeds(
        const std::array<double,6>* seed,
        bool aggressive_search) const;

    double solutionCost(const std::array<double,6>& q,
                        const std::array<double,6>* reference) const;

    bool isDuplicate(const std::array<double,6>& a,
                     const std::array<double,6>& b,
                     double tol = 1e-3) const;
};

}  // namespace cs625_kinematics
