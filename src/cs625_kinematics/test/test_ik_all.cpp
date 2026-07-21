#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "cs625_kinematics/fk_cs625.hpp"
#include "cs625_kinematics/ik_solver.hpp"

namespace cs625_kinematics {

TEST(TestIkAll, ReturnAtLeastOneValidSolution)
{
    IkSolver solver;

    const std::array<double,6> q_ref = {0.4, -1.1, 1.2, -0.8, 0.5, 0.1};
    const Eigen::Matrix4d target_T = fk_cs625_ee(q_ref);

    std::vector<IkSolutionInternal> solutions;
    const bool ok = solver.solveIKAll(target_T, 8, &q_ref, solutions);

    ASSERT_TRUE(ok);
    ASSERT_FALSE(solutions.empty());

    for (const auto& sol : solutions) {
        const Eigen::Matrix4d T_sol = fk_cs625_ee(sol.joints);
        EXPECT_LT(positionError(T_sol, target_T), 1e-4);
        EXPECT_LT(orientationError(T_sol, target_T), 1e-3);
    }
}

TEST(TestIkAll, RespectMaxSolutions)
{
    IkSolver solver;

    const std::array<double,6> q_ref = {0.2, -0.8, 0.9, -0.7, 0.4, -0.3};
    const Eigen::Matrix4d target_T = fk_cs625_ee(q_ref);

    std::vector<IkSolutionInternal> solutions;
    const bool ok = solver.solveIKAll(target_T, 1, &q_ref, solutions);

    ASSERT_TRUE(ok);
    ASSERT_LE(solutions.size(), 1u);
}

TEST(TestIkAll, SolutionsSortedByCost)
{
    IkSolver solver;

    const std::array<double,6> q_ref = {0.1, -1.0, 1.0, -0.5, 0.2, -0.1};
    const Eigen::Matrix4d target_T = fk_cs625_ee(q_ref);

    std::vector<IkSolutionInternal> solutions;
    const bool ok = solver.solveIKAll(target_T, 8, &q_ref, solutions);

    ASSERT_TRUE(ok);
    ASSERT_FALSE(solutions.empty());

    for (size_t i = 1; i < solutions.size(); ++i) {
        EXPECT_LE(solutions[i - 1].cost, solutions[i].cost);
    }
}

}  // namespace cs625_kinematics
