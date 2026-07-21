#include <gtest/gtest.h>

#include <array>

#include "cs625_kinematics/fk_cs625.hpp"
#include "cs625_kinematics/ik_solver.hpp"

namespace cs625_kinematics {

TEST(TestIkSingle, SolveFromKnownPoseWithSeed)
{
    IkSolver solver;

    const std::array<double,6> q_ref = {0.2, -1.0, 1.1, -0.6, 0.7, -0.2};
    const Eigen::Matrix4d target_T = fk_cs625_ee(q_ref);

    std::array<double,6> q_sol{};
    const bool ok = solver.solveIK(target_T, &q_ref, q_sol);

    ASSERT_TRUE(ok);

    const Eigen::Matrix4d T_sol = fk_cs625_ee(q_sol);
    EXPECT_LT(positionError(T_sol, target_T), 1e-4);
    EXPECT_LT(orientationError(T_sol, target_T), 1e-3);
}

TEST(TestIkSingle, SolveWithoutSeed)
{
    IkSolver solver;

    const std::array<double,6> q_ref = {0.0, -1.2, 1.0, -1.0, 0.3, 0.4};
    const Eigen::Matrix4d target_T = fk_cs625_ee(q_ref);

    std::array<double,6> q_sol{};
    const bool ok = solver.solveIK(target_T, nullptr, q_sol);

    ASSERT_TRUE(ok);

    const Eigen::Matrix4d T_sol = fk_cs625_ee(q_sol);
    EXPECT_LT(positionError(T_sol, target_T), 1e-4);
    EXPECT_LT(orientationError(T_sol, target_T), 1e-3);
}

TEST(TestIkSingle, FailOnClearlyInvalidPose)
{
    IkSolver solver;

    Eigen::Matrix4d target_T = Eigen::Matrix4d::Identity();
    target_T(0, 3) = 100.0;
    target_T(1, 3) = 100.0;
    target_T(2, 3) = 100.0;

    std::array<double,6> q_sol{};
    const bool ok = solver.solveIK(target_T, nullptr, q_sol);

    EXPECT_FALSE(ok);
}

}  // namespace cs625_kinematics
