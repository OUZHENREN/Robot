#pragma once

#include <array>
#include <Eigen/Dense>

namespace cs625_kinematics {

// -----------------------------------------------------------------------------
// FK conventions for C stage
// - base -> flange
// - base -> my_end_effector_link
// In this project, my_end_effector_link is the same fixed offset as the
// previously used "tool" transform.
// -----------------------------------------------------------------------------

// base -> flange
Eigen::Matrix4d fk_cs625_flange(const std::array<double, 6>& q);

// flange -> my_end_effector_link
Eigen::Matrix4d flange_to_my_end_effector();

// base -> my_end_effector_link
Eigen::Matrix4d fk_cs625_ee(const std::array<double, 6>& q);

// Backward-compatible interface:
// include_tool = false -> base -> flange
// include_tool = true  -> base -> my_end_effector_link
Eigen::Matrix4d fk_cs625(const std::array<double, 6>& q, bool include_tool = false);

// Pose error helpers
double positionError(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b);
double orientationError(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b);

}  // namespace cs625_kinematics
