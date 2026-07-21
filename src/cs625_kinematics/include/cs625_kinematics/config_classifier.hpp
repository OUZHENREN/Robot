#pragma once

#include <array>
#include <cstdint>

namespace cs625_kinematics {

struct ConfigFlags {
    int8_t shoulder; // 0 Front, 1 Rear, -1 unknown
    int8_t elbow;    // 0 Up,    1 Down, -1 unknown
    int8_t wrist;    // 0 NonFlip, 1 Flip, -1 unknown
};

constexpr int8_t CONFIG_UNKNOWN = -1;
constexpr int8_t CONFIG_FRONT = 0;
constexpr int8_t CONFIG_REAR = 1;
constexpr int8_t CONFIG_UP = 0;
constexpr int8_t CONFIG_DOWN = 1;
constexpr int8_t CONFIG_NONFLIP = 0;
constexpr int8_t CONFIG_FLIP = 1;

// Heuristic classification for IK candidate labeling.
// This is intended for engineering use and debugging, not a guaranteed
// manufacturer-exact branch definition.
ConfigFlags classifyConfig(const std::array<double,6>& q);

}  // namespace cs625_kinematics
