#include "cs625_kinematics/config_classifier.hpp"

#include <cmath>

namespace cs625_kinematics {

ConfigFlags classifyConfig(const std::array<double,6>& q)
{
    ConfigFlags cfg;
    cfg.shoulder = CONFIG_UNKNOWN;
    cfg.elbow = CONFIG_UNKNOWN;
    cfg.wrist = CONFIG_UNKNOWN;

    // -------------------------------------------------------------------------
    // Heuristic rules:
    // shoulder: split by joint-1 half-plane
    // elbow   : split by sign of joint-3
    // wrist   : split by sign of joint-5
    //
    // These rules are intentionally simple for stage-C candidate labeling.
    // They can be replaced later by geometry-based branch classification.
    // -------------------------------------------------------------------------

    if (std::isfinite(q[0])) {
        cfg.shoulder = (std::cos(q[0]) >= 0.0) ? CONFIG_FRONT : CONFIG_REAR;
    }

    if (std::isfinite(q[2])) {
        cfg.elbow = (q[2] >= 0.0) ? CONFIG_DOWN : CONFIG_UP;
    }

    if (std::isfinite(q[4])) {
        cfg.wrist = (q[4] >= 0.0) ? CONFIG_FLIP : CONFIG_NONFLIP;
    }

    return cfg;
}

}  // namespace cs625_kinematics
