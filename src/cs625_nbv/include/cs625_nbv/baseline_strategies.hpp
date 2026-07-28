#pragma once

#include <vector>
#include <random>
#include <cstdint>
#include "cs625_nbv/msg/viewpoint_candidate.hpp"

namespace cs625_nbv {

/**
 * @brief Baseline NBV strategies for comparison experiments.
 *
 * Each strategy takes a list of reachable candidates and returns
 * the selected viewpoint index. The orchestrator executes the
 * selected viewpoint before calling the strategy again.
 */
class BaselineStrategies {
public:
    /// Strategy type enum
    enum Type {
        SINGLE_VIEW = 0,
        FIXED_ORDER = 1,
        RANDOM_REACHABLE = 2,
        COVERAGE_GREEDY = 3,
        UNCERTAINTY_ONLY = 4,
        POSE_GAIN = 5,  // The proposed method
        PATH_COST_ONLY = 6,
    };

    explicit BaselineStrategies(uint32_t random_seed = 625U);

    /// Reset the random baseline to a reproducible seed.
    void set_random_seed(uint32_t random_seed) { rng_.seed(random_seed); }

    /**
     * @brief Select the next viewpoint using the given strategy.
     *
     * @param type        Strategy type.
     * @param candidates  List of reachable candidates (sorted by utility for some strategies).
     * @param step        Current step index (0-based, number of views already taken).
     * @return            Index into candidates of the selected viewpoint.
     *                    Returns -1 if no valid candidate.
     */
    int select_next(Type type,
                    std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates,
                    int step);

    /**
     * @brief Select next viewpoint by strategy name string.
     */
    int select_by_name(const std::string& strategy_name,
                       std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates,
                       int step);

private:
    std::mt19937 rng_;

    // Individual strategy implementations
    int select_single_view(const std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates, int step);
    int select_fixed_order(const std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates, int step);
    int select_random_reachable(const std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates);
    int select_coverage_greedy(std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates);
    int select_uncertainty_only(std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates);
    int select_pose_gain(std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates);
    int select_path_cost_only(const std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates);
};

}  // namespace cs625_nbv
