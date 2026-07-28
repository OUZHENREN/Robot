#include "cs625_nbv/baseline_strategies.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace cs625_nbv {

BaselineStrategies::BaselineStrategies(uint32_t random_seed)
    : rng_(random_seed) {}

int BaselineStrategies::select_next(
    Type type,
    std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates,
    int step)
{
    if (candidates.empty()) return -1;

    switch (type) {
        case SINGLE_VIEW:        return select_single_view(candidates, step);
        case FIXED_ORDER:        return select_fixed_order(candidates, step);
        case RANDOM_REACHABLE:   return select_random_reachable(candidates);
        case COVERAGE_GREEDY:    return select_coverage_greedy(candidates);
        case UNCERTAINTY_ONLY:   return select_uncertainty_only(candidates);
        case POSE_GAIN:          return select_pose_gain(candidates);
        case PATH_COST_ONLY:     return select_path_cost_only(candidates);
        default: return -1;
    }
}

int BaselineStrategies::select_by_name(
    const std::string& strategy_name,
    std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates,
    int step)
{
    if (strategy_name == "single_view")        return select_single_view(candidates, step);
    if (strategy_name == "fixed_order")        return select_fixed_order(candidates, step);
    if (strategy_name == "random_reachable")   return select_random_reachable(candidates);
    if (strategy_name == "coverage_greedy")    return select_coverage_greedy(candidates);
    if (strategy_name == "uncertainty_only")   return select_uncertainty_only(candidates);
    if (strategy_name == "pose_gain")          return select_pose_gain(candidates);
    if (strategy_name == "path_cost_only")     return select_path_cost_only(candidates);
    return -1;
}

// ---------------------------------------------------------------------------
// Strategy 0: Single View — always return the first candidate
// ---------------------------------------------------------------------------
int BaselineStrategies::select_single_view(
    const std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates, int step)
{
    if (step > 0) return -1;  // Only the first view
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].reachable) return static_cast<int>(i);
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Strategy 1: Fixed Order — iterate through candidates in order
// ---------------------------------------------------------------------------
int BaselineStrategies::select_fixed_order(
    const std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates, int step)
{
    // Pick the step-th reachable candidate
    int reachable_count = 0;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].reachable) {
            if (reachable_count == step) return static_cast<int>(i);
            reachable_count++;
        }
    }
    return -1;  // No more reachable candidates
}

// ---------------------------------------------------------------------------
// Strategy 2: Random Reachable — uniform random from reachable set
// ---------------------------------------------------------------------------
int BaselineStrategies::select_random_reachable(
    const std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates)
{
    // Collect reachable indices
    std::vector<int> reachable_idx;
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        if (candidates[i].reachable) reachable_idx.push_back(i);
    }
    if (reachable_idx.empty()) return -1;

    std::uniform_int_distribution<size_t> dist(0, reachable_idx.size() - 1);
    return reachable_idx[dist(rng_)];
}

// ---------------------------------------------------------------------------
// Strategy 3: Coverage Greedy — maximize newly visible surface
// ---------------------------------------------------------------------------
int BaselineStrategies::select_coverage_greedy(
    std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates)
{
    // Simplified: select the candidate with the largest viewing angle
    // (assumes wider angle = more surface visible).
    // In the full implementation, this ray-traces or uses a voxel occupancy grid.
    int best_idx = -1;
    double best_angle = -1.0;

    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        if (!candidates[i].reachable) continue;

        // Use information gain as proxy for coverage potential
        // (higher cosine with surface = more visible area)
        if (candidates[i].information_gain > best_angle) {
            best_angle = candidates[i].information_gain;
            best_idx = i;
        }
    }
    return best_idx;
}

// ---------------------------------------------------------------------------
// Strategy 4: Uncertainty Only — select max IG, ignore path cost
// ---------------------------------------------------------------------------
int BaselineStrategies::select_uncertainty_only(
    std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates)
{
    int best_idx = -1;
    double best_ig = -1.0;

    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        if (!candidates[i].reachable) continue;
        if (candidates[i].information_gain > best_ig) {
            best_ig = candidates[i].information_gain;
            best_idx = i;
        }
    }
    return best_idx;
}

// ---------------------------------------------------------------------------
// Strategy 5: PoseGain — max utility U(v) = IG / (1 + path_cost + time_cost)
// ---------------------------------------------------------------------------
int BaselineStrategies::select_pose_gain(
    std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates)
{
    int best_idx = -1;
    double best_utility = -1.0;

    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        if (!candidates[i].reachable) continue;
        if (candidates[i].utility_score > best_utility) {
            best_utility = candidates[i].utility_score;
            best_idx = i;
        }
    }
    return best_idx;
}

// ---------------------------------------------------------------------------
// Strategy 6: Path Cost Only — minimize the virtual trajectory proxy.
// ---------------------------------------------------------------------------
int BaselineStrategies::select_path_cost_only(
    const std::vector<cs625_nbv::msg::ViewpointCandidate>& candidates)
{
    int best_idx = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        if (!candidates[i].reachable) continue;
        const double cost = candidates[i].path_length + candidates[i].planning_time;
        if (cost < best_cost) {
            best_cost = cost;
            best_idx = i;
        }
    }
    return best_idx;
}

}  // namespace cs625_nbv
