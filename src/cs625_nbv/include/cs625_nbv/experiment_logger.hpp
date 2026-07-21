#pragma once

#include <string>
#include <vector>
#include <fstream>
#include "cs625_nbv/msg/episode_result.hpp"

namespace cs625_nbv {
using msg::EpisodeResult;

/**
 * @brief Structured experiment data logger.
 *
 * Creates per-episode directories with CSV and JSON output:
 *   ~/nbv_experiments/<timestamp>_<strategy_name>/
 *     ├── episode.csv      — per-view metrics
 *     ├── summary.json     — final aggregate metrics
 *     └── config.yaml      — parameter snapshot
 */
class ExperimentLogger {
public:
    /**
     * @param base_dir Root directory for experiment logs.
     */
    explicit ExperimentLogger(const std::string& base_dir = "~/nbv_experiments");

    /**
     * @brief Start a new episode log.
     *
     * Creates the output directory and opens CSV writer.
     */
    void start_episode(const std::string& strategy_name,
                       const std::string& scene_name,
                       int episode_id);

    /**
     * @brief Log one step (viewpoint) in the episode.
     */
    void log_step(int step,
                  double pos_x, double pos_y, double pos_z,
                  double rot_x, double rot_y, double rot_z,
                  double trans_error, double rot_error, double add_score,
                  double ig_achieved, double path_length, double planning_time);

    /**
     * @brief Finish the episode, write summary.json.
     */
    void end_episode(const EpisodeResult& result);

    /**
     * @brief Save current parameter snapshot as YAML.
     */
    void save_config_snapshot(const std::string& yaml_content);

    /**
     * @brief Generate aggregate statistics across episodes.
     */
    static void generate_statistics(
        const std::string& base_dir,
        std::vector<EpisodeResult>& all_results
    );

private:
    std::string base_dir_;
    std::string current_episode_dir_;
    std::ofstream csv_file_;
    int episode_id_{0};
};

}  // namespace cs625_nbv
