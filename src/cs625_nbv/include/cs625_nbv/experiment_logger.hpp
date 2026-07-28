#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <cstdint>
#include "cs625_nbv/msg/episode_result.hpp"

namespace cs625_nbv {
using msg::EpisodeResult;

struct EpisodeMetadata {
    std::string target_object_id;
    std::string data_source{"synthetic_smoke_test"};
    std::string validity_label{"interface_only"};
    int random_seed{625};
    std::string git_commit{"unknown"};
    std::string ros_distro{"unknown"};
    std::string launch_profile{"unknown"};
};

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
    std::string start_episode(const std::string& strategy_name,
                              const std::string& scene_name,
                              int episode_id,
                              const EpisodeMetadata& metadata);

    /**
     * @brief Log one step (viewpoint) in the episode.
     */
    void log_step(int step,
                  double pos_x, double pos_y, double pos_z,
                  double quat_x, double quat_y, double quat_z, double quat_w,
                  double trans_error, double rot_error, double add_score,
                  double ig_achieved, double path_length, double planning_time,
                  uint32_t observation_points, double visible_ratio,
                  double registration_rmse, double uncertainty_proxy,
                  double prior_covariance_translation_std,
                  double predicted_posterior_covariance_translation_std,
                  double covariance_translation_std, double view_novelty,
                  double observability_score,
                  int candidates_generated, int reachable_candidates);

    /**
     * @brief Finish the episode, write summary.json.
     */
    void end_episode(const EpisodeResult& result);

    /**
     * @brief Save current parameter snapshot as YAML.
     */
    void save_config_snapshot(const std::string& yaml_content);

    const std::string& current_episode_dir() const { return current_episode_dir_; }

    /**
     * @brief Export episode summaries to a report-ready CSV.
     * @return Number of rows written.
     */
    static int export_results_csv(const std::string& output_path,
                                  const std::vector<EpisodeResult>& results);

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
    EpisodeMetadata metadata_;
    std::string strategy_name_;
    std::string scene_name_;
};

}  // namespace cs625_nbv
