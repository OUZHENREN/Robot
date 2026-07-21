#include "cs625_nbv/experiment_logger.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <fstream>

namespace fs = std::filesystem;

namespace cs625_nbv {

ExperimentLogger::ExperimentLogger(const std::string& base_dir)
{
    // Expand tilde
    std::string expanded = base_dir;
    if (!expanded.empty() && expanded[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home) {
            expanded = std::string(home) + expanded.substr(1);
        }
    }
    base_dir_ = expanded;
}

void ExperimentLogger::start_episode(
    const std::string& strategy_name,
    const std::string& scene_name,
    int episode_id)
{
    episode_id_ = episode_id;

    // Generate timestamp
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream ts;
    ts << std::put_time(&tm, "%Y%m%d_%H%M%S");

    // Create directory
    std::ostringstream dirname;
    dirname << ts.str() << "_" << strategy_name << "_" << scene_name << "_" << episode_id;
    current_episode_dir_ = (fs::path(base_dir_) / dirname.str()).string();

    fs::create_directories(current_episode_dir_);
    fs::create_directories(fs::path(current_episode_dir_) / "point_clouds");
    fs::create_directories(fs::path(current_episode_dir_) / "covariance");

    // Open CSV file
    std::string csv_path = (fs::path(current_episode_dir_) / "episode.csv").string();
    csv_file_.open(csv_path);
    csv_file_ << "step,view_x,view_y,view_z,view_rx,view_ry,view_rz,"
              << "trans_error,rot_error,add_score,"
              << "ig_achieved,path_length,planning_time\n";
}

void ExperimentLogger::log_step(
    int step,
    double pos_x, double pos_y, double pos_z,
    double rot_x, double rot_y, double rot_z,
    double trans_error, double rot_error, double add_score,
    double ig_achieved, double path_length, double planning_time)
{
    csv_file_ << step << ","
              << pos_x << "," << pos_y << "," << pos_z << ","
              << rot_x << "," << rot_y << "," << rot_z << ","
              << trans_error << "," << rot_error << "," << add_score << ","
              << ig_achieved << "," << path_length << "," << planning_time << "\n";
    csv_file_.flush();
}

void ExperimentLogger::end_episode(const EpisodeResult& result)
{
    csv_file_.close();

    // Write summary JSON
    std::string json_path = (fs::path(current_episode_dir_) / "summary.json").string();
    // Simple JSON construction (in production, use nlohmann/json)
    std::ofstream json(json_path);
    json << "{\n";
    json << "  \"strategy_name\": \"" << result.strategy_name << "\",\n";
    json << "  \"scene_name\": \"" << result.scene_name << "\",\n";
    json << "  \"episode_id\": " << result.episode_id << ",\n";
    json << "  \"final_translation_error\": " << result.final_translation_error << ",\n";
    json << "  \"final_rotation_error\": " << result.final_rotation_error << ",\n";
    json << "  \"final_add_score\": " << result.final_add_score << ",\n";
    json << "  \"total_views\": " << result.total_views << ",\n";
    json << "  \"total_path_length\": " << result.total_path_length << ",\n";
    json << "  \"total_planning_time\": " << result.total_planning_time << ",\n";
    json << "  \"total_execution_time\": " << result.total_execution_time << ",\n";
    json << "  \"candidates_generated\": " << result.candidates_generated << ",\n";
    json << "  \"reachable_candidates\": " << result.reachable_candidates << ",\n";
    json << "  \"unreachable_ratio\": " << result.unreachable_ratio << ",\n";
    json << "  \"converged\": " << (result.converged ? "true" : "false") << ",\n";
    json << "  \"stop_reason\": " << result.stop_reason << "\n";
    json << "}\n";
    json.close();
}

void ExperimentLogger::save_config_snapshot(const std::string& yaml_content)
{
    std::string yaml_path = (fs::path(current_episode_dir_) / "config.yaml").string();
    std::ofstream yaml(yaml_path);
    yaml << yaml_content;
    yaml.close();
}

void ExperimentLogger::generate_statistics(
    const std::string& base_dir,
    std::vector<EpisodeResult>& all_results)
{
    // Aggregate statistics
    int count = static_cast<int>(all_results.size());
    if (count == 0) return;

    double sum_trans = 0, sum_rot = 0, sum_add = 0;
    double sum_views = 0, sum_path = 0, sum_time = 0;
    int converged_count = 0;

    for (const auto& r : all_results) {
        sum_trans += r.final_translation_error;
        sum_rot += r.final_rotation_error;
        sum_add += r.final_add_score;
        sum_views += r.total_views;
        sum_path += r.total_path_length;
        sum_time += r.total_planning_time;
        if (r.converged) converged_count++;
    }

    std::string stats_path = (fs::path(base_dir) / "aggregate_stats.csv").string();
    std::ofstream stats(stats_path);
    stats << "metric,mean,std,N\n";
    stats << "translation_error," << sum_trans / count << ",TBD," << count << "\n";
    stats << "rotation_error," << sum_rot / count << ",TBD," << count << "\n";
    stats << "add_score," << sum_add / count << ",TBD," << count << "\n";
    stats << "total_views," << sum_views / count << ",TBD," << count << "\n";
    stats << "total_path_length," << sum_path / count << ",TBD," << count << "\n";
    stats << "total_planning_time," << sum_time / count << ",TBD," << count << "\n";
    stats << "convergence_rate," << (100.0 * converged_count / count) << "%,TBD," << count << "\n";
    stats.close();
}

}  // namespace cs625_nbv
