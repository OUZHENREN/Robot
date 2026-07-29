#include "cs625_nbv/experiment_logger.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <fstream>
#include <cmath>
#include <cstdlib>

namespace fs = std::filesystem;

namespace cs625_nbv {
namespace {

std::string json_escape(const std::string& value)
{
    std::ostringstream escaped;
    for (const char ch : value) {
        switch (ch) {
            case '\\': escaped << "\\\\"; break;
            case '"': escaped << "\\\""; break;
            case '\n': escaped << "\\n"; break;
            case '\r': escaped << "\\r"; break;
            case '\t': escaped << "\\t"; break;
            default: escaped << ch; break;
        }
    }
    return escaped.str();
}

std::string csv_escape(const std::string& value)
{
    if (value.find_first_of(",\"\r\n") == std::string::npos) {
        return value;
    }
    std::string escaped = "\"";
    for (const char ch : value) {
        escaped += ch == '"' ? "\"\"" : std::string(1, ch);
    }
    escaped += '"';
    return escaped;
}

}  // namespace

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

std::string ExperimentLogger::start_episode(
    const std::string& strategy_name,
    const std::string& scene_name,
    int episode_id,
    const EpisodeMetadata& metadata)
{
    episode_id_ = episode_id;
    metadata_ = metadata;
    strategy_name_ = strategy_name;
    scene_name_ = scene_name;

    // Generate timestamp
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream ts;
    ts << std::put_time(&tm, "%Y%m%d_%H%M%S");

    // Create directory
    std::ostringstream dirname;
    dirname << ts.str() << "_" << strategy_name << "_" << scene_name << "_" << episode_id;
    fs::path episode_path = fs::path(base_dir_) / dirname.str();
    int collision_index = 1;
    while (fs::exists(episode_path)) {
        std::ostringstream unique_name;
        unique_name << dirname.str() << "_" << std::setw(3) << std::setfill('0')
                    << collision_index++;
        episode_path = fs::path(base_dir_) / unique_name.str();
    }
    current_episode_dir_ = episode_path.string();

    fs::create_directories(current_episode_dir_);
    fs::create_directories(fs::path(current_episode_dir_) / "point_clouds");
    fs::create_directories(fs::path(current_episode_dir_) / "covariance");

    // Open CSV file
    std::string csv_path = (fs::path(current_episode_dir_) / "episode.csv").string();
    csv_file_.open(csv_path);
    csv_file_ << "run_id,data_source,validity_label,strategy_name,scene_name,episode_id,"
              << "random_seed,step,view_x,view_y,view_z,view_qx,view_qy,view_qz,view_qw,"
              << "trans_error,rot_error,add_score,"
              << "ig_achieved,path_length,planning_time,"
              << "observation_points,visible_ratio,registration_rmse_m,uncertainty_proxy_m,"
              << "prior_covariance_translation_std_m,predicted_posterior_covariance_translation_std_m,"
              << "covariance_translation_std_m,observed_covariance_translation_std_reduction_m,"
              << "virtual_initial_translation_bias_m,view_novelty,observability_score,"
              << "candidates_generated,reachable_candidates,reachable_ratio\n";
    csv_file_ << std::setprecision(12);
    return current_episode_dir_;
}

void ExperimentLogger::log_step(
    int step,
    double pos_x, double pos_y, double pos_z,
    double quat_x, double quat_y, double quat_z, double quat_w,
    double trans_error, double rot_error, double add_score,
    double ig_achieved, double path_length, double planning_time,
    uint32_t observation_points, double visible_ratio,
    double registration_rmse, double uncertainty_proxy,
    double prior_covariance_translation_std,
    double predicted_posterior_covariance_translation_std,
    double covariance_translation_std,
    double observed_covariance_translation_std_reduction,
    double virtual_initial_translation_bias_m,
    double view_novelty,
    double observability_score,
    int candidates_generated, int reachable_candidates)
{
    const double reachable_ratio = candidates_generated > 0
        ? static_cast<double>(reachable_candidates) / candidates_generated
        : 0.0;
    csv_file_ << csv_escape(fs::path(current_episode_dir_).filename().string()) << ","
              << csv_escape(metadata_.data_source) << ","
              << csv_escape(metadata_.validity_label) << ","
              << csv_escape(strategy_name_) << ","
              << csv_escape(scene_name_) << ","
              << episode_id_ << ","
              << metadata_.random_seed << ","
              << step << ","
              << pos_x << "," << pos_y << "," << pos_z << ","
              << quat_x << "," << quat_y << "," << quat_z << "," << quat_w << ","
              << trans_error << "," << rot_error << "," << add_score << ","
              << ig_achieved << "," << path_length << "," << planning_time << ","
              << observation_points << "," << visible_ratio << ","
              << registration_rmse << "," << uncertainty_proxy << ","
              << prior_covariance_translation_std << ","
              << predicted_posterior_covariance_translation_std << ","
              << covariance_translation_std << ","
              << observed_covariance_translation_std_reduction << ","
              << virtual_initial_translation_bias_m << "," << view_novelty << ","
              << observability_score << ","
              << candidates_generated << "," << reachable_candidates << ","
              << reachable_ratio << "\n";
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
    json << "  \"run_id\": \"" << json_escape(result.run_id) << "\",\n";
    json << "  \"strategy_name\": \"" << json_escape(result.strategy_name) << "\",\n";
    json << "  \"scene_name\": \"" << json_escape(result.scene_name) << "\",\n";
    json << "  \"episode_id\": " << result.episode_id << ",\n";
    json << "  \"target_object_id\": \"" << json_escape(metadata_.target_object_id) << "\",\n";
    json << "  \"data_source\": \"" << json_escape(result.data_source) << "\",\n";
    json << "  \"validity_label\": \"" << json_escape(result.validity_label) << "\",\n";
    json << "  \"random_seed\": " << result.random_seed << ",\n";
    json << "  \"git_commit\": \"" << json_escape(metadata_.git_commit) << "\",\n";
    json << "  \"ros_distro\": \"" << json_escape(metadata_.ros_distro) << "\",\n";
    json << "  \"launch_profile\": \"" << json_escape(metadata_.launch_profile) << "\",\n";
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
    json << "  \"stop_reason\": " << result.stop_reason << ",\n";
    json << "  \"failure_reason\": \"" << json_escape(result.failure_reason) << "\",\n";
    json << "  \"research_use_allowed\": "
         << (result.validity_label == "research_candidate" ? "true" : "false") << "\n";
    json << "}\n";
    json.close();

    export_results_csv(
        (fs::path(current_episode_dir_) / "report_summary.csv").string(),
        std::vector<EpisodeResult>{result}
    );
}

int ExperimentLogger::export_results_csv(
    const std::string& output_path,
    const std::vector<EpisodeResult>& results)
{
    const fs::path path(output_path);
    if (path.has_parent_path()) {
        fs::create_directories(path.parent_path());
    }
    std::ofstream csv(path);
    if (!csv.is_open()) {
        return 0;
    }
    csv << "run_id,data_source,validity_label,strategy_name,scene_name,episode_id,"
        << "random_seed,final_translation_error_m,final_rotation_error_rad,"
        << "final_add_score_m,total_views,total_path_length_m,total_planning_time_s,"
        << "total_execution_time_s,candidates_generated,reachable_candidates,"
        << "unreachable_candidates,unreachable_ratio,converged,stop_reason,failure_reason\n";
    csv << std::setprecision(12);
    for (const auto& result : results) {
        csv << csv_escape(result.run_id) << ","
            << csv_escape(result.data_source) << ","
            << csv_escape(result.validity_label) << ","
            << csv_escape(result.strategy_name) << ","
            << csv_escape(result.scene_name) << ","
            << result.episode_id << ","
            << result.random_seed << ","
            << result.final_translation_error << ","
            << result.final_rotation_error << ","
            << result.final_add_score << ","
            << result.total_views << ","
            << result.total_path_length << ","
            << result.total_planning_time << ","
            << result.total_execution_time << ","
            << result.candidates_generated << ","
            << result.reachable_candidates << ","
            << result.unreachable_candidates << ","
            << result.unreachable_ratio << ","
            << (result.converged ? "true" : "false") << ","
            << result.stop_reason << ","
            << csv_escape(result.failure_reason) << "\n";
    }
    return static_cast<int>(results.size());
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
