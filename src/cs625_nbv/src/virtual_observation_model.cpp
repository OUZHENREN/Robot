#include "cs625_nbv/virtual_observation_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cs625_nbv {

std::vector<VirtualSurfacePoint> VirtualObservationModel::build_cuboid_surface(
    const Eigen::Vector3d& center,
    const VirtualObservationConfig& config)
{
    const Eigen::Vector3d half = 0.5 * config.cuboid_size;
    const double step = std::max(0.002, config.surface_step);
    std::vector<VirtualSurfacePoint> points;
    const auto append = [&points](double x, double y, double z, double nx, double ny, double nz) {
        points.push_back({Eigen::Vector3d(x, y, z), Eigen::Vector3d(nx, ny, nz)});
    };
    for (double x = -half.x(); x <= half.x() + 1e-9; x += step) {
        for (double y = -half.y(); y <= half.y() + 1e-9; y += step) {
            append(center.x() + x, center.y() + y, center.z() - half.z(), 0, 0, -1);
            append(center.x() + x, center.y() + y, center.z() + half.z(), 0, 0, 1);
        }
    }
    for (double x = -half.x(); x <= half.x() + 1e-9; x += step) {
        for (double z = -half.z(); z <= half.z() + 1e-9; z += step) {
            append(center.x() + x, center.y() - half.y(), center.z() + z, 0, -1, 0);
            append(center.x() + x, center.y() + half.y(), center.z() + z, 0, 1, 0);
        }
    }
    for (double y = -half.y(); y <= half.y() + 1e-9; y += step) {
        for (double z = -half.z(); z <= half.z() + 1e-9; z += step) {
            append(center.x() - half.x(), center.y() + y, center.z() + z, -1, 0, 0);
            append(center.x() + half.x(), center.y() + y, center.z() + z, 1, 0, 0);
        }
    }
    return points;
}

std::vector<std::size_t> VirtualObservationModel::zbuffer_visible_indices(
    const std::vector<VirtualSurfacePoint>& surface,
    const Eigen::Isometry3d& camera_pose,
    const VirtualObservationConfig& config)
{
    const int width = std::max(1, config.image_width);
    const int height = std::max(1, config.image_height);
    const double tan_h = std::tan(0.5 * config.horizontal_fov);
    const double tan_v = std::tan(0.5 * config.vertical_fov);
    if (tan_h <= 0.0 || tan_v <= 0.0) return {};
    std::vector<double> depth(static_cast<std::size_t>(width * height),
        std::numeric_limits<double>::infinity());
    std::vector<int> index(static_cast<std::size_t>(width * height), -1);
    const Eigen::Isometry3d world_to_camera = camera_pose.inverse();
    for (std::size_t point_index = 0; point_index < surface.size(); ++point_index) {
        const Eigen::Vector3d point = world_to_camera * surface[point_index].position;
        if (point.z() <= config.min_range || point.z() > config.max_range) continue;
        const double normalized_x = point.x() / (point.z() * tan_h);
        const double normalized_y = point.y() / (point.z() * tan_v);
        if (std::abs(normalized_x) > 1.0 || std::abs(normalized_y) > 1.0) continue;
        const int u = std::clamp(static_cast<int>((normalized_x + 1.0) * 0.5 * width), 0, width - 1);
        const int v = std::clamp(static_cast<int>((normalized_y + 1.0) * 0.5 * height), 0, height - 1);
        const std::size_t pixel = static_cast<std::size_t>(v * width + u);
        if (point.z() < depth[pixel]) {
            depth[pixel] = point.z();
            index[pixel] = static_cast<int>(point_index);
        }
    }
    std::vector<std::size_t> visible;
    visible.reserve(surface.size());
    for (const int point_index : index) {
        if (point_index >= 0) visible.push_back(static_cast<std::size_t>(point_index));
    }
    std::sort(visible.begin(), visible.end());
    return visible;
}

double VirtualObservationModel::predicted_visible_fraction(
    const Eigen::Vector3d& center,
    const Eigen::Isometry3d& camera_pose,
    const VirtualObservationConfig& config)
{
    const auto surface = build_cuboid_surface(center, config);
    if (surface.empty()) return 0.0;
    const auto visible = zbuffer_visible_indices(surface, camera_pose, config);
    const double unoccluded = static_cast<double>(visible.size()) / static_cast<double>(surface.size());
    return std::clamp(unoccluded * (1.0 - config.occlusion_fraction), 0.0, 1.0);
}

}  // namespace cs625_nbv
