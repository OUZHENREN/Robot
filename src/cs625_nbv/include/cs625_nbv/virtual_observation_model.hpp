#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <vector>

namespace cs625_nbv {

struct VirtualSurfacePoint {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
};

// Parameters shared by the synthetic camera and the pre-observation predictor.
// This model remains virtual-only and is not a real camera calibration model.
struct VirtualObservationConfig {
    Eigen::Vector3d cuboid_size{0.08, 0.06, 0.10};
    double surface_step{0.01};
    int image_width{160};
    int image_height{120};
    double horizontal_fov{1.047};
    double vertical_fov{0.785};
    double min_range{0.05};
    double max_range{5.0};
    double occlusion_fraction{0.0};
};

class VirtualObservationModel {
public:
    static std::vector<VirtualSurfacePoint> build_cuboid_surface(
        const Eigen::Vector3d& center,
        const VirtualObservationConfig& config);

    // Projects the discrete model using the supplied full camera pose and
    // returns the closest point per pixel (z-buffer visibility).
    static std::vector<std::size_t> zbuffer_visible_indices(
        const std::vector<VirtualSurfacePoint>& surface,
        const Eigen::Isometry3d& camera_pose,
        const VirtualObservationConfig& config);

    // Expected fraction before capture. The occlusion fraction is applied in
    // expectation; the synthetic sensor applies a deterministic seeded draw.
    static double predicted_visible_fraction(
        const Eigen::Vector3d& center,
        const Eigen::Isometry3d& camera_pose,
        const VirtualObservationConfig& config);
};

}  // namespace cs625_nbv
