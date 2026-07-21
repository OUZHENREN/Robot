#include "cs625_compliant_placement/placement_controller.hpp"

#include <cmath>

namespace cs625_compliant_placement
{

PlacementController::PlacementController()
{
}

void PlacementController::configure(
  double approach_speed_scale,
  double soft_speed_scale,
  double contact_hold_speed_scale,
  bool enable_contact_hold,
  bool enable_jam_detection,
  bool auto_enter_soft_approach)
{
  approach_speed_scale_ = approach_speed_scale;
  soft_speed_scale_ = soft_speed_scale;
  contact_hold_speed_scale_ = contact_hold_speed_scale;
  enable_contact_hold_ = enable_contact_hold;
  enable_jam_detection_ = enable_jam_detection;
  auto_enter_soft_approach_ = auto_enter_soft_approach;
}

void PlacementController::reset()
{
  phase_ = PlacementPhase::IDLE;
}

double PlacementController::compute_distance_to_target_mm(const ControllerInput & input) const
{
  const double dx = input.current_x_mm - input.target_x_mm;
  const double dy = input.current_y_mm - input.target_y_mm;
  const double dz = input.current_z_mm - input.target_z_mm;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

ControllerOutput PlacementController::update(const ControllerInput & input)
{
  ControllerOutput output;
  output.phase = phase_;
  output.commanded_speed_scale = 0.0;

  if (!input.active) {
    phase_ = PlacementPhase::IDLE;
    output.phase = phase_;
    output.stop_reason = "inactive";
    return output;
  }

  const double distance_mm = compute_distance_to_target_mm(input);

  if (phase_ == PlacementPhase::IDLE) {
    phase_ = auto_enter_soft_approach_ ? PlacementPhase::SOFT_APPROACH : PlacementPhase::APPROACH;
  }

  if (input.safety_triggered) {
    phase_ = PlacementPhase::ABORTED;
    output.phase = phase_;
    output.aborted = true;
    output.stop_reason = "safety_triggered";
    return output;
  }

  if (distance_mm <= input.target_tolerance_mm) {
    phase_ = PlacementPhase::DONE;
    output.phase = phase_;
    output.done = true;
    output.target_reached = true;
    output.stop_reason = "target_reached";
    return output;
  }

  if (input.use_force_contact && input.contact_detected) {
    phase_ = PlacementPhase::CONTACT_HOLD;
    output.phase = phase_;
    output.commanded_speed_scale =
      enable_contact_hold_ ? contact_hold_speed_scale_ : soft_speed_scale_;
    output.stop_reason = "contact_hold";
    return output;
  }

  if (phase_ == PlacementPhase::APPROACH) {
    output.phase = phase_;
    output.commanded_speed_scale = approach_speed_scale_;
    output.stop_reason = "approach";
    return output;
  }

  if (phase_ == PlacementPhase::SOFT_APPROACH) {
    output.phase = phase_;
    output.commanded_speed_scale = soft_speed_scale_;
    output.stop_reason = "soft_approach";
    return output;
  }

  if (phase_ == PlacementPhase::CONTACT_HOLD) {
    output.phase = phase_;
    output.commanded_speed_scale = contact_hold_speed_scale_;
    output.stop_reason = "contact_hold";
    return output;
  }

  output.phase = phase_;
  output.stop_reason = "unknown";
  return output;
}

}  // namespace cs625_compliant_placement
