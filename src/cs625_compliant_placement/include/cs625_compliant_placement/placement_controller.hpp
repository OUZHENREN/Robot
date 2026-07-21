#pragma once

#include "cs625_compliant_placement/placement_types.hpp"

namespace cs625_compliant_placement
{

class PlacementController
{
public:
  PlacementController();

  void configure(
    double approach_speed_scale,
    double soft_speed_scale,
    double contact_hold_speed_scale,
    bool enable_contact_hold,
    bool enable_jam_detection,
    bool auto_enter_soft_approach);

  void reset();

  ControllerOutput update(const ControllerInput & input);

private:
  double compute_distance_to_target_mm(const ControllerInput & input) const;

private:
  PlacementPhase phase_ {PlacementPhase::IDLE};

  double approach_speed_scale_ {0.30};
  double soft_speed_scale_ {0.10};
  double contact_hold_speed_scale_ {0.05};

  bool enable_contact_hold_ {true};
  bool enable_jam_detection_ {true};
  bool auto_enter_soft_approach_ {true};
};

}  // namespace cs625_compliant_placement
