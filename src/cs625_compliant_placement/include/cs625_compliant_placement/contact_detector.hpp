#pragma once

#include "cs625_state_monitor/msg/cs625_state.hpp"
#include "cs625_compliant_placement/placement_types.hpp"

namespace cs625_compliant_placement
{

class ContactDetector
{
public:
  ContactDetector();

  void configure(
    double contact_alpha,
    double baseline_alpha);

  DetectorResult update(const cs625_state_monitor::msg::CS625State & msg);

  void reset();

private:
  double compute_tau_metric(const cs625_state_monitor::msg::CS625State & msg) const;

private:
  bool initialized_ {false};

  double contact_alpha_ {0.2};
  double baseline_alpha_ {0.01};

  double baseline_tau_metric_ {0.0};
  double filtered_tau_metric_ {0.0};
};

}  // namespace cs625_compliant_placement
