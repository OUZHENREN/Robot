#include "cs625_compliant_placement/contact_detector.hpp"

#include <algorithm>
#include <cmath>

namespace cs625_compliant_placement
{

ContactDetector::ContactDetector()
{
}

void ContactDetector::configure(
  double contact_alpha,
  double baseline_alpha)
{
  contact_alpha_ = contact_alpha;
  baseline_alpha_ = baseline_alpha;
}

void ContactDetector::reset()
{
  initialized_ = false;
  baseline_tau_metric_ = 0.0;
  filtered_tau_metric_ = 0.0;
}

double ContactDetector::compute_tau_metric(
  const cs625_state_monitor::msg::CS625State & msg) const
{
  double sum = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    sum += std::abs(msg.joint_torques[i]);
  }
  return sum;
}

DetectorResult ContactDetector::update(const cs625_state_monitor::msg::CS625State & msg)
{
  DetectorResult result;

  if (!msg.has_joint_data) {
    return result;
  }

  const double tau_metric = compute_tau_metric(msg);

  if (!initialized_) {
    baseline_tau_metric_ = tau_metric;
    filtered_tau_metric_ = tau_metric;
    initialized_ = true;
  }

  baseline_tau_metric_ =
    baseline_alpha_ * tau_metric + (1.0 - baseline_alpha_) * baseline_tau_metric_;

  filtered_tau_metric_ =
    contact_alpha_ * tau_metric + (1.0 - contact_alpha_) * filtered_tau_metric_;

  const double residual = std::max(0.0, filtered_tau_metric_ - baseline_tau_metric_);

  result.tau_metric = tau_metric;
  result.filtered_tau_metric = residual;

  return result;
}

}  // namespace cs625_compliant_placement
