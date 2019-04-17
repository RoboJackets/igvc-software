#include "differential_drive_distance_field_cost.h"

using namespace sdf_cost;

SignedDistanceFieldCost::SignedDistanceFieldCost(std::shared_ptr<SignedDistanceField> signed_distance_field,
                                                 const SDFCostOptions& options)
  : coeffs_{ options.coefficients }
  , signed_distance_field_{ std::move(signed_distance_field) }
  , velocity_limit_{ options.velocity_limit }
{
}

float SignedDistanceFieldCost::getCost(const RobotState& state, const Controls& controls)
{
  float cost = 0.0f;

  cost += coeffs_.acceleration * (std::abs(controls[0]) + std::abs(controls[1]));
  cost += coeffs_.velocity * (1 / state.velocity());
  cost += coeffs_.path * getSDFValue(state);
  if (state.velocity() < 0)
  {
    cost = 1e8;
  }
  if (state.velocity() > velocity_limit_)
  {
    cost = 1e8;
  }
  // Penalize turning
  cost += coeffs_.angular_acceleration * std::abs(controls[0] - controls[1]);

  return cost;
}

float SignedDistanceFieldCost::getSDFValue(const RobotState& state)
{
  std::optional<float> value = signed_distance_field_->getValue(state.x, state.y);
  if (value)
  {
    ROS_INFO_STREAM_THROTTLE(0.1, "value: " << *value);
    return *value;
  }
  else
  {
    ROS_ERROR_THROTTLE(1, "getSDFValue out of bounds!!");
    return 0.0f;
  }
}
