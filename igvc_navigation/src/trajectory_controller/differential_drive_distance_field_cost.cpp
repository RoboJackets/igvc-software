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
  if (isnan(state.velocity())) {
    ROS_ERROR_STREAM_THROTTLE(0.2, "WTF? left: " << state.wheel_velocity_.left << ", right: " << state.wheel_velocity_.right);
  }
  float cost = 0.0f;

  float acc = (std::abs(controls[0]) + std::abs(controls[1]));
  cost += coeffs_.acceleration * acc;
  ROS_INFO_STREAM_THROTTLE(1, "acc: " << coeffs_.acceleration << " * " << acc << " = " << coeffs_.acceleration * acc);
  ROS_INFO_STREAM_THROTTLE(1, "cost: " << cost);

  cost += coeffs_.velocity * (3 -state.velocity());
  ROS_INFO_STREAM_THROTTLE(1, "vel: " << coeffs_.velocity << " * 3 - " << state.velocity() << " = " << coeffs_.velocity * (3 - state.velocity()));
  ROS_INFO_STREAM_THROTTLE(1, "cost: " << cost);

  cost += coeffs_.path * getSDFValue(state) * getSDFValue(state);

  ROS_INFO_STREAM_THROTTLE(1, "path: " << coeffs_.path << " * " << getSDFValue(state) << " = " << coeffs_.path * getSDFValue(state));
  ROS_INFO_STREAM_THROTTLE(1, "cost: " << cost);

//   Penalize turning
  cost += coeffs_.angular_acceleration * std::abs(controls[0] - controls[1]);
  cost += coeffs_.angular_velocity * std::abs(state.wheel_velocity_.right - state.wheel_velocity_.left);

  if (state.velocity() < 0)
  {
    cost = std::numeric_limits<float>::infinity();
  }
  if (state.velocity() > velocity_limit_)
  {
    cost = std::numeric_limits<float>::infinity();
  }

  return cost;
}

float SignedDistanceFieldCost::getSDFValue(const RobotState& state)
{
  std::optional<float> value = signed_distance_field_->getValue(state.x, state.y);
  if (value)
  {
    return *value;
  }
  else
  {
    ROS_ERROR_THROTTLE(1, "getSDFValue out of bounds!!");
    return 0.0f;
  }
}
