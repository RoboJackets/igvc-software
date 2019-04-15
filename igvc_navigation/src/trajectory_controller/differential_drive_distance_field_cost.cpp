#include "differential_drive_distance_field_cost.h"

using namespace sdf_cost;

SignedDistanceFieldCost::SignedDistanceFieldCost(const SDFCostCoefficients& coeffs,
                                                 std::shared_ptr<SignedDistanceField> signed_distance_field, float velocity_limit)
  : coeffs_{ coeffs }, signed_distance_field_{ std::move(signed_distance_field) }, velocity_limit_{ velocity_limit }
{
}

float SignedDistanceFieldCost::getCost(const RobotState& state, const Controls& controls)
{
  float cost = 0.0f;

  cost += coeffs_.acceleration * (std::abs(controls[0]) + std::abs(controls[1]));
  cost += coeffs_.velocity * (1/state.velocity());
  cost += coeffs_.path * getSDFValue(state) * getSDFValue(state);
  if (state.velocity() < 0) {
    cost = std::numeric_limits<float>::max();
  }
  if (state.velocity() > velocity_limit_) {
    cost = std::numeric_limits<float>::max();
  }
  // Penalize turning
  cost += coeffs_.angular_acceleration * std::abs(controls[0] - controls[1]);

  return cost;
}

float SignedDistanceFieldCost::getSDFValue(const RobotState& state)
{
  std::optional<float> value = signed_distance_field_->getValue(state.x, state.y);
  if (value) {
    return *value;
  } else {
    return 0.0f;
  }
}
