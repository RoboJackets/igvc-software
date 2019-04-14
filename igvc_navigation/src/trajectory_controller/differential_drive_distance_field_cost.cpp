#include "differential_drive_distance_field_cost.h"

using namespace sdf_cost;

SignedDistanceFieldCost::SignedDistanceFieldCost(const SDFCoefficients& coeffs) : coeffs_{ coeffs } {

}

float SignedDistanceFieldCost::getCost(const RobotState& state, const Controls& controls)
{
  float cost = 0.0f;

  cost += coeffs_.acceleration * (std::abs(controls[0]) + std::abs(controls[1]));
  cost += coeffs_.velocity * state.velocity();
  cost += coeffs_.path * getSDFValue(state);
  return cost;
}

float SignedDistanceFieldCost::getSDFValue(const RobotState& state) {
  return 0.0f;
}
