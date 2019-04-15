/**
 * Differential drive implementation of the model to the Some controller.
 *
 * Author: Oswin So <oswinso@gmail.com>
 * Date Created: April 13th, 2019
 */
#ifndef SRC_SIMPLE_SIGNED_DISTANCE_FIELD_DIFFERENTIAL_DRIVE_COST_H
#define SRC_DIFFERENTIAL_DRIVE_DISTANCE_FIELD_COST_H

#include <igvc_navigation/signed_distance_field.h>
#include <igvc_utils/RobotState.hpp>
#include "cost_function.h"

using namespace some_controller;
using signed_distance_field::SignedDistanceField;

namespace sdf_cost
{
struct SDFCostCoefficients
{
  float acceleration;
  float angular_acceleration;
  float velocity;
  float path;
};

struct SDFCostOptions
{
  SDFCostCoefficients coefficients;
  float velocity_limit;
};

constexpr int ControlDims = 2;
class SignedDistanceFieldCost : public CostFunction<RobotState, ControlDims, SignedDistanceFieldCost>
{
public:
  explicit SignedDistanceFieldCost(std::shared_ptr<SignedDistanceField> signed_distance_field, const SDFCostOptions& options);
  float getCost(const RobotState& state, const Controls& controls);

private:
  float getSDFValue(const RobotState& state);

  SDFCostCoefficients coeffs_;
  std::shared_ptr<SignedDistanceField> signed_distance_field_;
  float velocity_limit_;
};
}  // namespace sdf_cost

#endif  // SRC_SIMPLE_SIGNED_DISTANCE_FIELD_DIFFERENTIAL_DRIVE_COST_H
