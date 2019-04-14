/**
 * Differential drive implementation of the model to the Some controller.
 *
 * Author: Oswin So <oswinso@gmail.com>
 * Date Created: April 13th, 2019
 */
#ifndef SRC_SIMPLE_SIGNED_DISTANCE_FIELD_DIFFERENTIAL_DRIVE_COST_H
#define SRC_DIFFERENTIAL_DRIVE_DISTANCE_FIELD_COST_H

#include <igvc_utils/RobotState.hpp>
#include "cost_function.h"

using namespace trajectory_controller;

namespace sdf_cost
{
struct SDFCoefficients {
  float acceleration;
  float velocity;
  float path;
};
constexpr int ControlDims = 2;
class SignedDistanceFieldCost
  : public CostFunction<RobotState, ControlDims, SignedDistanceFieldCost>
{
public:
  explicit SignedDistanceFieldCost(const SDFCoefficients& coeffs);
  float getCost(const RobotState& state, const Controls& controls);
private:
  SDFCoefficients coeffs_;
};
}  // namespace differential_drive_signed_distance_field_cost

#endif  // SRC_SIMPLE_SIGNED_DISTANCE_FIELD_DIFFERENTIAL_DRIVE_COST_H
