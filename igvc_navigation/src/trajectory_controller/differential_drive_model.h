/**
 * Differential drive implementation of the model to the Some controller.
 *
 * Author: Oswin So <oswinso@gmail.com>
 * Date Created: April 13th, 2019
 */
#ifndef SRC_DIFFERENTIAL_DRIVE_MODEL_H
#define SRC_DIFFERENTIAL_DRIVE_MODEL_H

#include "model.h"
#include <igvc_utils/RobotState.hpp>

using namespace trajectory_controller;

namespace differential_drive_model {
  constexpr int ControlDims = 2;

  class DifferentialDriveModel : public Model<RobotState, ControlDims, DifferentialDriveModel> {
  public:
    explicit DifferentialDriveModel(Bound acceleration_bound, float axle_length);
    std::array<Bound, ControlDims> getBounds();
    RobotState propogateState(RobotState state, const Controls& controls, float dt);
  private:
    Bound acceleration_bound_;
    float axle_length_;
  };
}

#endif //SRC_DIFFERENTIAL_DRIVE_MODEL_H
