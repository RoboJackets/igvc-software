#include "trajectory_controller.h"
#include "differential_drive_model.h"
#include "differential_drive_distance_field_cost.h"
#include "some_controller.h"

namespace trajectory_controller {
using differential_drive_model::DifferentialDriveModel;
using sdf_cost::SignedDistanceFieldCost;
using sdf_cost::SDFCostCoefficients;

using signed_distance_field::SignedDistanceField;
using signed_distance_field::SignedDistanceFieldOptions;

using Model = DifferentialDriveModel;
using Cost = SignedDistanceFieldCost;
using State = RobotState;
using Controls = Model::Controls;

TrajectoryController::TrajectoryController() {
  std::shared_ptr<Model> model = std::make_shared<Model>(Bound{-1, 1}, 1.0f);

  int rows = 11;
  int cols = 11;
  float x = 0;
  float y = 0;
  float resolution = 1.0f;
  SignedDistanceFieldOptions sdf_options{rows, cols, x, y, resolution};
  std::shared_ptr<SignedDistanceField> signed_distance_field = std::make_shared<SignedDistanceField>(sdf_options);
  SDFCostCoefficients options {1.0, 1.0, 1.0};

  std::shared_ptr<Cost> cost_function = std::make_shared<Cost>(options, signed_distance_field);
  float timestep = 1.0f;
  float horizon = 10.0f;
  int samples = 10;
  SomeController<Model, Cost> controller(model, cost_function, timestep, horizon, samples);
  RobotState state{};
  std::vector<Model::Controls> controls = controller.optimize(state);
}
}
