/**
 * Some controller™ implementation
 *
 * Author: Oswin So <oswinso@gmail.com>
 * Date Created: April 13th, 2019
 */
#ifndef SRC_TRAJECTORY_CONTROLLER_H
#define SRC_TRAJECTORY_CONTROLLER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "differential_drive_distance_field_cost.h"
#include "differential_drive_model.h"
#include "some_controller.h"

namespace trajectory_controller
{
using differential_drive_model::DifferentialDriveModel;
using differential_drive_model::DifferentialDriveOptions;

using sdf_cost::SDFCostOptions;
using sdf_cost::SignedDistanceFieldCost;
using some_controller::OptimizationResult;

using signed_distance_field::SignedDistanceField;
using signed_distance_field::SignedDistanceFieldOptions;

using Model = DifferentialDriveModel;
using Cost = SignedDistanceFieldCost;
using State = RobotState;
using Controls = Model::Controls;

struct ControllerResult
{
  std::unique_ptr<OptimizationResult<Model>> optimization_result;
  std::unique_ptr<cv::Mat> signed_distance_field;

public:
  ControllerResult(std::unique_ptr<OptimizationResult<Model>> optimization_res, std::unique_ptr<cv::Mat> sdf);
};

class TrajectoryController
{
public:
  TrajectoryController(const SignedDistanceFieldOptions& sdf_options,
                       const SomeControllerOptions& some_controller_options, const DifferentialDriveOptions& differential_drive_options,
                       const SDFCostOptions& sdf_cost_options);
  std::unique_ptr<ControllerResult> getControls(const nav_msgs::PathConstPtr& path, const RobotState& state);

private:
  std::pair<int, int> getPathIndices(const nav_msgs::PathConstPtr& path, const RobotState& state) const;
  void visualizeRollout(const OptimizationResult<Model>& optimization_result) const;
  ros::Publisher rollout_pub_;

  SignedDistanceFieldOptions sdf_options_;
  SomeControllerOptions some_controller_options_;
  DifferentialDriveOptions differential_drive_options_;
  SDFCostOptions sdf_cost_options_;

  std::unique_ptr<cv::Mat> traversal_costs_;
  std::shared_ptr<SignedDistanceField> signed_distance_field_;
  std::shared_ptr<Model> model_;
  std::shared_ptr<Cost> cost_function_;
  std::shared_ptr<SomeController<Model, Cost>> controller_;
};
}  // namespace trajectory_controller

#endif  // SRC_TRAJECTORY_CONTROLLER_H
