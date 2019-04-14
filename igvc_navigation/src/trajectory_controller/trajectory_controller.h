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
using sdf_cost::SDFCostCoefficients;
using sdf_cost::SignedDistanceFieldCost;
using some_controller::OptimizationResult;

using signed_distance_field::SignedDistanceField;
using signed_distance_field::SignedDistanceFieldOptions;

using Model = DifferentialDriveModel;
using Cost = SignedDistanceFieldCost;
using State = RobotState;
using Controls = Model::Controls;

class TrajectoryController
{
public:
  TrajectoryController();
  std::unique_ptr<cv::Mat> test();

private:
  static visualization_msgs::Marker toLineStrip(const std::vector<State>& states, int id, float width, float r, float g,
                                                float b, float a);
  void visualizeRollout(const OptimizationResult<Model>& optimization_result) const;
  ros::Publisher rollout_pub_;
};
}  // namespace trajectory_controller

#endif  // SRC_TRAJECTORY_CONTROLLER_H
