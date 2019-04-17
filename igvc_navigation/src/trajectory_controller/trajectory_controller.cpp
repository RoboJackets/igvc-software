#include <utility>

#include <visualization_msgs/MarkerArray.h>
#include "trajectory_controller.h"

namespace trajectory_controller
{
TrajectoryController::TrajectoryController(const SignedDistanceFieldOptions& sdf_options,
                                           const SomeControllerOptions& some_controller_options,
                                           const DifferentialDriveOptions& differential_drive_options,
                                           const SDFCostOptions& sdf_cost_options)
  : sdf_options_{ sdf_options }
  , some_controller_options_{ some_controller_options }
  , differential_drive_options_{ differential_drive_options }
  , sdf_cost_options_{ sdf_cost_options }
  , traversal_costs_{ std::make_unique<cv::Mat>(sdf_options_.rows_, sdf_options_.cols_, CV_32F, sdf_options_.traversal_cost_) }
  , signed_distance_field_{ std::make_shared<SignedDistanceField>(sdf_options_) }
  , model_{ std::make_shared<Model>(differential_drive_options_) }
  , cost_function_{ std::make_shared<Cost>(signed_distance_field_, sdf_cost_options_) }
  , controller_{ std::make_shared<SomeController<Model, Cost>>(model_, cost_function_, some_controller_options) }
{
}

std::unique_ptr<ControllerResult> TrajectoryController::getControls(const nav_msgs::PathConstPtr& path,
                                                                    const RobotState& state)
{
  signed_distance_field_->setCenter(state.x, state.y);
  const auto [start_idx, end_idx] = getPathIndices(path, state);
  // TODO: Change this to have the gamma point be the point at the end of the path, an the traversal costs to have
  // little cost along the path and high cost everywhere else
  signed_distance_field_->calculate(*path, start_idx, end_idx, *traversal_costs_);

  std::unique_ptr<OptimizationResult<Model>> optimization_result = controller_->optimize(state);

  igvc_msgs::velocity_pair controls;
  RobotState lmao = optimization_result->weighted_particle.state_vec_[5];
  controls.left_velocity = lmao.wheel_velocity_.left;
  controls.right_velocity = lmao.wheel_velocity_.right;

//  double v = (controls.left_velocity + controls.right_velocity) / 2;
//  double w = (controls.right_velocity - controls.left_velocity) / 0.48;
//  ROS_INFO_STREAM("k: " << w/v);

  std::unique_ptr<cv::Mat> sdf_mat = signed_distance_field_->toMat();
  std::unique_ptr<ControllerResult> controller_result =
      std::make_unique<ControllerResult>(std::move(optimization_result), std::move(sdf_mat), std::move(controls));
  return controller_result;
}

std::pair<int, int> TrajectoryController::getPathIndices(const nav_msgs::PathConstPtr& path,
                                                         const RobotState& state) const
{
  auto closest =
      std::min_element(path->poses.begin(), path->poses.end(),
                       [&state](const geometry_msgs::PoseStamped& lhs, const geometry_msgs::PoseStamped& rhs) {
                         float lhs_dist = state.distTo(lhs.pose.position);
                         float rhs_dist = state.distTo(rhs.pose.position);
                         return lhs_dist < rhs_dist;
                       });
  int start_idx = closest - path->poses.begin();
  int end_idx = closest - path->poses.begin();
  for (auto i = closest; i >= path->poses.begin(); i--)
  {
    if (signed_distance_field_->isValidNode(i->pose.position.x, i->pose.position.y))
    {
      start_idx = i - path->poses.begin();
    }
    else
    {
      break;
    }
  }
  for (auto i = closest; i != path->poses.end(); i++)
  {
    if (signed_distance_field_->isValidNode(i->pose.position.x, i->pose.position.y))
    {
      end_idx = i - path->poses.begin();
    }
    else
    {
      break;
    }
  }
  return { start_idx, end_idx };
}

ControllerResult::ControllerResult(std::unique_ptr<OptimizationResult<Model>> optimization_res,
                                   std::unique_ptr<cv::Mat> sdf, igvc_msgs::velocity_pair controls)
  : optimization_result{ std::move(optimization_res) }
  , signed_distance_field{ std::move(sdf) }
  , controls{ std::move(controls) }
{
}
}  // namespace trajectory_controller
