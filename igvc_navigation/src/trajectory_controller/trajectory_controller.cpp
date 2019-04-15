#include "trajectory_controller.h"
#include <visualization_msgs/MarkerArray.h>

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
  , traversal_costs_{ std::make_unique<cv::Mat>(sdf_options_.rows_, sdf_options_.cols_, CV_32F, 1.0f) }
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
  signed_distance_field_->calculate(*path, start_idx, end_idx, *traversal_costs_);

  std::unique_ptr<OptimizationResult<Model>> optimization_result = controller_->optimize(state);
  std::unique_ptr<cv::Mat> sdf_mat;
  std::unique_ptr<ControllerResult> controller_result = std::make_unique<ControllerResult>(std::move(optimization_result), std::move(sdf_mat));
//  controller_result->signed_distance_field = std::move(sdf_mat);
//  controller_result->optimization_result = std::move(optimization_result);
  return controller_result;
//  return std::make_unique<ControllerResult>(std::move(optimization_result), std::move(sdf_mat));
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
                                   std::unique_ptr<cv::Mat> sdf)
  : optimization_result{ std::move(optimization_res) }, signed_distance_field{ std::move(sdf) }
{
}
}  // namespace trajectory_controller
