#include "trajectory_controller.h"
#include <visualization_msgs/MarkerArray.h>

namespace trajectory_controller
{
TrajectoryController::TrajectoryController(const SignedDistanceFieldOptions& sdf_options, float timestep, float horizon,
                                           int samples, float max_velocity)
  : sdf_options_{ sdf_options }
  , timestep_{ timestep }
  , horizon_{ horizon }
  , samples_{ samples }
  , max_velocity_{ max_velocity }
{
}

std::unique_ptr<cv::Mat> TrajectoryController::test()
{
  //  std::cout << "Width: " << sdf_options_.width_ << ", Height: " << sdf_options_.height_
  //            << ", rows: " << sdf_options_.rows_ << ", cols: " << sdf_options_.cols_
  //            << ", resolution: " << sdf_options_.resolution_ << std::endl;
  std::shared_ptr<Model> model = std::make_shared<Model>(Bound{ -5, 5 }, 1.0f);
  std::shared_ptr<SignedDistanceField> signed_distance_field = std::make_shared<SignedDistanceField>(sdf_options_);
  SDFCostCoefficients options{ 1.0, 1.0, 10.0, 100.0 };

  std::shared_ptr<Cost> cost_function = std::make_shared<Cost>(options, signed_distance_field, max_velocity_);
  SomeController<Model, Cost> controller(model, cost_function, timestep_, horizon_, samples_);
  RobotState state{};
  state.wheel_velocity_.left = 0.1;
  state.wheel_velocity_.right = 0.1;

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  path.poses.emplace_back(pose);
  pose.pose.position.x = 5;
  pose.pose.position.y = 0;
  path.poses.emplace_back(pose);
  pose.pose.position.x = 5;
  pose.pose.position.y = 5;
  path.poses.emplace_back(pose);
  cv::Mat traversal_costs(sdf_options_.rows_, sdf_options_.cols_, CV_32F, 1.0f);
  signed_distance_field->calculate(path, 0, 2, traversal_costs);

  OptimizationResult<Model> result = controller.optimize(state);
  visualizeRollout(result);
  return signed_distance_field->toMat();
}

void TrajectoryController::visualizeRollout(const OptimizationResult<Model>& optimization_result) const
{
  int id = 0;
  visualization_msgs::MarkerArray marker_array;
  for (const Particle<Model>& particle : optimization_result.particles)
  {
    marker_array.markers.emplace_back(toLineStrip(particle.state_vec_, id++, 0.001f, 0.97f, 0.43f, 0.48f, 0.8f));
  }
  marker_array.markers.emplace_back(
      toLineStrip(optimization_result.best_particle.state_vec_, id++, 0.01f, 1.0f, 1.0f, 1.0f, 0.8f));
  rollout_pub_.publish(marker_array);
}

visualization_msgs::Marker TrajectoryController::toLineStrip(const std::vector<State>& states, int id, float width,
                                                             float r, float g, float b, float a)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/odom";
  marker.header.stamp = ros::Time::now();

  marker.ns = "rollout";
  marker.id = id;

  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  marker.type = visualization_msgs::Marker::LINE_STRIP;
  for (const State& state : states)
  {
    geometry_msgs::Point point;
    point.x = state.x;
    point.y = state.y;
    marker.points.emplace_back(point);
  }
  return marker;
}
}  // namespace trajectory_controller
