#include "smooth_control.h"
#include <igvc_utils/robot_state.h>
#include <algorithm>

SmoothControl::SmoothControl(SmoothControlOptions smooth_control_options, PathGenerationOptions path_generation_options,
                             TargetSelectionOptions target_selection_options,
                             CurvatureBlendingOptions curvature_blending_options, double axle_length)
  : smooth_control_options_{ smooth_control_options }
  , path_generation_options_{ path_generation_options }
  , target_selection_options_{ target_selection_options }
  , curvature_blending_options_{ curvature_blending_options }
  , axle_length_{ axle_length }
{
  ros::NodeHandle pNh{ "~" };
  target_pub_ = pNh.advertise<nav_msgs::Path>("smooth_control/target", 1);
  closest_point_pub_ = pNh.advertise<nav_msgs::Path>("smooth_control/closest_point", 1);
}

void SmoothControl::getPath(const nav_msgs::PathConstPtr& path, const igvc_msgs::trajectoryPtr& trajectory_ptr,
                            const RobotState& start_pos)
{
  RobotState state = start_pos;
  std::optional<RobotState> simulation_target = target_;
  size_t path_index = 0;

  ros::Time time = path->header.stamp;
  trajectory_ptr->header.stamp = path->header.stamp;

  // Calculate timesteps
  static ros::Duration dt = ros::Duration(1 / path_generation_options_.simulation_frequency);

  // Visualization
  nav_msgs::Path closest_point_path;
  nav_msgs::Path targets_path;

  closest_point_path.header = path->header;
  targets_path.header = path->header;
  for (int i = 0; i < path_generation_options_.getNumSamples() && path_index < path->poses.size() - 1; i++, time += dt)
  {
    // Get target
    path_index = getClosestIndex(path, state);
    RobotState second_target;
    std::tie(simulation_target, second_target) = getTargetPosition(path, state, simulation_target, path_index);

    RobotControl control = getControl(state, *simulation_target, second_target);
    state.propogateState(control, dt.toSec());

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = path->header;
    pose_stamped.pose = path->poses[path_index].pose;
    closest_point_path.poses.emplace_back(pose_stamped);

    geometry_msgs::PoseStamped target{};
    target.pose = simulation_target->toPose();
    target.header.stamp = time;
    targets_path.poses.emplace_back(target);

    trajectory_ptr->trajectory.emplace_back(state.toTrajectoryPoint(time, control, axle_length_));
  }
  target_pub_.publish(targets_path);
  closest_point_pub_.publish(closest_point_path);
}

RobotControl SmoothControl::getControl(const RobotState& state, const RobotState& target,
                                       const RobotState& second_target) const
{
  double K_first = getCurvature(state, target);
  double K_second = getCurvature(state, second_target);
  double distance = state.distTo(target);
  double K = blendCurvature(distance, K_first, K_second);
  return RobotControl::fromKV(K, path_generation_options_.simulation_velocity, axle_length_);
}

double SmoothControl::getCurvature(const RobotState& state, const RobotState& target) const
{
  double line_of_sight = atan2(target.y() - state.y(), target.x() - state.x());

  double delta = state.yaw() - line_of_sight;
  double theta = target.yaw() - line_of_sight;

  // adjust both angles to lie between -PI and PI
  igvc::fit_to_polar(delta);
  igvc::fit_to_polar(theta);

  // calculate the radius of curvature, K
  double d = state.distTo(target);  // euclidian distance to target
  double K = smooth_control_options_.k2 * (delta - atan(-smooth_control_options_.k1 * theta));
  K += (1 + (smooth_control_options_.k1 / (1 + pow(smooth_control_options_.k1 * theta, 2)))) * sin(delta);
  K /= -d;
  return K;
}

std::pair<RobotState, RobotState> SmoothControl::getTargetPosition(const nav_msgs::PathConstPtr& path,
                                                                   const RobotState& state,
                                                                   const std::optional<RobotState>& target,
                                                                   size_t path_index) const
{
  // If already have a target, find the closest point along the current path to the current target
  // If the closest point is too far,then break and find a new target
  if (target)
  {
    if (!reachedTarget(state, *target))
    {
      size_t target_index = findTargetOnPath(path, *target);
      RobotState newTarget{ path->poses[target_index] };
      // If the target is found, and the target isn't behind us, and the target is closer to target + 1 than we are
      double state_dist_target_next = state.distTo(path->poses[target_index + 1].pose.position);
      double target_dist_target_next = target->distTo(path->poses[target_index + 1].pose.position);
      if (target_index != 0 && target_index >= path_index && target_dist_target_next < state_dist_target_next &&
          distAlongPath(path, path_index, target_index) < target_selection_options_.lookahead_dist)
      {
        RobotState first_target = newTarget;
        RobotState second_target = acquireNewTarget(path, first_target, target_index);
        return { *target, second_target };
      }
    }
  }
  RobotState first_target = acquireNewTarget(path, state, path_index);
  size_t first_target_index = findTargetOnPath(path, first_target);
  RobotState second_target = acquireNewTarget(path, first_target, first_target_index);
  return { first_target, second_target };
}

RobotState SmoothControl::acquireNewTarget(const nav_msgs::PathConstPtr& path, const RobotState& state,
                                           size_t state_index) const
{
  geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;

  // target position
  RobotState newTarget{ end.x, end.y, tf::getYaw(path->poses[path->poses.size() - 1].pose.orientation) };

  double distance = 0;

  for (size_t i = state_index; i < path->poses.size() - 1; i++)
  {
    geometry_msgs::Point point1, point2;
    point1 = path->poses[i].pose.position;
    point2 = path->poses[i + 1].pose.position;
    double increment = igvc::get_distance(point1.x, point1.y, point2.x, point2.y);

    if (distance + increment > target_selection_options_.lookahead_dist)
    {
      newTarget.set_x(point2.x);
      newTarget.set_y(point2.y);
      newTarget.set_yaw(tf::getYaw(path->poses[i + 1].pose.orientation));
      return newTarget;
    }
    distance += increment;
  }

  return newTarget;
}

size_t SmoothControl::findTargetOnPath(const nav_msgs::PathConstPtr& path, const RobotState& target) const
{
  auto closest = std::min_element(path->poses.begin(), path->poses.end(),
                                  [&target](geometry_msgs::PoseStamped lhs, geometry_msgs::PoseStamped rhs) {
                                    return target.distTo(lhs.pose.position) < target.distTo(rhs.pose.position);
                                  });
  if (target.distTo(closest->pose.position) < target_selection_options_.target_move_threshold)
  {
    return static_cast<size_t>(closest - path->poses.begin());
  }
  ROS_WARN_THROTTLE(1, "Target moved more than target_move_threshold");
  return 0;
}

double SmoothControl::distAlongPath(const nav_msgs::PathConstPtr& path, size_t path_index, size_t target_index) const
{
  double distance = -1;

  for (size_t i = path_index; i < target_index; i++)
  {
    geometry_msgs::Point point1, point2;
    point1 = path->poses[i].pose.position;
    point2 = path->poses[i + 1].pose.position;
    distance += igvc::get_distance(point1.x, point1.y, point2.x, point2.y);
  }
  return distance;
}

bool SmoothControl::reachedTarget(const RobotState& state, const RobotState& target) const
{
  return state.distTo(target) < target_selection_options_.target_reached_distance;
}

size_t SmoothControl::getClosestIndex(const nav_msgs::PathConstPtr& path, const RobotState& state) const
{
  auto closest = std::min_element(path->poses.begin(), path->poses.end(),
                                  [&state](geometry_msgs::PoseStamped lhs, geometry_msgs::PoseStamped rhs) {
                                    return state.distTo(lhs.pose.position) < state.distTo(rhs.pose.position);
                                  });
  return static_cast<size_t>(closest - path->poses.begin());
}

double SmoothControl::blendCurvature(double distance, double K1, double K2) const
{
  // Check if we are in the "transition zone" from the first target to the second, and should begin blending K1 and K2
  double K;
  if (distance < curvature_blending_options_.blending_distance)
  {
    double blending_factor =
        (curvature_blending_options_.blending_distance - distance) / curvature_blending_options_.blending_distance;
    K = blending_factor * K1 + (1 - blending_factor) * K2;
  }
  else
  {  // Otherwise,
    K = K1;
  }
  return K;
}
