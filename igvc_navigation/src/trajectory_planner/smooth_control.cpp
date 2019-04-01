#include "smooth_control.h"
#include <algorithm>

void SmoothControl::getPath(igvc_msgs::velocity_pair &vel, const nav_msgs::PathConstPtr &path,
                            nav_msgs::Path &trajectory, const RobotState &start_pos, RobotState &target)
{
  RobotState state = start_pos;
  std::optional<RobotState> simulation_target = target_;
  size_t path_index = 0;

  ros::Time time = path->header.stamp;

  // Store starting position in trajectory
  geometry_msgs::PoseStamped start;
  start.pose.position.x = start_pos.x;
  start.pose.position.y = start_pos.y;
  start.pose.orientation = start_pos.quat();
  start.header.stamp = time;
  trajectory.poses.emplace_back(start);

  // Calculate timesteps
  static ros::Duration dt = ros::Duration(1 / simulation_frequency_);

  // Visualization
  nav_msgs::Path closest_point_path;
  nav_msgs::Path targets_path;

  closest_point_path.header = path->header;
  targets_path.header = path->header;
  for (int i = 0; i < simulation_horizon_ * simulation_frequency_ && path_index < path->poses.size() - 1; i++)
  {
    // Get target
    path_index = getClosestIndex(path, state);
    RobotState second_target;
    std::tie(simulation_target, second_target) = getTargetPosition(path, state, simulation_target, path_index);

    //    ROS_INFO_STREAM(i << "(" << path_index << ") --> " << getClosestIndex(path, *simulation_target));

    // Calculate control using Control Law
    Action action =
        getAction(state, *simulation_target, second_target, dt);  // TODO: Change this dt from simulation dt to real dt
    //    ROS_INFO_STREAM("Action: " << action.velocity.v_start << " -> " << action.velocity.v_stop);
    action.dt = dt.toSec();

    if (i == 0)
    {
      target_ = *simulation_target;
      target = *simulation_target;
      getWheelVelocities(vel, action.w, action.velocity.average());
    }

    propogateState(action, state);
    time += dt;

    // For Visualization
    geometry_msgs::PoseStamped pose;
    pose.header = path->header;
    pose.pose.position.x = path->poses[path_index].pose.position.x;
    pose.pose.position.y = path->poses[path_index].pose.position.y;
    pose.pose.orientation = path->poses[path_index].pose.orientation;
    closest_point_path.poses.emplace_back(pose);

    pose.pose.position.x = simulation_target->x;
    pose.pose.position.y = simulation_target->y;
    pose.pose.orientation = simulation_target->quat();
    targets_path.poses.emplace_back(pose);

    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.orientation = state.quat();
    pose.header.stamp = start.header.stamp;
    trajectory.poses.emplace_back(pose);
  }
  target_pub_.publish(targets_path);
  closest_point_pub_.publish(closest_point_path);
}

void SmoothControl::propogateState(const Action& action, RobotState& state)
{
  double v = action.velocity.average();
  double w = action.w;
  double dt = action.dt;

  // pose after applying action to current pose
  Eigen::Vector3d resultant_pose;

  if (std::abs(w) > 1e-10)
  {
    // calculate instantaneous center of curvature (ICC = [ICCx, ICCy])
    double R = v / w;
    double ICCx = state.x - (R * sin(state.yaw));
    double ICCy = state.y + (R * cos(state.yaw));

    using namespace Eigen;
    Matrix3d T;
    double wdt = w * dt;
    T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
    Vector3d a(state.x - ICCx, state.y - ICCy, state.yaw);
    Vector3d b = T * a;
    Vector3d c = b + Vector3d(ICCx, ICCy, wdt);
    igvc::fit_to_polar(c[2]);

    resultant_pose << c[0], c[1], c[2];
  }
  else
  {
    resultant_pose << state.x + (cos(state.yaw) * v * dt), state.y + (sin(state.yaw) * v * dt), state.yaw;
  }

  state.setState(resultant_pose);

  igvc_msgs::velocity_pair wheel_velocity;
  getWheelVelocities(wheel_velocity, action.w, action.velocity.v_stop);
  state.setVelocity(wheel_velocity);
}

Action SmoothControl::getAction(const RobotState& state, const RobotState& target, const RobotState& second_target,
                                const ros::Duration& dt) const
{
  double K_first = getCurvature(state, target);
  double K_second = getCurvature(state, second_target);
  double distance = state.distTo(target);
  return motionProfile(state, distance, K_first, K_second, dt);
}

double SmoothControl::getCurvature(const RobotState& state, const RobotState& target) const
{
  double line_of_sight = atan2(target.y - state.y, target.x - state.x);

  double delta = state.yaw - line_of_sight;
  double theta = target.yaw - line_of_sight;

  // adjust both angles to lie between -PI and PI
  igvc::fit_to_polar(delta);
  igvc::fit_to_polar(theta);

  // calculate the radius of curvature, K
  double d = state.distTo(target);  // euclidian distance to target
  double K = k2_ * (delta - atan(-k1_ * theta));
  K += (1 + (k1_ / (1 + pow(k1_ * theta, 2)))) * sin(delta);
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
    // If we haven't reached the target
    if (!reachedTarget(state, *target))
    {
      size_t target_index = findTargetOnPath(path, *target);
      RobotState newTarget{ path->poses[target_index] };
      // If the target is found, and the target isn't behind us, and the target is closer to target + 1 than we are
      double state_dist_target_next = state.distTo(path->poses[target_index + 1].pose.position);
      double target_dist_target_next = target->distTo(path->poses[target_index + 1].pose.position);
      if (target_index != 0 && target_index >= path_index && target_dist_target_next < state_dist_target_next &&
          distAlongPath(path, path_index, target_index) < lookahead_dist_)
      {
        RobotState first_target = newTarget;
        RobotState second_target = acquireNewTarget(path, first_target, target_index);
        return { first_target, second_target };
      }
    }
  }
  RobotState first_target = acquireNewTarget(path, state, path_index);
  size_t first_target_index = findTargetOnPath(path, first_target);
  RobotState second_target = acquireNewTarget(path, first_target, first_target_index);
  return { first_target, second_target };
}

SmoothControl::SmoothControl(double k1, double k2, double axle_length, double simulation_frequency,
                             double target_velocity, double lookahead_dist, double simulation_horizon,
                             double target_reached_distance, double target_move_threshold, double acceleration_limit,
                             double beta, double lambda, double transition_distance)
  : k1_{ k1 }
  , k2_{ k2 }
  , axle_length_{ axle_length }
  , simulation_frequency_{ simulation_frequency }
  , target_velocity_{ target_velocity }
  , lookahead_dist_{ lookahead_dist }
  , simulation_horizon_{ simulation_horizon }
  , target_reached_distance_{ target_reached_distance }
  , target_move_threshold_{ target_move_threshold }
  , acceleration_limit_{ acceleration_limit }
  , beta_{ beta }
  , lambda_{ lambda }
  , transition_distance_{ transition_distance }
{
  ros::NodeHandle pNh{ "~" };
  target_pub_ = pNh.advertise<nav_msgs::Path>("smooth_control/target", 1);
  closest_point_pub_ = pNh.advertise<nav_msgs::Path>("smooth_control/closest_point", 1);
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

    if (distance + increment > lookahead_dist_)
    {
      //      Eigen::Vector3d first(point1.x, point1.y, 0);
      //      Eigen::Vector3d second(point2.x, point2.y, 0);
      //      Eigen::Vector3d slope = (second - first) / increment;
      //
      //      slope *= (lookahead_dist_ - distance);

      newTarget.x = point2.x;
      newTarget.y = point2.y;
      newTarget.yaw = tf::getYaw(path->poses[i + 1].pose.orientation);
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
  if (target.distTo(closest->pose.position) < target_move_threshold_)
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
  return state.distTo(target) < target_reached_distance_;
}

size_t SmoothControl::getClosestIndex(const nav_msgs::PathConstPtr& path, const RobotState& state) const
{
  auto closest = std::min_element(path->poses.begin(), path->poses.end(),
                                  [&state](geometry_msgs::PoseStamped lhs, geometry_msgs::PoseStamped rhs) {
                                    return state.distTo(lhs.pose.position) < state.distTo(rhs.pose.position);
                                  });
  return static_cast<size_t>(closest - path->poses.begin());
}

Action SmoothControl::motionProfile(const RobotState& state, double distance, double K1, double K2,
                                    const ros::Duration& dt) const
{
  // Check if we are in the "transition zone" from the first target to the second, and should begin blending K1 and K2
  double K;
  if (distance < transition_distance_)
  {
    double blending_factor = (transition_distance_ - distance) / transition_distance_;
    K = blending_factor * K1 + (1 - blending_factor) * K2;
  }
  else
  {  // Otherwise,
    K = K1;
  }

  double w = K * target_velocity_;
  igvc_msgs::velocity_pair wheel_velocity;
  getWheelVelocities(wheel_velocity, w, v);
  //  return toAction({ state.velocity.left, wheel_velocity.left_velocity}, { state.velocity.right,
  //  wheel_velocity.right_velocity });

  //  ROS_INFO_STREAM("w: " << w << ", K: " << K << ", w/v: " << w/target_velocity_);

  // Check out which wants greater acceleration, and cap it to acceleration_limit_
  double left_acceleration = (wheel_velocity.left_velocity - state.velocity.left) / dt.toSec();
  double right_acceleration = (wheel_velocity.right_velocity - state.velocity.right) / dt.toSec();
  double capped_left;
  double capped_right;
  if (std::abs(left_acceleration) > std::abs(right_acceleration) && std::abs(left_acceleration) > acceleration_limit_)
  {
    double actual_acceleration_left = std::copysign(acceleration_limit_, left_acceleration);
    double actual_acceleration_right =
        std::copysign((actual_acceleration_left / left_acceleration) * right_acceleration, right_acceleration);
    capped_left = state.velocity.left + dt.toSec() * actual_acceleration_left;
    capped_right = state.velocity.right + dt.toSec() * actual_acceleration_right;
  }
  else if (std::abs(right_acceleration) > std::abs(left_acceleration) &&
           std::abs(right_acceleration) > acceleration_limit_)
  {
    double actual_acceleration_right = std::copysign(acceleration_limit_, right_acceleration);
    double actual_acceleration_left =
        std::copysign((actual_acceleration_right / right_acceleration) * left_acceleration, left_acceleration);
    capped_right = state.velocity.right + dt.toSec() * actual_acceleration_right;
    capped_left = state.velocity.left + dt.toSec() * actual_acceleration_left;
  }
  else
  {
    capped_left = wheel_velocity.left_velocity;
    capped_right = wheel_velocity.right_velocity;
  }
  return toAction({ state.velocity.left, capped_left }, { state.velocity.right, capped_right });
}

void SmoothControl::getWheelVelocities(igvc_msgs::velocity_pair& vel_msg, double w, double v) const
{
  vel_msg.left_velocity = v - w * axle_length_ / 2;
  vel_msg.right_velocity = v + w * axle_length_ / 2;
}

Action SmoothControl::toAction(VelocityProfile left, VelocityProfile right) const
{
  //  ROS_INFO_STREAM(left.v_start << " -> " << left.v_stop << ", " << right.v_start << " -> " << right.v_stop);
  Action action{};
  action.w = (right.average() - left.average()) / axle_length_;
  //  ROS_INFO_STREAM("new K: " << action.w / ((left.average() + right.average())/2));
  action.velocity.v_start = (left.v_start + right.v_start) / 2;
  action.velocity.v_stop = (left.v_stop + right.v_stop) / 2;
  return action;
}
