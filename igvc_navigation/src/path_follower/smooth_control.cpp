#include "smooth_control.h"

void SmoothControl::getTrajectory(igvc_msgs::velocity_pair& vel, const nav_msgs::PathConstPtr& path,
                                  nav_msgs::Path& trajectory, const RobotState& start_pos, RobotState& target)
{
  RobotState state = start_pos;
  RobotState simulation_target;
  unsigned int path_index = 0;

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
    path_index = getClosestPosition(path, state);

    simulation_target = getTargetPosition(path, path_index, state);


    // Calculate control using Control Law
    Action action = getAction(state, simulation_target);
    action.dt = dt.toSec();

    if (i == 0)
    {
      target = simulation_target;
      getWheelVelocities(vel, action);
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

    pose.pose.position.x = simulation_target.x;
    pose.pose.position.y = simulation_target.y;
    pose.pose.orientation = simulation_target.quat();
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
  double v = action.v;
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
    resultant_pose << state.x + (cos(state.yaw * v * dt)), state.y + (sin(state.yaw) * v * dt), state.yaw;
  }

  state.setState(resultant_pose);
}

Action SmoothControl::getAction(const RobotState& state, const RobotState& target)
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

  // calculate angular velocity using radius of curvature and target velocity
  double w = K * target_velocity_;
  return Action{ target_velocity_, w, 0 };
}

RobotState SmoothControl::getTargetPosition(const nav_msgs::PathConstPtr& path, unsigned int path_index,
                                            const RobotState& state) const
{
  // target position
  double tar_x;
  double tar_y;
  double tar_yaw = 1.57;

  geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;

  double distance = 0;

  for (; path_index < path->poses.size() - 1; path_index++)
  {
    geometry_msgs::Point point1, point2;
    point1 = path->poses[path_index].pose.position;
    point2 = path->poses[path_index + 1].pose.position;
    double increment = igvc::get_distance(point1.x, point1.y, point2.x, point2.y);

    if (distance + increment > lookahead_dist_)
    {
      Eigen::Vector3d first(point1.x, point1.y, 0);
      Eigen::Vector3d second(point2.x, point2.y, 0);
      Eigen::Vector3d slope = (second - first) / increment;

      slope *= (lookahead_dist_ - distance);

      tar_x = first[0] + slope[0];
      tar_y = first[1] + slope[1];
      tar_yaw = tf::getYaw(path->poses[path_index].pose.orientation);
      if (tar_yaw == 0)
      {
        ROS_ERROR("WHYYYYY");
      }
      return RobotState{ tar_x, tar_y, tar_yaw };
    }
    distance += increment;
  }
  tar_x = end.x;
  tar_y = end.y;

  return RobotState{ tar_x, tar_y, tf::getYaw(path->poses[path->poses.size() - 2].pose.orientation) };
}

unsigned int SmoothControl::getClosestPosition(const nav_msgs::PathConstPtr& path, const RobotState& state) const
{
  double closest = state.distTo(path->poses[0].pose.position.x, path->poses[0].pose.position.y);

  for (unsigned int path_index = 0; path_index < path->poses.size(); path_index++)
  {
    double cur_dist = state.distTo(path->poses[path_index].pose.position.x, path->poses[path_index].pose.position.y);
    if (cur_dist <= closest)
    {
      closest = cur_dist;
    }
    else // Derivative < 0, found maximum
    {
      return path_index;
    }
  }
}

void SmoothControl::getWheelVelocities(igvc_msgs::velocity_pair& vel_msg, const Action& action) const
{
  vel_msg.left_velocity = action.v - action.w * axle_length_ / 2;
  vel_msg.right_velocity = action.v + action.w * axle_length_ / 2;
}

SmoothControl::SmoothControl(double k1, double k2, double axle_length, double simulation_frequency,
                             double target_velocity, double lookahead_dist, double simulation_horizon)
  : k1_{ k1 }
  , k2_{ k2 }
  , axle_length_{ axle_length }
  , simulation_frequency_{ simulation_frequency }
  , target_velocity_{ target_velocity }
  , lookahead_dist_{ lookahead_dist }
  , simulation_horizon_{ simulation_horizon }
{
  ros::NodeHandle pNh{ "~" };
  target_pub_ = pNh.advertise<nav_msgs::Path>("smooth_control/target", 1);
  closest_point_pub_ = pNh.advertise<nav_msgs::Path>("smooth_control/why2", 1);
}
