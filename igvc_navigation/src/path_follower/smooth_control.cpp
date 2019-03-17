#include "smooth_control.h"

void SmoothControl::getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::PathConstPtr path,
                                  nav_msgs::Path& trajectory, const RobotState& start_pos, RobotState& target)
{
  RobotState state = start_pos;
  RobotState simulation_target;
  unsigned int path_index = 0;

  ros::Time time = ros::Time::now();

  // Store starting position in trajectory
  geometry_msgs::PoseStamped start;
  start.pose.position.x = start_pos.x;
  start.pose.position.y = start_pos.y;
  start.pose.orientation = start_pos.quat();
  start.header.stamp = time;
  trajectory.poses.emplace_back(start);

  // Calculate timesteps
  static ros::Duration dt = ros::Duration(simulation_horizon_ / granularity_);
  time += dt;

  for (int i = 0; i < granularity_ && path_index < path->poses.size(); i++)
  {
    // Get target
    path_index = getClosestPosition(path, state, path_index);
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

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.orientation = state.quat();
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.pose.orientation, quat);
    double r, p, y;
    tf::Matrix3x3(quat).getRPY(r, p, y);
    pose.header.stamp = time;
    trajectory.poses.emplace_back(pose);
  }
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
  // delta = line_of_sight - heading
  double delta = line_of_sight - state.yaw;
  // theta = line_of_sight - target heading
  double theta = line_of_sight - target.yaw;

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
  double tar_x, tar_y, tar_yaw;

  geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;

  if (state.distTo(end.x, end.y) > lookahead_dist_)
  {
    double distance = 0;
    bool cont = true;

    while (cont && path_index < path->poses.size() - 1)
    {
      geometry_msgs::Point point1, point2;
      point1 = path->poses[path_index].pose.position;
      point2 = path->poses[path_index + 1].pose.position;
      double increment = igvc::get_distance(point1.x, point1.y, point2.x, point2.y);

      if (distance + increment > lookahead_dist_)
      {
        cont = false;
        Eigen::Vector3d first(point1.x, point1.y, 0);
        Eigen::Vector3d second(point2.x, point2.y, 0);
        Eigen::Vector3d slope = (second - first) / increment;

        slope *= (lookahead_dist_ - distance);

        tar_x = first[0] + slope[0];
        tar_y = first[1] + slope[1];
        tar_yaw = atan2(slope[1], slope[0]);
      }
      else
      {
        path_index++;
        distance += increment;
      }
    }
  }
  else
  {
    double delta_x = end.x - path->poses[path->poses.size() - 2].pose.position.x;
    double delta_y = end.y - path->poses[path->poses.size() - 2].pose.position.y;
    tar_x = end.x;
    tar_y = end.y;
    tar_yaw = atan2(delta_y, delta_x);
  }

  return RobotState{ tar_x, tar_y, tar_yaw };
}

unsigned int SmoothControl::getClosestPosition(const nav_msgs::PathConstPtr& path, const RobotState& state,
                                               unsigned int start_index)
{
  unsigned int path_index = start_index;
  double closest = state.distTo(path->poses[0].pose.position.x, path->poses[0].pose.position.y);

  double temp = state.distTo(path->poses[path_index].pose.position.x, path->poses[path_index].pose.position.y);

  while (path_index < path->poses.size() && temp <= closest)
  {
    if (temp < closest)
    {
      closest = temp;
    }
    path_index++;
    temp = state.distTo(path->poses[path_index].pose.position.x, path->poses[path_index].pose.position.y);
  }

  return path_index;
}

void SmoothControl::getWheelVelocities(igvc_msgs::velocity_pair& vel_msg, const Action& action) const
{
  vel_msg.left_velocity = action.v - action.w * axle_length_ / 2;
  vel_msg.right_velocity = action.v + action.w * axle_length_ / 2;
}

SmoothControl::SmoothControl(double k1, double k2, double axle_length, double granularity, double target_velocity,
                             double lookahead_dist, double simulation_horizon)
  : k1_{ k1 }
  , k2_{ k2 }
  , axle_length_{ axle_length }
  , granularity_{ granularity }
  , target_velocity_{ target_velocity }
  , lookahead_dist_{ lookahead_dist }
  , simulation_horizon_{ simulation_horizon }
{
}
