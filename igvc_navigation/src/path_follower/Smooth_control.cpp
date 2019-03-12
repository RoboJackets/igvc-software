#include "Smooth_control.h"

void Smooth_control::get_trajectory(igvc_msgs::velocity_pair& vel, nav_msgs::PathConstPtr path,
                                    nav_msgs::Path& trajectory, RobotState state, RobotState& target)
{
  RobotState start_state = state;

  unsigned int path_index;
  Eigen::Vector3d los;
  Eigen::Vector3d heading;

  path_index = get_closest_position(path, state);
  target.setState(get_target_position(path, path_index, state));
  get_los_and_heading(los, heading, state, target);
  Egocentric_angles angles = get_egocentric_angles(path, path_index, los, heading);

  // distance to target
  double tar_dist = state.distTo(target);
  // calculate timestep
  double est_time = tar_dist / m_target_velocity;
  double dt = est_time / m_granularity;

  for (int i = 0; i < m_granularity; i++)
  {
    Action action = get_action(angles.delta, angles.theta, state, target);
    action.dt = dt;

    if (i == 0)
    {
      geometry_msgs::PoseStamped start;
      start.pose.position.x = state.x;
      start.pose.position.y = state.y;
      trajectory.poses.push_back(start);

      vel.left_velocity = action.v - action.w * m_axle_length / 2;
      vel.right_velocity = action.v + action.w * m_axle_length / 2;
    }

    propogate_state(path, action, state);

    // update delta and theta
    path_index = get_closest_position(path, state);
    get_los_and_heading(los, heading, state, target);
    angles = get_egocentric_angles(path, path_index, los, heading);

    // don't visualize trajectory past the motion target
    double dist_from_start = start_state.distTo(state);
    if (dist_from_start > tar_dist)
    {
      return;
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    trajectory.poses.push_back(pose);
  }
}

void Smooth_control::propogate_state(const nav_msgs::PathConstPtr& path, const Action& action, RobotState& state)
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

Action Smooth_control::get_action(double delta, double theta, const RobotState& state, const RobotState& target)
{
  // get egocentric polar coordinates for delta and theta

  // adjust both angles to lie between -PI and PI
  igvc::fit_to_polar(delta);
  igvc::fit_to_polar(theta);

  // calculate the radius of curvature, K
  double d = state.distTo(target);  // euclidian distance to target
  double K = m_k2 * (delta - atan(-m_k1 * theta));
  K += (1 + (m_k1 / (1 + pow(m_k1 * theta, 2)))) * sin(delta);
  K /= -d;

  // calculate angular velocity using radius of curvature and target velocity
  double w = K * m_target_velocity;
  return Action{ m_target_velocity, w, 0 };
}

Egocentric_angles Smooth_control::get_egocentric_angles(const nav_msgs::PathConstPtr& path, unsigned int path_index,
                                                        const Eigen::Vector3d& los, const Eigen::Vector3d& heading)
{
  // get egocentric polar angle of los relative to heading
  double delta;
  igvc::compute_angle(delta, los, heading);

  // get i and j components of target orientation vector (res_orientation)
  double distance = 0;
  geometry_msgs::Point point1, point2;
  while (path_index < path->poses.size() - 1)
  {
    point1 = path->poses[path_index].pose.position;
    point2 = path->poses[path_index + 1].pose.position;
    double increment = igvc::get_distance(point1.x, point1.y, point2.x, point2.y);

    if (distance + increment > m_lookahead_dist)
    {
      break;
    }

    path_index++;
    distance += increment;
  }

  double pose_x = point2.x - point1.x;
  double pose_y = point2.y - point1.y;

  Eigen::Vector3d tar_orientation(pose_x, pose_y, 0);  // target orientation
  tar_orientation.normalize();

  // get egocentric polar angle of los relative to target orientation
  double theta;
  igvc::compute_angle(theta, los, tar_orientation);
  return Egocentric_angles{ delta, theta };
}

void Smooth_control::get_los_and_heading(Eigen::Vector3d& los, Eigen::Vector3d& heading, const RobotState& state,
                                         const RobotState& target)
{
  double slope_x = target.x - state.x;
  double slope_y = target.y - state.y;

  // line of sight
  los << slope_x, slope_y, 0;
  los.normalize();

  // get current robot heading in vector format
  heading << std::cos(state.yaw), std::sin(state.yaw), 0;
}

Eigen::Vector3d Smooth_control::get_target_position(const nav_msgs::PathConstPtr& path, unsigned int path_index,
                                                    const RobotState& state)
{
  // target position
  double tar_x, tar_y;

  geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;

  if (state.distTo(end.x, end.y) > m_lookahead_dist)
  {
    double distance = 0;
    bool cont = true;

    while (cont && path_index < path->poses.size() - 1)
    {
      geometry_msgs::Point point1, point2;
      point1 = path->poses[path_index].pose.position;
      point2 = path->poses[path_index + 1].pose.position;
      double increment = igvc::get_distance(point1.x, point1.y, point2.x, point2.y);

      if (distance + increment > m_lookahead_dist)
      {
        cont = false;
        Eigen::Vector3d first(point1.x, point1.y, 0);
        Eigen::Vector3d second(point2.x, point2.y, 0);
        Eigen::Vector3d slope = second - first;

        slope /= increment;
        slope *= (distance - m_lookahead_dist) + increment;

        slope += first;
        tar_x = slope[0];
        tar_y = slope[1];
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
    tar_x = end.x;
    tar_y = end.y;
  }

  // load target position and arbitrary angle into the motion target vector
  return Eigen::Vector3d{ tar_x, tar_y, 0 };
}

unsigned int Smooth_control::get_closest_position(const nav_msgs::PathConstPtr& path, const RobotState& state)
{
  unsigned int path_index = 0;
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

Smooth_control::Smooth_control(double k1, double k2, double axle_length, double granularity, double target_velocity,
                               double lookahead_dist)
  : m_k1{ k1 }
  , m_k2{ k2 }
  , m_axle_length{ axle_length }
  , m_granularity{ granularity }
  , m_target_velocity{ target_velocity }
  , m_lookahead_dist{ lookahead_dist }
{
}
