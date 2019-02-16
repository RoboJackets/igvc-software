#include "SmoothControl.h"

void SmoothControl::getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::PathConstPtr path,
                                  nav_msgs::Path& trajectory, RobotState cur_pos)
{
  this->cur_pos = cur_pos;
  this->init_pos_ = cur_pos;

  unsigned int path_index;
  path_index = getClosestPosition(path);
  getTargetPosition(path, path_index);
  getLineOfSightAndHeading();
  getEgocentricAngles(path, path_index);

  // distance to target
  double tar_dist = this->cur_pos.distTo(this->target);
  // calculate timestep
  double est_time = tar_dist / v;
  double dt = est_time / granularity;

  for (int i = 0; i < granularity; i++)
  {
    Eigen::Vector3d action;
    getAction(action);
    action[2] = dt;

    if (i == 0)
    {
      geometry_msgs::PoseStamped start;
      start.pose.position.x = this->cur_pos.x;
      start.pose.position.y = this->cur_pos.y;
      trajectory.poses.push_back(start);

      vel.left_velocity = action[0] - action[1] * axle_length / 2;
      vel.right_velocity = action[0] + action[1] * axle_length / 2;
    }

    getResult(path, action);

    // don't visualize trajectory past the motion target
    double dist_from_start = this->cur_pos.distTo(this->init_pos_);
    if (dist_from_start > tar_dist)
    {
      return;
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = this->cur_pos.x;
    pose.pose.position.y = this->cur_pos.y;
    trajectory.poses.push_back(pose);
  }
}

void SmoothControl::getResult(nav_msgs::PathConstPtr path, Eigen::Vector3d action)
{
  double v = action[0];
  double w = action[1];
  double dt = action[2];

  // pose after applying action to current pose
  Eigen::Vector3d resultant_pose;

  Eigen::Vector3d cur_pos;
  cur_pos = this->cur_pos.getVector3d();

  if (std::abs(w) > 1e-10)
  {
    // calculate instantaneous center of curvature (ICC = [ICCx, ICCy])
    double R = v / w;
    double ICCx = cur_pos[0] - (R * sin(cur_pos[2]));
    double ICCy = cur_pos[1] + (R * cos(cur_pos[2]));

    using namespace Eigen;
    Matrix3d T;
    double wdt = w * dt;
    T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
    Vector3d a(cur_pos[0] - ICCx, cur_pos[1] - ICCy, cur_pos[2]);
    Vector3d b = T * a;
    Vector3d c = b + Vector3d(ICCx, ICCy, wdt);
    igvc::fit_to_polar(c[2]);

    resultant_pose << c[0], c[1], c[2];
  }
  else
  {
    resultant_pose << cur_pos[0] + (cos(cur_pos[2]) * v * dt), cur_pos[1] + (sin(cur_pos[2]) * v * dt), cur_pos[2];
  }

  this->cur_pos.setState(resultant_pose);

  // update delta and theta
  unsigned int path_index = getClosestPosition(path);
  getLineOfSightAndHeading();
  getEgocentricAngles(path, path_index);
}

void SmoothControl::getAction(Eigen::Vector3d& result)
{
  // get egocentric polar coordinates for delta and theta
  double delta = delta_, theta = theta_;

  // adjust both angles to lie between -PI and PI
  igvc::fit_to_polar(delta);
  igvc::fit_to_polar(theta);

  // calculate the radius of curvature, K
  double d = cur_pos.distTo(target);  // euclidian distance to target
  double K = k2 * (delta - atan(-k1 * theta));
  K += (1 + (k1 / (1 + pow(k1 * theta, 2)))) * sin(delta);
  K /= -d;

  // calculate angular velocity using radius of curvature and target velocity
  double w = K * v;
  result[0] = v;
  result[1] = w;
}

void SmoothControl::getEgocentricAngles(nav_msgs::PathConstPtr path, unsigned int path_index)
{
  // get egocentric polar angle of los relative to heading
  double delta;
  igvc::compute_angle(delta, los_, heading_);
  this->delta_ = delta;

  // get i and j components of target orientation vector (res_orientation)
  double distance = 0;
  geometry_msgs::Point point1, point2;
  while (path_index < path->poses.size() - 1)
  {
    point1 = path->poses[path_index].pose.position;
    point2 = path->poses[path_index + 1].pose.position;
    double increment = igvc::get_distance(point1.x, point1.y, point2.x, point2.y);

    if (distance + increment > lookahead_dist)
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
  this->tar_orientation_ << tar_orientation;

  // get egocentric polar angle of los relative to target orientation
  double theta;
  igvc::compute_angle(theta, los_, tar_orientation_);
  this->theta_ = theta;
}

void SmoothControl::getLineOfSightAndHeading()
{
  double slope_x = target.x - cur_pos.x;
  double slope_y = target.y - cur_pos.y;

  // line of sight
  Eigen::Vector3d los(slope_x, slope_y, 0);
  los.normalize();
  this->los_ << los;

  // get current robot heading in vector format
  Eigen::Vector3d heading(std::cos(this->cur_pos.yaw), std::sin(this->cur_pos.yaw), 0);
  this->heading_ << heading;
}

void SmoothControl::getTargetPosition(nav_msgs::PathConstPtr path, unsigned int path_index)
{
  // target position
  float tar_x, tar_y;

  geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;

  if (cur_pos.distTo(end.x, end.y) > lookahead_dist)
  {
    double distance = 0;
    bool cont = true;

    while (cont && path_index < path->poses.size() - 1)
    {
      geometry_msgs::Point point1, point2;
      point1 = path->poses[path_index].pose.position;
      point2 = path->poses[path_index + 1].pose.position;
      double increment = igvc::get_distance(point1.x, point1.y, point2.x, point2.y);

      if (distance + increment > lookahead_dist)
      {
        cont = false;
        Eigen::Vector3d first(point1.x, point1.y, 0);
        Eigen::Vector3d second(point2.x, point2.y, 0);
        Eigen::Vector3d slope = second - first;

        slope /= increment;
        slope *= (distance - lookahead_dist) + increment;

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
  this->target.setState(Eigen::Vector3d{ tar_x, tar_y, 0 });
}

unsigned int SmoothControl::getClosestPosition(nav_msgs::PathConstPtr path)
{
  unsigned int path_index = 0;
  double closest = cur_pos.distTo(path->poses[0].pose.position.x, path->poses[0].pose.position.y);

  double temp = cur_pos.distTo(path->poses[path_index].pose.position.x, path->poses[path_index].pose.position.y);

  while (path_index < path->poses.size() && temp <= closest)
  {
    if (temp < closest)
    {
      closest = temp;
    }
    path_index++;
    temp = cur_pos.distTo(path->poses[path_index].pose.position.x, path->poses[path_index].pose.position.y);
  }

  return path_index;
}
