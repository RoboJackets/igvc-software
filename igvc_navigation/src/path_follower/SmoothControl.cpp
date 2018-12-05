#include "SmoothControl.h"

void SmoothControl::getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::PathConstPtr path,
                                  nav_msgs::Path& trajectory, Eigen::Vector3d cur_pos)
{
  this->cur_pos << cur_pos;

  // TODO: Create initialize_variables() method
  unsigned int path_index;
  path_index = getClosestPosition(path);
  getTargetPosition(path, path_index);
  getLineOfSightAndHeading();
  getEgocentricAngles(path, path_index);

  double dt = rollOutTime / 10;

  for (int i = 0; i < 15; i++)
  {
    Eigen::Vector3d action;
    getAction(action);
    action[2] = dt;

    if (i == 0)
    {
      geometry_msgs::PoseStamped start;
      start.pose.position.x = this->cur_pos[0];
      start.pose.position.y = this->cur_pos[1];
      trajectory.poses.push_back(start);

      vel.left_velocity = action[0] - action[1] * axle_length / 2;
      vel.right_velocity = action[0] + action[1] * axle_length / 2;
    }

    getResult(path, action);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = this->cur_pos[0];
    pose.pose.position.y = this->cur_pos[1];
    trajectory.poses.push_back(pose);
  }
}

/**
Estimates the resultant pose given the current velocity and angular velocity
command for visualization purposes. Additionally, updates the egocentric polar
angles [delta, theta].

Makes extensive use of differential drive odometry equations to calculate resultant
pose.

Source: http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf
*/
void SmoothControl::getResult(nav_msgs::PathConstPtr path, Eigen::Vector3d action)
{
  double v = action[0];
  double w = action[1];
  double dt = action[2];

  Eigen::Vector3d resultant_pose;

  Eigen::Vector3d cur_pos;
  cur_pos << this->cur_pos;

  if (std::abs(w) > 1e-10)
  {
    // calculate instantaneous center of curvature
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
    fitToPolar(c[2]);

    resultant_pose << c[0], c[1], c[2];
  }
  else
  {
    resultant_pose << cur_pos[0] + (cos(cur_pos[2]) * v * dt), cur_pos[1] + (sin(cur_pos[2]) * v * dt), cur_pos[2];
  }

  this->cur_pos << resultant_pose;

  // update delta and theta
  unsigned int path_index = getClosestPosition(path);
  getTargetPosition(path, path_index);
  getLineOfSightAndHeading();
  getEgocentricAngles(path, path_index);
}

/**
Computes the radius of curvature to obtain a new angular velocity value.
*/
void SmoothControl::getAction(Eigen::Vector3d& result)
{
  // get egocentric polar coordinates for delta and theta
  double delta = delta_, theta = theta_;

  // adjust both angles to lie between -PI and PI
  fitToPolar(delta);
  fitToPolar(theta);

  // calculate the radius of curvature, K
  double d = sqrt(pow(target[0] - cur_pos[0], 2) + pow(target[1] - cur_pos[1], 2));
  double K = k2 * (delta - atan(-k1 * theta));
  K += (1 + (k1 / (1 + pow(k1 * theta, 2)))) * sin(delta);
  K /= -d;

  // calculate angular velocity using radius of curvature and target velocity
  double w = K * v;
  result[0] = v;
  result[1] = w;
}

/**
Calculates egocentric polar angles for smooth control law calculations:
    - delta, the angle between the line of sight and the current robot heading.
    - theta, the angle between the line of sight and the target heading
*/
void SmoothControl::getEgocentricAngles(nav_msgs::PathConstPtr path, unsigned int path_index)
{
  // get egocentric polar angle of los relative to heading
  double delta;
  compute_angle(delta, los_, heading_);
  this->delta_ = delta;

  // get i and j components of target orientation vector (res_orientation)
  double distance = 0;
  geometry_msgs::Point point1, point2;
  while (path_index < path->poses.size() - 1)
  {
    point1 = path->poses[path_index].pose.position;
    point2 = path->poses[path_index + 1].pose.position;
    double increment = get_distance(point1.x, point1.y, point2.x, point2.y);

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
  compute_angle(theta, los_, tar_orientation_);
  this->theta_ = theta;
}

/**
Calculate the line of sight (los) from the robot to the target position as
well as the current robot heading in vector format
*/
void SmoothControl::getLineOfSightAndHeading()
{
  double slope_x = target[0] - cur_pos[0];
  double slope_y = target[1] - cur_pos[1];

  Eigen::Vector3d los(slope_x, slope_y, 0);  // line of sight
  los.normalize();
  this->los_ << los;

  // get current robot heading in vector format
  Eigen::Vector3d heading(std::cos(cur_pos[2]), std::sin(cur_pos[2]), 0);
  this->heading_ << heading;
}

/**
Find the furthest point along trajectory that isn't further than the
lookahead distance. This is the target position.
*/
void SmoothControl::getTargetPosition(nav_msgs::PathConstPtr path, unsigned int path_index)
{
  // target position
  float tar_x, tar_y;

  geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;

  if (get_distance(cur_pos[0], cur_pos[1], end.x, end.y) > lookahead_dist)
  {
    double distance = 0;
    bool cont = true;

    while (cont && path_index < path->poses.size() - 1)
    {
      geometry_msgs::Point point1, point2;
      point1 = path->poses[path_index].pose.position;
      point2 = path->poses[path_index + 1].pose.position;
      double increment = get_distance(point1.x, point1.y, point2.x, point2.y);

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
  target << tar_x, tar_y, 0;
}

/**
Find the index of the closest point on the path.
*/
unsigned int SmoothControl::getClosestPosition(nav_msgs::PathConstPtr path)
{
  unsigned int path_index = 0;
  double closest = get_distance(cur_pos[0], cur_pos[1], path->poses[0].pose.position.x, path->poses[0].pose.position.y);

  double temp = get_distance(cur_pos[0], cur_pos[1], path->poses[path_index].pose.position.x,
                             path->poses[path_index].pose.position.y);

  while (path_index < path->poses.size() && temp <= closest)
  {
    if (temp < closest)
    {
      closest = temp;
    }
    path_index++;
    temp = get_distance(cur_pos[0], cur_pos[1], path->poses[path_index].pose.position.x,
                        path->poses[path_index].pose.position.y);
  }

  return path_index;
}

/* TODO: move these methods into NodeUtils */

/**
Adjust angle to lie within the polar range [-PI, PI]
*/
void SmoothControl::fitToPolar(double& angle)
{
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;
}

/**
Calculates euclidian distance between two points
*/
double SmoothControl::get_distance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

/**
Computes the egocentric polar angle of vec2 wrt vec1 in 2D, that is:
  - clockwise: negative
  - counter-clockwise: positive

source: https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
*/
void SmoothControl::compute_angle(double& angle, Eigen::Vector3d vec2, Eigen::Vector3d vec1)
{
  double dot = vec2[0] * vec1[0] + vec2[1] * vec1[1];  // dot product - proportional to cos
  double det = vec2[0] * vec1[1] - vec2[1] * vec1[0];  // determinant - proportional to sin

  angle = atan2(det, dot);
}
