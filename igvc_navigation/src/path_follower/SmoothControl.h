/**
Implementation of smooth control law derived from:
A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment

Paper found here:

https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
*/

#ifndef SMOOTH_CONTROL_H
#define SMOOTH_CONTROL_H

#define _USE_MATH_DEFINES

#include <igvc_msgs/velocity_pair.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <cmath>

class SmoothControl
{
public:
  double k1, k2;
  double axle_length;
  double rollOutTime;
  double v;
  double lookahead_dist;

  double cur_theta;
  double tar_theta;

  Eigen::Vector3d los;
  Eigen::Vector3d heading;
  Eigen::Vector3d tar_orientation;

  Eigen::Vector3d cur_pos;
  Eigen::Vector3d target;

  void getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::PathConstPtr path,
        nav_msgs::Path& trajectory, Eigen::Vector3d cur_pos)

  void getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::Path& trajectory,
        Eigen::Vector3d cur_pos, Eigen::Vector3d target, Eigen::Vector2d egocentric_heading);

private:
  void getAction(Eigen::Vector3d& result, Eigen::Vector3d cur_pos,
        Eigen::Vector3d target, Eigen::Vector2d egocentric_heading);
  void getResult(Eigen::Vector3d& result, Eigen::Vector2d& egocentric_heading,
        Eigen::Vector3d cur_pos, Eigen::Vector3d action);

  // TODO: move these methods into NodeUtils.
  void fitToPolar(double& angle);
  double get_distance(double x1, double y1, double x2, double y2)
  void compute_angle(float& angle, Eigen::Vector3d vec2, Eigen::Vector3d vec1)
};
#endif
