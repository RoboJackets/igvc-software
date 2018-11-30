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

  Eigen::Vector3d cur_pos;
  Eigen::Vector3d target;

  void getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::PathConstPtr path,
        nav_msgs::Path& trajectory, Eigen::Vector3d cur_pos);

private:
  Eigen::Vector3d _los;
  Eigen::Vector3d _heading;
  Eigen::Vector3d _tar_orientation;

  double _delta;
  double _theta;

  void getAction(Eigen::Vector3d& result);
  void getResult(nav_msgs::PathConstPtr path, Eigen::Vector3d action);
  unsigned int getClosestPosition(nav_msgs::PathConstPtr path);
  void getTargetPosition(nav_msgs::PathConstPtr path, unsigned int path_index);
  void getLineOfSightAndHeading();
  void getEgocentricAngles(nav_msgs::PathConstPtr path, unsigned int path_index);

  // TODO: move these methods into NodeUtils.
  void fitToPolar(double& angle);
  double get_distance(double x1, double y1, double x2, double y2);
  void compute_angle(double& angle, Eigen::Vector3d vec2, Eigen::Vector3d vec1);
};
#endif
