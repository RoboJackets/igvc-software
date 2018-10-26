#ifndef SMOOTH_CONTROL_H
#define SMOOTH_CONTROL_H

#define _USE_MATH_DEFINES

#include <igvc_msgs/velocity_pair.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <cmath>

// http://www-personal.umich.edu/~jongjinp/papers/Park-icra-11.pdf

class SmoothControl
{
public:
  double k1, k2;
  double axle_length;
  double rollOutTime;
  double v;

  void getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::Path& trajectory, Eigen::Vector3d cur_pos,
                     Eigen::Vector3d target);

private:
  void getAction(Eigen::Vector3d& result, Eigen::Vector3d cur_pos, Eigen::Vector3d target);
  void getResult(Eigen::Vector3d& result, Eigen::Vector3d cur_pos, Eigen::Vector3d action);
};
#endif
