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

  void getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::Path& trajectory,
        Eigen::Vector3d cur_pos, Eigen::Vector3d target);

private:
  void getAction(Eigen::Vector3d& result, Eigen::Vector3d cur_pos, Eigen::Vector3d target);
  void getResult(Eigen::Vector3d& result, Eigen::Vector3d cur_pos, Eigen::Vector3d action);
  void fitToPolar(double& angle);
};
#endif
