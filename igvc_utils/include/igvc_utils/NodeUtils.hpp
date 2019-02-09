#ifndef NODEUTILS_HPP
#define NODEUTILS_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <tuple>

namespace igvc
{
template <class T>
void param(const ros::NodeHandle &pNh, const std::string &param_name, T &param_val, const T &default_val)
{
  if (!pNh.param(param_name, param_val, default_val))
  {
    ROS_ERROR_STREAM("Missing parameter " << param_name << " from " << pNh.getNamespace()
                                          << ". Continuing with default values " << default_val);
  }
}

template <class T>
void getParam(const ros::NodeHandle &pNh, const std::string &param_name, T &param_val)
{
  if (!pNh.getParam(param_name, param_val))
  {
    ROS_ERROR_STREAM("Missing parameter " << param_name << " from " << pNh.getNamespace() << "... exiting");
    ros::shutdown();
  }
}

/**
Calculates euclidian distance between two points
*/
template <typename T>
inline T get_distance(T x1, T y1, T x2, T y2)
{
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

/**
Calculates euclidian distance between two points, taking tuples for each
(x,y) point as argument
*/
template <typename T>
inline T get_distance(const std::tuple<T, T> &p1, const std::tuple<T, T> &p2)
{
  T x1, y1;
  std::tie(x1, y1) = p1;

  T x2, y2;
  std::tie(x2, y2) = p2;

  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// symmetric round up
// Bias: away from zero
template <typename T>
T ceil0(const T &value)
{
  T result = std::ceil(std::fabs(value));
  return (value < 0.0) ? -result : result;
}

/**
Adjust angle to lie within the polar range [-PI, PI]
*/
inline void fit_to_polar(double &angle)
{
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;
}

/**
Computes the egocentric polar angle of vec2 wrt vec1 in 2D, that is:
  - clockwise: negative
  - counter-clockwise: positive

source: https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
*/
inline void compute_angle(double &angle, Eigen::Vector3d vec2, Eigen::Vector3d vec1)
{
  double dot = vec2[0] * vec1[0] + vec2[1] * vec1[1];  // dot product - proportional to cos
  double det = vec2[0] * vec1[1] - vec2[1] * vec1[0];  // determinant - proportional to sin

  angle = atan2(det, dot);
}

}  // namespace igvc
#endif
