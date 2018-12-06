#ifndef NODEUTILS_HPP
#define NODEUTILS_HPP

#include <ros/ros.h>

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

}  // namespace igvc
#endif
