//
// Created by oswinso on 11/11/18.
//
#ifndef PARAMUTILS_CPP
#define PARAMUTILS_CPP

#include <ros/ros.h>
#include <stdlib.h>

namespace igvc
{
    void resolveError(const std::string &string);

    template<class T>
    void param(const ros::NodeHandle pNh, const std::string &param_name, T &param_val, const T &default_val)
    {
      if (!pNh.param(param_name, param_val, default_val)) {
        resolveError(param_name);
      }
    }

    template<class T>
    void getParam(const ros::NodeHandle &pNh, const std::string &param_name, T &param_val)
    {
      if (!pNh.getParam(param_name, param_val)) {
        resolveError(param_name);
      }
    }

    void resolveError(const std::string &param_name) {
      ROS_ERROR_STREAM("Missing parameter " << param_name << "... exiting");
      exit(EXIT_FAILURE);
    }
}
#endif

