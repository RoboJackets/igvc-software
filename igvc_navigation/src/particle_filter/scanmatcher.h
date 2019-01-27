#ifndef PROJECT_SCANMATCHER_H
#define PROJECT_SCANMATCHER_H
#pragma once

#include <boost/circular_buffer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>
#include <igvc_utils/NodeUtils.hpp>

class Scanmatcher
{
public:
  Scanmatcher(ros::NodeHandle pNh) : m_last_cloud(nullptr)
  {
    igvc::getParam(pNh, "scanmatcher/use_guess", m_use_guess);
  }
  double scanmatch(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input, tf::Transform transform,
                const tf::Transform& guess);

private:
  bool m_use_guess = false;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_last_cloud;
};

#endif  // PROJECT_SCANMATCHER_H
