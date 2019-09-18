#ifndef SRC_POINTCLOUD_FILTER_H
#define SRC_POINTCLOUD_FILTER_H

#include <pointcloud_filter/back_filter/back_filter.h>
#include <pointcloud_filter/pointcloud_filter_config.h>
#include <ros/ros.h>

namespace pointcloud_filter
{
class PointcloudFilter
{
public:
  PointcloudFilter(const ros::NodeHandle& nh = ros::NodeHandle);

private:
  ros::NodeHandle nh_;

  BackFilter back_filter_;
  PointcloudFilterConfig config_;
};
}  // namespace pointcloud_filter

#endif  // SRC_POINTCLOUD_FILTER_H
