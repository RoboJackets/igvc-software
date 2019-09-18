#ifndef SRC_POINTCLOUD_FILTER_H
#define SRC_POINTCLOUD_FILTER_H

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
  PointcloudFilterConfig config_;
};
}  // namespace pointcloud_filter

#endif  // SRC_POINTCLOUD_FILTER_H
