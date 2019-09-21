#include <pointcloud_filter/pointcloud_filter.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_filter_node");

  pointcloud_filter::PointcloudFilter pointcloud_filter{};

  ros::spin();
}
