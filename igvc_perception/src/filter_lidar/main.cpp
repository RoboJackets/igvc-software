#include <parameter_assertions/assertions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>

ros::Publisher _pointcloud_pub;

double x_offset, y_offset, y_size, x_size, max_dist, back_buffer;

void point_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> result;
  double old_size = x_size;
  for (auto it = msg->points.begin(); it != msg->points.end(); it++)
  {
    if (it->x < 0)
    {
      x_size += back_buffer;
    }
    if (!(abs(it->x + x_offset) < x_size && abs(it->y + y_offset) < y_size) &&
        sqrt(pow(it->x, 2) + pow(it->y, 2)) < max_dist)
    {
      result.push_back(*it);
    }
    x_size = old_size;
  }
  result.header.frame_id = msg->header.frame_id;
  result.header.stamp = msg->header.stamp;
  _pointcloud_pub.publish(result);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter_lidar");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  assertions::getParam(pNh, "x_offset", x_offset);
  assertions::getParam(pNh, "y_offset", y_offset);
  assertions::getParam(pNh, "x_size", x_size);
  assertions::getParam(pNh, "y_size", y_size);
  assertions::getParam(pNh, "max_dist", max_dist);
  assertions::getParam(pNh, "back_buffer", back_buffer);

  _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/scan/pointcloud", 1);

  ros::Subscriber scan_sub = nh.subscribe("/pc2", 1, point_cloud_callback);

  ros::spin();
}
