#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>

ros::Publisher _pointcloud_pub;

double x_offset, y_offset, y_size, x_size, max_dist;

void point_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> result;
  for (auto it = msg->points.begin(); it != msg->points.end(); it++)
  {
    if (!(abs(it->x + x_offset) < x_size && abs(it->y + y_offset) < y_size) &&
        sqrt(pow(it->x, 2) + pow(it->y, 2)) < max_dist)
    {
      result.push_back(*it);
    }
  }
  result.header.frame_id = "/lidar";
  _pointcloud_pub.publish(result);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter_lidar");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  pNh.getParam("x_offset", x_offset);
  pNh.getParam("y_offset", y_offset);
  pNh.getParam("x_size", x_size);
  pNh.getParam("y_size", y_size);
  pNh.getParam("max_dist", max_dist);

  _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/scan/pointcloud", 1);

  ros::Subscriber scan_sub = nh.subscribe("/pc2", 1, point_cloud_callback);

  ros::spin();
}
