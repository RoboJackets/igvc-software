#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "project");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/usb_cam_center/line_cloud", 1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  for(float i = 0; i < 10; i++) {
    cloud->push_back(pcl::PointXYZ(i, -1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(i, 1.0, 0.0));
  }

  cloud->header.frame_id = "/base_footprint";

  ros::Rate rate(10);
  while(ros::ok()) {
    ros::spinOnce();
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pub.publish(*cloud);
  }

}
