#include <ros/publisher.h>
#include <ros/ros.h>

#include "scan_to_pointcloud.hpp"

ros::NodeHandle nh;
ros::NodeHandle pNh("~");

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_to_pointcloud");

  ros::Subscriber scan_sub = ScanCallback::scanSub(nh, pNh);

  ros::spin(); 
}