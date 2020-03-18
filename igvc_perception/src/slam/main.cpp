#include <ros/ros.h>
#include <slam/slam.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam");
  Slam slam_node;
  ros::spin();
}