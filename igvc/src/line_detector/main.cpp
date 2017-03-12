#include <ros/ros.h>
#include "linedetector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "linedetector");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");
  std::string topic;
  if (!pNh.hasParam("topic"))
    ROS_WARN_STREAM("No topics specified for line detector. No map will be generated.");

  pNh.getParam("topic", topic);

  LineDetector det{ nh, topic };

  ROS_INFO_STREAM("Line Detector started.");

  // Rate is number of refreshes per second
  ros::Rate rate(5.);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
