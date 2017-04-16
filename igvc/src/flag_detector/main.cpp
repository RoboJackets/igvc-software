#include <ros/ros.h>
#include "flagdetector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flagdetector");

  ros::NodeHandle nh;

  FlagDetector det{ nh };

  ros::spin();

  return 0;
}
