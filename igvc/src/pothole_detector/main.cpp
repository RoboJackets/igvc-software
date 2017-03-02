#include "potholedetector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "potholedetector");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");
  std::string topic;
  if (!pNh.hasParam("topic"))
    ROS_WARN_STREAM("No topics specified for pothole detector. No map will be generated.");

  pNh.getParam("topic", topic);

  PotholeDetector det{ nh, topic };

  ROS_INFO_STREAM("Pothole detector started");

  ros::spin();

  return 0;
}
