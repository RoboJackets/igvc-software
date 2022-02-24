#ifndef SEGMENTED_CAMERA_H
#define SEGMENTED_CAMERA_H

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h> //is this needed?
#include <parameter_assertions/assertions.h>
#include <ros/ros.h>

class SegmentedCameraInfoPublisher
{
public:
  SegmentedCameraInfoPublisher();

private:
  ros::NodeHandle nh;
  ros::NodeHandle pNh;
  // Output size for the transform
  double output_width;
  double output_height;
  // map of camera name to line and barrel publishers
  std::map<std::string, ros::Publisher> g_pubs;
  std::vector<ros::Subscriber> subs;
  void ScaleCameraInfo(const sensor_msgs::CameraInfoConstPtr& camera_info, double width, double height,
                       std::string camera_name);
  void spinnerUpdate();
};

#endif  // SEGMENTED_CAMERA_H