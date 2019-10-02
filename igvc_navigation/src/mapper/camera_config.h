#ifndef SRC_CAMERA_CONFIG_H
#define SRC_CAMERA_CONFIG_H

#include <ros/ros.h>
#include <string>

namespace line_layer
{
class CameraConfig
{
public:
  CameraConfig(const ros::NodeHandle& parent_nh, const std::string& base_topic);

  std::string base_topic;

  struct
  {
    std::string raw_image_ns;
    std::string raw_image;
    std::string segmented_image_ns;
    std::string segmented_image;
  } topics;
};
}  // namespace line_layer

#endif  // SRC_CAMERA_CONFIG_H
