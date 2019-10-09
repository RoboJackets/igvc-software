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

  double max_distance;

  double miss_exponential_coeff;
  double miss_angle_exponential_coeff;
  double miss;
  double hit_exponential_coeff;
  double hit;

  struct
  {
    std::string raw_image_ns;
    std::string raw_image;
    std::string segmented_image_ns;
    std::string segmented_image;
  } topics;

  struct
  {
    std::string line_topic;
    std::string nonline_topic;
  } debug;
};
}  // namespace line_layer

#endif  // SRC_CAMERA_CONFIG_H
