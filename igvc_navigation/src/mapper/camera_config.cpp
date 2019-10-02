#include "camera_config.h"
#include <parameter_assertions/assertions.h>

namespace line_layer
{
CameraConfig::CameraConfig(const ros::NodeHandle& parent_nh, const std::string& base_topic) : base_topic{ base_topic }
{
  ros::NodeHandle nh{ parent_nh, base_topic };

  assertions::getParam(nh, "topics/raw_image_ns", topics.raw_image_ns);
  assertions::getParam(nh, "topics/raw_image", topics.raw_image);
  assertions::getParam(nh, "topics/segmented_image_ns", topics.segmented_image_ns);
  assertions::getParam(nh, "topics/segmented_image", topics.segmented_image);
}
}  // namespace line_layer
