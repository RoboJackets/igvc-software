#include "camera_config.h"
#include <mapper/probability_utils.h>
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

  assertions::getParam(nh, "max_distance", max_distance);

  assertions::getParam(nh, "probabilities/miss", miss);
  miss = probability_utils::toLogOdds(1.0 - miss);  // It's a miss, so 1 - hit
  assertions::getParam(nh, "probabilities/hit", hit);
  hit = probability_utils::toLogOdds(hit);
  assertions::getParam(nh, "probabilities/miss_exponential_coeff", miss_exponential_coeff);
  assertions::getParam(nh, "probabilities/hit_exponential_coeff", hit_exponential_coeff);

  assertions::getParam(nh, "debug/line_topic", debug.line_topic);
  assertions::getParam(nh, "debug/nonline_topic", debug.nonline_topic);
}
}  // namespace line_layer
