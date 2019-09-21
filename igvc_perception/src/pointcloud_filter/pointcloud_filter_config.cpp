#include <parameter_assertions/assertions.h>
#include <pointcloud_filter/pointcloud_filter_config.h>

namespace pointcloud_filter
{
PointcloudFilterConfig::PointcloudFilterConfig(const ros::NodeHandle &nh)
{
  assertions::getParam(nh, "topic/input", topic_input);
  assertions::getParam(nh, "topic/transformed", topic_transformed);
  assertions::getParam(nh, "topic/occupied", topic_occupied);
  assertions::getParam(nh, "topic/free", topic_free);
}
}  // namespace pointcloud_filter
