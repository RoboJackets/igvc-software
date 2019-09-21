#include <parameter_assertions/assertions.h>
#include <pointcloud_filter/tf_transform_filter/tf_transform_filter_config.h>

namespace pointcloud_filter
{
TFTransformFilterConfig::TFTransformFilterConfig(const ros::NodeHandle &nh)
{
  ros::NodeHandle child_nh{ nh, "tf_transform_filter" };

  assertions::getParam(child_nh, "target_frame", target_frame);
  assertions::getParam(child_nh, "timeout", timeout);
}
}  // namespace pointcloud_filter
