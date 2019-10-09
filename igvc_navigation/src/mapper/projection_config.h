#ifndef SRC_PROJECTION_CONFIG_H
#define SRC_PROJECTION_CONFIG_H

#include <ros/ros.h>

namespace line_layer
{
class ProjectionConfig
{
 public:
  ProjectionConfig(const ros::NodeHandle& parent_nh, double resolution);

  double length_x;
  double length_y;
  int size_x;
  int size_y;

  int line_closing_kernel_size;
  int freespace_closing_kernel_size;
};
}  // namespace line_layer

#endif //SRC_PROJECTION_CONFIG_H
