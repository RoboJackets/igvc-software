#ifndef SRC_WRAPPERLAYER_H
#define SRC_WRAPPERLAYER_H

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <ros/ros.h>
#include <mutex>

#include "ros_mapper.h"

namespace wrapper_layer
{
class WrapperLayer : public costmap_2d::Layer
{
public:
  WrapperLayer();

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

private:
  ROSMapper mapper_;
};
}  // namespace wrapper_layer

#endif  // SRC_WRAPPERLAYER_H
