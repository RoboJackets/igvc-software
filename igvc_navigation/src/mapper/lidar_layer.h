#ifndef SRC_LIDAR_LAYER_H
#define SRC_LIDAR_LAYER_H

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <grid_map_ros/grid_map_ros.hpp>

namespace lidar_layer
{
class LidarLayer : public costmap_2d::Layer
{
public:
  LidarLayer();

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
private:
  ros::NodeHandle private_nh_;
  grid_map::GridMap map_{ { "lidar" } };

  void initializeGridmap();
};
}  // namespace lidar_layer

#endif  // SRC_LIDAR_LAYER_H
