#ifndef SRC_TRAVERSABILITY_LAYER_H
#define SRC_TRAVERSABILITY_LAYER_H

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include "gridmap_layer.h"
#include "traversability_layer_config.h"

namespace traversability_layer
{
class TraversabilityLayer : public gridmap_layer::GridmapLayer
{
public:
  TraversabilityLayer();

  void onInitialize() override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;

private:
  ros::NodeHandle private_nh_;
  TraversabilityLayerConfig config_;

  ros::Subscriber slope_sub_;
  ros::Publisher costmap_pub_;

  costmap_2d::Costmap2D costmap_2d_{};

  void initGridmap();
  void initPubSub();

  void slopeMapCallback(const grid_map_msgs::GridMap& slope_map_msg);

  void matchCostmapDims(const costmap_2d::Costmap2D& master_grid);

  void transferToCostmap();

  void updateStaticWindow();
  void updateRollingWindow();

  void publishCostMap();
};
}  // namespace traversability_layer

#endif  // SRC_TRAVERSABILITY_LAYER_H
