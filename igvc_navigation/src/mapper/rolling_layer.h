#ifndef SRC_ROLLING_LAYER_H
#define SRC_ROLLING_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include "rolling_layer_config.h"

namespace rolling_layer
{
class RollingLayer : public costmap_2d::CostmapLayer
{
public:
  RollingLayer();
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

private:
  void initPubSub();

  void costmapCallback(const nav_msgs::OccupancyGridConstPtr& map);
  void costmapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& map_update);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber costmap_sub_;

  RollingLayerConfig config_;
};
}  // namespace rolling_layer

#endif  // SRC_ROLLING_LAYER_H
