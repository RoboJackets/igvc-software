#ifndef SRC_UNROLLING_LAYER_H
#define SRC_UNROLLING_LAYER_H

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include "unrolling_layer_config.h"

namespace unrolling_layer
{
class UnrollingLayer : public costmap_2d::CostmapLayer
{
public:
  UnrollingLayer();

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

private:
  void initPubSub();
  void initTranslator();
  void incomingMap(const nav_msgs::OccupancyGridConstPtr& map);
  void incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& map);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber map_update_sub_;
  std::array<uint8_t, std::numeric_limits<char>::max()> translator;

  UnrollingLayerConfig config_;
  std::optional<nav_msgs::MapMetaData> current_metadata_;

  size_t min_map_x_;
  size_t min_map_y_;
  size_t max_map_x_;
  size_t max_map_y_;
};
}  // namespace unrolling_layer

#endif  // SRC_UNROLLING_LAYER_H
