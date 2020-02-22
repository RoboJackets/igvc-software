#include "traversability_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(traversability_layer::TraversabilityLayer, costmap_2d::Layer)

namespace traversability_layer
{
TraversabilityLayer::TraversabilityLayer()
  : GridmapLayer({ logodds_layer, probability_layer }), private_nh_("~"), config_(private_nh_)
{
  initGridmap();
  initPubSub();
}

void TraversabilityLayer::initGridmap()
{
  map_.setFrameId(config_.map.frame_id);
  grid_map::Length dimensions{ config_.map.length_x, config_.map.length_y };
  map_.setGeometry(dimensions, config_.map.resolution);
  layer_ = &map_.get(logodds_layer);
  (*layer_).setZero();
}

void TraversabilityLayer::initPubSub()
{
  traversability_sub_ =
      private_nh_.subscribe("/traversability/gridmap", 1, &TraversabilityLayer::traversabilityMapCallback, this);
  costmap_pub_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("/traversability/costmap", 1);
}

void TraversabilityLayer::onInitialize()
{
  GridmapLayer::onInitialize();
}

void TraversabilityLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
}

void TraversabilityLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                       double *max_x, double *max_y)
{
  GridmapLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  if (rolling_window_)
  {
    costmap_2d_.updateOrigin(robot_x - costmap_2d_.getSizeInMetersX() / 2,
                             robot_y - costmap_2d_.getSizeInMetersY() / 2);
  }
}

void TraversabilityLayer::traversabilityMapCallback(grid_map_msgs::GridMap traversability_map)
{
}
}  // namespace traversability_layer
