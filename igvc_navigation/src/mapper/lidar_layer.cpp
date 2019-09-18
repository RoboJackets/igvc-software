#include "lidar_layer.h"
namespace lidar_layer
{
LidarLayer::LidarLayer() : private_nh_{ "~" }
{
  initializeGridmap();
}

void LidarLayer::initializeGridmap()
{
  map_.setFrameId();
}

void LidarLayer::onInitialize()
{
  Layer::onInitialize();
}

void LidarLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                              double *max_x, double *max_y)
{
  // TODO: Change this to only the points that have been updated
  map.
}

void LidarLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  Layer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace lidar_layer
