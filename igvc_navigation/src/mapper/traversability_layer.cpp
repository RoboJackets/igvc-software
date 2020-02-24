#include "traversability_layer.h"
#include <pluginlib/class_list_macros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <mapper/probability_utils.h>

PLUGINLIB_EXPORT_CLASS(traversability_layer::TraversabilityLayer, costmap_2d::Layer)

namespace traversability_layer
{
TraversabilityLayer::TraversabilityLayer()
  : GridmapLayer({ "logodds", "probability" }), private_nh_("~"), config_(private_nh_)
{
  initGridmap();
  initPubSub();
}

void TraversabilityLayer::initGridmap()
{
  map_.setFrameId(config_.map.frame_id);
  grid_map::Length dimensions{ config_.map.length_x, config_.map.length_y };
  map_.setGeometry(dimensions, config_.map.resolution);
  map_.get("logodds").setZero();
}

void TraversabilityLayer::initPubSub()
{
  slope_sub_ = private_nh_.subscribe("/slope/gridmap", 1, &TraversabilityLayer::slopeMapCallback, this);
  costmap_pub_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("/slope/costmap", 1);
}

void TraversabilityLayer::onInitialize()
{
  GridmapLayer::onInitialize();
  matchCostmapDims(*layered_costmap_->getCostmap());
}

void TraversabilityLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  matchCostmapDims(master_grid);
  transferToCostmap();
  resetDirty();

  uchar *master_array = master_grid.getCharMap();
  uchar *line_array = costmap_2d_.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      unsigned char old_cost = master_array[it];
      if (old_cost == costmap_2d::NO_INFORMATION || old_cost < line_array[it])
        master_array[it] = line_array[it];
      it++;
    }
  }
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

void TraversabilityLayer::slopeMapCallback(const grid_map_msgs::GridMap &slope_map_msg)
{
  grid_map::GridMap slope_map;
  grid_map::GridMapRosConverter::fromMessage(slope_map_msg, slope_map);

  for (grid_map::GridMapIterator it(slope_map); !it.isPastEnd(); ++it)
  {
    grid_map::Position pos;
    slope_map.getPosition((*it), pos);
    if (map_.isInside(pos))
    {
      touch(*it);
      float slope = slope_map.get("slope")((*it)[0], (*it)[1]);
      float *logodd = &map_.atPosition("logodds", pos);
      if (slope > config_.map.slope_threshold)
      {
        *logodd = std::min(*logodd - config_.logodd_increment, config_.map.min_occupancy);
      }
      else
      {
        *logodd = std::min(*logodd + config_.logodd_increment, config_.map.min_occupancy);
      }
    }
  }
}

void TraversabilityLayer::matchCostmapDims(const costmap_2d::Costmap2D &master_grid)
{
  unsigned int cells_x = master_grid.getSizeInCellsX();
  unsigned int cells_y = master_grid.getSizeInCellsY();
  double resolution = master_grid.getResolution();
  bool different_dims = costmap_2d_.getSizeInCellsX() != cells_x || costmap_2d_.getSizeInCellsY() != cells_y ||
                        costmap_2d_.getResolution() != resolution;

  double origin_x = master_grid.getOriginX();
  double origin_y = master_grid.getOriginY();

  if (different_dims)
  {
    costmap_2d_.resizeMap(cells_x, cells_y, resolution, origin_x, origin_y);
  }
  costmap_2d_.updateOrigin(origin_x, origin_y);
}

void TraversabilityLayer::transferToCostmap()
{
  size_t num_cells = map_.getSize().prod();

  uchar *char_map = costmap_2d_.getCharMap();

  auto optional_it = getDirtyIterator();

  if (!optional_it)
  {
    return;
  }

  for (auto it = *optional_it; !it.isPastEnd(); ++it)
  {
    const auto &log_odds = map_.get("logodds")((*it)[0], (*it)[1]);
    float probability = probability_utils::fromLogOdds(log_odds);
    size_t linear_index = grid_map::getLinearIndexFromIndex(*it, map_.getSize(), false);

    if (probability > config_.map.occupied_threshold)
    {
      char_map[num_cells - linear_index - 1] = costmap_2d::LETHAL_OBSTACLE;
    }
    else
    {
      char_map[num_cells - linear_index - 1] = costmap_2d::FREE_SPACE;
    }
  }
}
}  // namespace traversability_layer
