#ifndef SRC_GRIDMAP_LAYER_H
#define SRC_GRIDMAP_LAYER_H

#include <costmap_2d/layer.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace gridmap_layer
{
class GridmapLayer : public costmap_2d::Layer
{
public:
  GridmapLayer(const std::vector<std::string>& layers);
  GridmapLayer();

  void onInitialize() override = 0;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override = 0;

protected:
  void touch(const grid_map::Index& index);
  void resetDirty();
  std::optional<grid_map::SubmapIterator> getDirtyIterator() const;

  grid_map::Index dirty_min_idx_;
  grid_map::Index dirty_max_idx_;
  grid_map::GridMap map_{};
};
}  // namespace gridmap_layer

#endif  // SRC_GRIDMAP_LAYER_H
