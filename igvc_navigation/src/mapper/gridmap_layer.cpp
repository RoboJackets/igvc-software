#include "gridmap_layer.h"

namespace gridmap_layer
{
GridmapLayer::GridmapLayer(const std::vector<std::string>& layers) : map_{ layers }
{
  resetDirty();
}

GridmapLayer::GridmapLayer() : map_{}
{
  resetDirty();
}

void GridmapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
  grid_map::Position min_pos;
  grid_map::Position max_pos;
  // min_idx is max_pos since indexes increases going down and right
  map_.getPosition(dirty_min_idx_, max_pos);
  map_.getPosition(dirty_max_idx_, min_pos);

  *min_x = std::min(*min_x, min_pos[0]);
  *max_x = std::max(*max_x, max_pos[0]);

  *min_y = std::min(*min_y, min_pos[1]);
  *max_y = std::max(*max_y, max_pos[1]);
}

void GridmapLayer::touch(const grid_map::Index& index)
{
  dirty_min_idx_[0] = std::min(dirty_min_idx_[0], index[0]);
  dirty_max_idx_[0] = std::max(dirty_max_idx_[0], index[0]);

  dirty_min_idx_[1] = std::min(dirty_min_idx_[1], index[1]);
  dirty_max_idx_[1] = std::max(dirty_max_idx_[1], index[1]);
}

void GridmapLayer::resetDirty()
{
  dirty_min_idx_ = { std::numeric_limits<int>::max(), std::numeric_limits<int>::max() };
  dirty_max_idx_ = { std::numeric_limits<int>::min(), std::numeric_limits<int>::min() };
}

std::optional<grid_map::SubmapIterator> GridmapLayer::getDirtyIterator() const
{
  if (dirty_min_idx_[0] == std::numeric_limits<int>::max())
    return std::nullopt;

  const grid_map::Index& start_index = dirty_min_idx_;
  const int size_x = dirty_max_idx_[0] - dirty_min_idx_[0] + 1;
  const int size_y = dirty_max_idx_[1] - dirty_min_idx_[1] + 1;
  const grid_map::Index buffer_size{ size_x, size_y };

  return grid_map::SubmapIterator{ map_, start_index, buffer_size };
}
}  // namespace gridmap_layer
