#include "unrolling_layer.h"
#include <nav_msgs/OccupancyGrid.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(unrolling_layer::UnrollingLayer, costmap_2d::Layer)

namespace unrolling_layer
{
UnrollingLayer::UnrollingLayer() : private_nh_{ "~" }, config_{ private_nh_ }
{
}

void UnrollingLayer::onInitialize()
{
  matchSize();
  initTranslator();
  initPubSub();

  bool track_unknown = nh_.param("track_unknown_space", false);

  default_value_ = track_unknown ? costmap_2d::NO_INFORMATION : costmap_2d::FREE_SPACE;

  current_ = true;
}

void UnrollingLayer::initTranslator()
{
  constexpr uint8_t free_space_msg_cost = 0;
  constexpr uint8_t inflated_msg_cost = 99;
  constexpr uint8_t lethal_msg_cost = 100;
  constexpr uint8_t unknown_msg_cost = -1;

  translator.fill(costmap_2d::FREE_SPACE);

  translator[free_space_msg_cost] = costmap_2d::FREE_SPACE;
  translator[inflated_msg_cost] = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  translator[lethal_msg_cost] = costmap_2d::LETHAL_OBSTACLE;
  translator[unknown_msg_cost] = costmap_2d::NO_INFORMATION;
}

void UnrollingLayer::initPubSub()
{
  if (map_update_sub_.getTopic() != ros::names::resolve(config_.topic))
  {
    map_sub_ = nh_.subscribe(config_.topic, 1, &UnrollingLayer::incomingMap, this);
    map_update_sub_ = nh_.subscribe(config_.topic + "_updates", 1, &UnrollingLayer::incomingUpdate, this);
  }
}

void UnrollingLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& map)
{
  current_metadata_ = map->info;

  // We need to translate indices from update to our costmap
  const uint32_t length_x = map->info.width;
  const uint32_t length_y = map->info.height;

  const auto resolution = static_cast<double>(map->info.resolution);
  const auto origin_x = static_cast<double>(map->info.origin.position.x);
  const auto origin_y = static_cast<double>(map->info.origin.position.y);

  // Calculate where map's origin is on our map
  const int map_idx_x = static_cast<int>(std::round((origin_x - getOriginX()) / resolution));
  const int map_idx_y = static_cast<int>(std::round((origin_y - getOriginY()) / resolution));

  size_t start_idx_x = 0;
  size_t start_idx_y = 0;
  // If map_idx is negative, then calculate the offset we need to start iterating map at
  // so that we don't go off the map
  if (map_idx_x < 0)
  {
    start_idx_x = -map_idx_x;
  }
  if (map_idx_y < 0)
  {
    start_idx_y = -map_idx_y;
  }

  // Calculate where map's end is on our map
  const size_t idx_end_x = map_idx_x + length_x;
  const size_t idx_end_y = map_idx_y + length_y;

  const auto last_idx_x = getSizeInCellsX();
  const auto last_idx_y = getSizeInCellsY();

  // If map's end is larger than our largest, then last index to iterate is our largest - their origin in our map
  size_t end_idx_x = length_x;
  size_t end_idx_y = length_y;

  if (idx_end_x > last_idx_x)
  {
    end_idx_x = last_idx_x - map_idx_x;
  }
  if (idx_end_y > last_idx_y)
  {
    end_idx_y = last_idx_y - map_idx_y;
  }

  UpdateMapMetadata metadata{
    map_idx_x, map_idx_y, start_idx_x, start_idx_y, end_idx_x, end_idx_y, length_x, length_y
  };
  updateMap(map->data, metadata);
}

void UnrollingLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& map)
{
  if (!current_metadata_)
  {
    return;
  }

  // We need to translate indices from update to our costmap
  const uint32_t length_x = map->width;
  const uint32_t length_y = map->height;

  const auto resolution = static_cast<double>(current_metadata_->resolution);
  const auto origin_x = static_cast<double>(current_metadata_->origin.position.x);
  const auto origin_y = static_cast<double>(current_metadata_->origin.position.y);

  const auto update_origin_x = origin_x + (map->x * resolution);
  const auto update_origin_y = origin_y + (map->y * resolution);

  // Calculate where the update map's origin is on our map
  const int map_idx_x = static_cast<int>((update_origin_x - getOriginX()) / resolution);
  const int map_idx_y = static_cast<int>((update_origin_y - getOriginY()) / resolution);

  size_t start_idx_x = 0;
  size_t start_idx_y = 0;
  // If map_idx is negative, then calculate the offset we need to start iterating map at
  // so that we don't go off the map
  if (map_idx_x < 0)
  {
    start_idx_x = -map_idx_x;
  }
  if (map_idx_y < 0)
  {
    start_idx_y = -map_idx_y;
  }

  // Calculate where map's end is on our map
  const size_t our_idx_end_x = map_idx_x + length_x;
  const size_t our_idx_end_y = map_idx_y + length_y;

  const auto last_idx_x = getSizeInCellsX();
  const auto last_idx_y = getSizeInCellsY();

  size_t end_idx_x = length_x;
  size_t end_idx_y = length_y;

  // If map's end is larger than our largest, then last index to iterate is our largest - their origin in our map
  if (our_idx_end_x > last_idx_x)
  {
    end_idx_x = last_idx_x - map_idx_x;
  }
  if (our_idx_end_y > last_idx_y)
  {
    end_idx_y = last_idx_y - map_idx_y;
  }

  UpdateMapMetadata metadata{
    map_idx_x, map_idx_y, start_idx_x, start_idx_y, end_idx_x, end_idx_y, length_x, length_y
  };
  updateMap(map->data, metadata);
}

void UnrollingLayer::updateMap(const std::vector<int8_t>& map, const UpdateMapMetadata& metadata)
{
  touch(metadata.map_idx_x + metadata.start_idx_x, metadata.map_idx_y + metadata.start_idx_y,
        metadata.map_idx_x + metadata.end_idx_x, metadata.map_idx_y + metadata.end_idx_y);
  for (size_t y = metadata.start_idx_y; y < metadata.end_idx_y; y++)
  {
    const size_t our_y = metadata.map_idx_y + y;
    const size_t map_index_base = y * metadata.length_x;
    const size_t our_index_base = our_y * getSizeInCellsX();
    for (size_t x = metadata.start_idx_x; x < metadata.end_idx_x; x++)
    {
      const size_t our_x = metadata.map_idx_x + x;
      const size_t map_index = map_index_base + x;
      const size_t our_index = our_index_base + our_x;

      costmap_[our_index] = translator[static_cast<uint8_t>(map[map_index])];
    }
  }
}

void UnrollingLayer::touch(size_t start_x, size_t start_y, size_t end_x, size_t end_y)
{
  min_map_x_ = std::min(min_map_x_, start_x);
  min_map_y_ = std::min(min_map_y_, start_y);
  max_map_x_ = std::max(max_map_x_, end_x);
  max_map_y_ = std::max(max_map_y_, end_y);
}

void UnrollingLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                  double* max_x, double* max_y)
{
  ROS_INFO_STREAM(">>> updateBounds");
  if (max_map_x_ == 0 && max_map_y_ == 0 && min_map_x_ == std::numeric_limits<size_t>::max() &&
      min_map_y_ == std::numeric_limits<size_t>::max())
  {
    return;
  }

  double min_wx;
  double min_wy;
  double max_wx;
  double max_wy;

  //  mapToWorld(min_x_, min_y_, wx, wy);
  //  *min_x = std::min(wx, *min_x);
  //  *min_y = std::min(wy, *min_y);
  mapToWorld(min_map_x_, min_map_y_, min_wx, min_wy);
  *min_x = std::min(min_wx, *min_x);
  *min_y = std::min(min_wy, *min_y);

  //  mapToWorld(max_x_, max_y_, wx, wy);
  //  *max_x = std::max(wx, *max_x);
  //  *max_y = std::max(wy, *max_y);
  mapToWorld(max_map_x_, max_map_y_, max_wx, max_wy);
  *max_x = std::max(max_wx, *max_x);
  *max_y = std::max(max_wy, *max_y);

  min_map_x_ = std::numeric_limits<size_t>::max();
  min_map_y_ = std::numeric_limits<size_t>::max();
  max_map_x_ = 0;
  max_map_y_ = 0;
  ROS_INFO_STREAM("<< done updateBounds");
}

void UnrollingLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  ROS_INFO_STREAM(">>> updateCosts (" << min_i << ", " << min_j << ") -> (" << max_i << ", " << max_j << ")");
  uint8_t* master_array = master_grid.getCharMap();
  uint8_t* line_array = getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++)
  {
    const unsigned int base = j * span;
    for (int i = min_i; i < max_i; i++)
    {
      const unsigned int it = base + i;
      master_array[it] = line_array[it];
    }
  }
  ROS_INFO_STREAM("<<< done updateCosts");
}

}  // namespace unrolling_layer
