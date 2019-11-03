#include "unrolling_layer.h"
#include <nav_msgs/OccupancyGrid.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(unrolling_layer::UnrollingLayer, costmap_2d::Layer)

namespace unrolling_layer
{
UnrollingLayer::UnrollingLayer() : private_nh_{ "~" }, config_{ private_nh_ }
{
  initPubSub();
  initTranslator();
}

void UnrollingLayer::onInitialize()
{
  current_ = true;
  matchSize();
}

void UnrollingLayer::initTranslator()
{
  constexpr int8_t free_space_msg_cost = 0;
  constexpr int8_t inflated_msg_cost = 99;
  constexpr int8_t lethal_msg_cost = 100;
  constexpr int8_t unknown_msg_cost = -1;

  translator[free_space_msg_cost] = costmap_2d::FREE_SPACE;
  translator[inflated_msg_cost] = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  translator[lethal_msg_cost] = costmap_2d::LETHAL_OBSTACLE;
  translator[unknown_msg_cost] = costmap_2d::NO_INFORMATION;
}

void UnrollingLayer::initPubSub()
{
  map_update_sub_ = nh_.subscribe(config_.topic, 1, &UnrollingLayer::incomingUpdate, this);
  costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/mapper/fuck", 1);
}

void UnrollingLayer::incomingUpdate(const nav_msgs::OccupancyGridConstPtr& map)
{
  // We need to translate indices from update to our costmap
  const uint32_t length_x = map->info.width;
  const uint32_t length_y = map->info.height;

  const auto resolution = static_cast<double>(map->info.resolution);
  const auto origin_x = static_cast<double>(map->info.origin.position.x);
  const auto origin_y = static_cast<double>(map->info.origin.position.y);

  // Calculate where map's origin is on our map
  const int map_idx_x = (origin_x - getOriginX()) / resolution;
  const int map_idx_y = (origin_y - getOriginY()) / resolution;

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

  for (size_t y = start_idx_y; y < end_idx_y; y++)
  {
    size_t our_y = map_idx_y + y;
    size_t map_index_base = y * length_x;
    size_t our_index_base = our_y * getSizeInCellsX();
    for (size_t x = start_idx_x; x < end_idx_x; x++)
    {
      size_t our_x = map_idx_x + x;
      size_t map_index = map_index_base + x;
      size_t our_index = our_index_base + our_x;

      costmap_[our_index] = translator[map->data[map_index]];
    }
  }
  publishCostmap();
}

void UnrollingLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                  double* max_x, double* max_y)
{
  double min_wx;
  double min_wy;
  double max_wx;
  double max_wy;

  //  mapToWorld(min_x_, min_y_, wx, wy);
  //  *min_x = std::min(wx, *min_x);
  //  *min_y = std::min(wy, *min_y);
  mapToWorld(0, 0, min_wx, min_wy);
  *min_x = std::min(min_wx, *min_x);
  *min_y = std::min(min_wy, *min_y);

  //  mapToWorld(max_x_, max_y_, wx, wy);
  //  *max_x = std::max(wx, *max_x);
  //  *max_y = std::max(wy, *max_y);
  mapToWorld(getSizeInCellsX() - 1, getSizeInCellsY() - 1, max_wx, max_wy);
  *max_x = std::max(max_wx, *max_x);
  *max_y = std::max(max_wy, *max_y);
}

void UnrollingLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  uint8_t* master_array = master_grid.getCharMap();
  uint8_t* line_array = getCharMap();
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

void UnrollingLayer::initCostTranslationTable()
{
  constexpr int8_t free_space_msg_cost = 0;
  constexpr int8_t inflated_msg_cost = 99;
  constexpr int8_t lethal_msg_cost = 100;
  constexpr int8_t unknown_msg_cost = -1;

  cost_translation_table_.resize(std::numeric_limits<uint8_t>::max() + 1);

  cost_translation_table_[costmap_2d::FREE_SPACE] = free_space_msg_cost;                 // NO obstacle
  cost_translation_table_[costmap_2d::INSCRIBED_INFLATED_OBSTACLE] = inflated_msg_cost;  // INSCRIBED obstacle
  cost_translation_table_[costmap_2d::LETHAL_OBSTACLE] = lethal_msg_cost;                // LETHAL obstacle
  cost_translation_table_[costmap_2d::NO_INFORMATION] = unknown_msg_cost;                // UNKNOWN

  // regular cost values scale the range 1 to 252 (inclusive) to fit
  // into 1 to 98 (inclusive).
  for (int i = 1; i <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1; i++)
  {
    // NOLINTNEXTLINE
    cost_translation_table_[i] = static_cast<uint8_t>(1 + (97 * (i - 1)) / 251);
  }
}

void UnrollingLayer::publishCostmap()
{
  if (cost_translation_table_.empty())
  {
    initCostTranslationTable();
  }

  nav_msgs::OccupancyGridPtr msg = boost::make_shared<nav_msgs::OccupancyGrid>();

  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(getMutex()));
  double resolution = getResolution();

  msg->header.frame_id = "odom";
  msg->header.stamp = ros::Time::now();
  msg->info.resolution = resolution;

  msg->info.width = getSizeInCellsX();
  msg->info.height = getSizeInCellsY();

  double wx, wy;
  mapToWorld(0, 0, wx, wy);
  msg->info.origin.position.x = wx - resolution / 2;
  msg->info.origin.position.y = wy - resolution / 2;
  msg->info.origin.position.z = 0.0;
  msg->info.origin.orientation.w = 1.0;

  msg->data.resize(msg->info.width * msg->info.height);

  unsigned char* data = getCharMap();
  for (size_t i = 0; i < msg->data.size(); i++)
  {
    msg->data[i] = cost_translation_table_[data[i]];
  }

  costmap_pub_.publish(msg);
}

}  // namespace unrolling_layer
