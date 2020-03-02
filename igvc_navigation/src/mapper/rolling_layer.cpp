#include "rolling_layer.h"

namespace rolling_layer
{
RollingLayer::RollingLayer() : private_nh_("~"), config_(private_nh_)
{
}

void RollingLayer::onInitialize()
{
  initPubSub();
  current_ = true;
}

void RollingLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                double *max_x, double *max_y)
{

}

void RollingLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
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
}

void RollingLayer::initPubSub()
{
    costmap_sub_ = nh_.subscribe(config_.topic, 1, &RollingLayer::costmapCallback, this);
    costmap_update_sub_ = nh_.subscribe(config_.topic + "_updates", 1, &RollingLayer::costmapUpdateCallback, this);
}

void RollingLayer::costmapCallback(const nav_msgs::OccupancyGridConstPtr &map)
{

}

void RollingLayer::costmapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr &map_update)
{
}
}  // namespace rolling_layer
