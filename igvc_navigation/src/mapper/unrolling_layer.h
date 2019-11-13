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
  /**
   * Initializes the ROS publishers and subscribers
   */
  void initPubSub();

  /**
   * Initializes the translation array to translate from 0-100 (nav_msgs::OccupancyGrid) to 0-255 (costmap_2d)
   */
  void initTranslator();

  /**
   * Callback for a OccupancyGridConstPtr message
   * @param map
   */
  void incomingMap(const nav_msgs::OccupancyGridConstPtr& map);

  /**
   * Callback for a OccupancyGridUpdateConstPtr message
   * @param map
   */
  void incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& map);

  /**
   * Updates min_map_x_, max_map_x_ etc. with the passed in start and end coordinates in cells
   * @param start_x
   * @param start_y
   * @param end_x
   * @param end_y
   */
  void touch(size_t start_x, size_t start_y, size_t end_x, size_t end_y);

  struct UpdateMapMetadata
  {
    int map_idx_x;
    int map_idx_y;
    size_t start_idx_x;
    size_t start_idx_y;
    size_t end_idx_x;
    size_t end_idx_y;
    size_t length_x;
    size_t length_y;
  };

  /**
   * Calculate the coordinate transform metadata for use with updateMap
   * @param length_x length in x of the update map in cells
   * @param length_y length in y of the update map in cells
   * @param origin_x x-coord of the update map origin in the global map in m
   * @param origin_y y-coord of the update map origin in the global map in m
   * @return UpdateMapMetadata containing coordinate transform information
   */
  [[nodiscard]] UpdateMapMetadata calculateMapMetadata(size_t length_x, size_t length_y, double origin_x,
                                                       double origin_y) const;

  /**
   * Updates the map using the passed in map and metadata
   * @param map map to use for update
   * @param metadata metadata containing coordinate transform information from update map to global map
   */
  void updateMap(const std::vector<int8_t>& map, const UpdateMapMetadata& metadata);

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
