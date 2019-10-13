#ifndef SRC_LIDAR_LAYER_H
#define SRC_LIDAR_LAYER_H

#include <unordered_set>

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <pcl_ros/point_cloud.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include "eigen_hash.h"
#include "gridmap_layer.h"
#include "lidar_layer_config.h"

namespace lidar_layer
{
class LidarLayer : public gridmap_layer::GridmapLayer
{
public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  LidarLayer();

  void onInitialize() override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

private:
  static constexpr auto logodds_layer = "logodds";
  static constexpr auto probability_layer = "probability";
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  grid_map::Matrix* layer_{};
  LidarLayerConfig config_;

  ros::Subscriber occupied_sub_;
  ros::Subscriber free_sub_;
  ros::Publisher gridmap_pub_;

  std::unordered_set<grid_map::Index> last_occupied_cells_{};

  costmap_2d::Costmap2D costmap_2d_{};

  void initGridmap();
  void initPubSub();

  void occupiedCallback(const sensor_msgs::PointCloud2ConstPtr& occupied_pc);
  void freeCallback(const sensor_msgs::PointCloud2ConstPtr& free_pc);

  void insertScan(const PointCloud& pointcloud, const geometry_msgs::TransformStamped& lidar_transform);

  void insertFreeSpace(const PointCloud& pointcloud, const geometry_msgs::TransformStamped& lidar_transform);

  /**
   * Returns a transformed pointcloud, as well as a transform to base_footprint
   * @param pc pointcloud to be transformed
   * @return a std::pair of the transformed pointcloud and the transform to base_footprint
   */
  [[nodiscard]] std::pair<PointCloud, geometry_msgs::TransformStamped>
  getCloudAndTransform(const sensor_msgs::PointCloud2ConstPtr& pc);

  void updateMapTimestamp(const ros::Time& stamp);

  void updateProbabilityLayer();
  void transferToCostmap();

  void debugPublishMap();

  inline void markScanMiss(const grid_map::Index& index)
  {
    (*layer_)(index[0], index[1]) =
        std::max((*layer_)(index[0], index[1]) + config_.lidar.scan_miss, config_.map.min_occupancy);
  }

  void markScanHit(const grid_map::Index& index, const grid_map::Position& point, const grid_map::Position& lidar_pos);

  inline void markFreeMiss(const grid_map::Index& index)
  {
    (*layer_)(index[0], index[1]) =
        std::max((*layer_)(index[0], index[1]) + config_.lidar.free_miss, config_.map.min_occupancy);
  }
};
}  // namespace lidar_layer

#endif  // SRC_LIDAR_LAYER_H
