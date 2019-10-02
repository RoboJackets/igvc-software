#ifndef SRC_LINE_LAYER_H
#define SRC_LINE_LAYER_H

#include <unordered_set>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <pcl_ros/point_cloud.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include "line_layer_config.h"

namespace line_layer
{
class LineLayer : public costmap_2d::Layer
{
public:
  using RawSegmentedSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                                     sensor_msgs::Image, sensor_msgs::CameraInfo>;

  LineLayer();

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

private:
  using ImageSubscriber = message_filters::Subscriber<sensor_msgs::Image>;
  using CameraInfoSubscriber = message_filters::Subscriber<sensor_msgs::CameraInfo>;

  static constexpr auto logodds_layer = "logodds";
  static constexpr auto probability_layer = "probability";
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  grid_map::GridMap map_;
  grid_map::Matrix* layer_{};
  LineLayerConfig config_;

  struct CameraSubscribers
  {
    std::unique_ptr<ImageSubscriber> raw_image_sub;
    std::unique_ptr<CameraInfoSubscriber> raw_info_sub;
    std::unique_ptr<ImageSubscriber> segmented_image_sub;
    std::unique_ptr<CameraInfoSubscriber> segmented_info_sub;
  };

  CameraSubscribers center_subscribers_;

  //  std::unordered_set<grid_map::Index> last_occupied_cells_{};
  std::unique_ptr<RawSegmentedSynchronizer> center_synchronizer_{};

  costmap_2d::Costmap2D costmap_2d_{};

  void initGridmap();
  void initPubSub();

  void imageSyncedCallback(const sensor_msgs::ImageConstPtr& raw_image, const sensor_msgs::CameraInfoConstPtr& raw_info,
                           const sensor_msgs::ImageConstPtr& segmented_image,
                           const sensor_msgs::CameraInfoConstPtr& segmented_info);
};
}  // namespace line_layer

#endif  // SRC_LINE_LAYER_H
