#ifndef SRC_LINE_LAYER_H
#define SRC_LINE_LAYER_H

#include <unordered_set>

#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <pcl_ros/point_cloud.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include "eigen_hash.h"
#include "line_layer_config.h"

namespace line_layer
{
class LineLayer : public costmap_2d::Layer
{
public:
  using RawSegmentedSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                                     sensor_msgs::Image, sensor_msgs::CameraInfo>;
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

  LineLayer();

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

  struct ProjectionResult
  {
    PointCloud line;
    PointCloud nonline;
  };

  struct IndexDistPair
  {
    grid_map::Index index;
    double distance;
  };

  struct index_dist_pair_hash
  {
    std::size_t operator()(const IndexDistPair& pair) const
    {
      return std::hash<grid_map::Index>()(pair.index);
    }
  };

  struct index_dist_pair_equal
  {
    bool operator()(const IndexDistPair& lhs, const IndexDistPair& rhs) const
    {
      return std::equal_to<grid_map::Index>()(lhs.index, rhs.index);
    }
  };

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

  image_geometry::PinholeCameraModel pinhole_model_;

  bool model_initialized_ = false;

  ros::Publisher gridmap_pub_;
  ros::Publisher debug_line_pub_;
  ros::Publisher debug_nonline_pub_;

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

  void ensurePinholeModelInitialized(const sensor_msgs::CameraInfo& segmented_info);
  geometry_msgs::TransformStamped getTransformToCamera(const std::string& frame, const ros::Time& stamp) const;
  cv::Mat convertToMat(const sensor_msgs::ImageConstPtr& image) const;

  ProjectionResult projectImage(const cv::Mat& segmented_mat,
                                const geometry_msgs::TransformStamped& camera_to_odom) const;

  void updateProbabilityLayer();
  void transferToCostmap();
  void debugPublishMap();

  void insertProjectedPointclouds(const PointCloud& line, const PointCloud& nonline,
                                  const geometry_msgs::Transform& camera_pos);

  void markEmpty(const grid_map::Index& index, double distance);
  void markHit(const grid_map::Index& index, double distance);

  void boundRadius(PointCloud& pc, const geometry_msgs::Transform& camera_pos) const;
};
}  // namespace line_layer

#endif  // SRC_LINE_LAYER_H
