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
#include "gridmap_layer.h"
#include "line_layer_config.h"

namespace line_layer
{
class LineLayer : public gridmap_layer::GridmapLayer
{
public:
  using RawSegmentedSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                                     sensor_msgs::Image, sensor_msgs::CameraInfo>;
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

  LineLayer();

  void onInitialize() override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;

  struct IndexDistPair
  {
    grid_map::Index index;
    double distance;
  };

private:
  using ImageSubscriber = message_filters::Subscriber<sensor_msgs::Image>;
  using CameraInfoSubscriber = message_filters::Subscriber<sensor_msgs::CameraInfo>;

  static constexpr auto logodds_layer = "logodds";
  static constexpr auto probability_layer = "probability";
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  grid_map::Matrix* layer_{};
  LineLayerConfig config_;
  cv::Mat line_buffer_;       // cv::Mat centered at current position for use as a "buffer" for lines
  cv::Mat freespace_buffer_;  // cv::Mat centered at current position for use as a "buffer" for freespace
  cv::Mat not_lines_;         // not line_buffer_

  std::vector<image_geometry::PinholeCameraModel> pinhole_models_;
  std::vector<std::vector<Eigen::Vector3d>> cached_rays_;

  ros::Publisher gridmap_pub_;
  ros::Publisher costmap_pub_;
  std::vector<int8_t> cost_translation_table_;
  struct DebugPublishers
  {
    ros::Publisher debug_line_pub_;
    ros::Publisher debug_nonline_pub_;
  };
  std::vector<DebugPublishers> debug_publishers_;

  struct CameraSubscribers
  {
    std::unique_ptr<ImageSubscriber> raw_image_sub;
    std::unique_ptr<CameraInfoSubscriber> raw_info_sub;
    std::unique_ptr<ImageSubscriber> segmented_image_sub;
    std::unique_ptr<CameraInfoSubscriber> segmented_info_sub;
  };

  std::vector<CameraSubscribers> camera_subscribers_;

  std::vector<std::unique_ptr<RawSegmentedSynchronizer>> synchronizers_;

  costmap_2d::Costmap2D costmap_2d_{};

  void initGridmap();
  void initPubSub();

  void imageSyncedCallback(const sensor_msgs::ImageConstPtr& raw_image, const sensor_msgs::CameraInfoConstPtr& raw_info,
                           const sensor_msgs::ImageConstPtr& segmented_image,
                           const sensor_msgs::CameraInfoConstPtr& segmented_info, size_t camera_index);

  void ensurePinholeModelInitialized(const sensor_msgs::CameraInfo& segmented_info, size_t camera_index);
  void calculateCachedRays(const sensor_msgs::CameraInfo& info, size_t camera_index);

  geometry_msgs::TransformStamped getTransformToCamera(const std::string& frame, const ros::Time& stamp) const;
  cv::Mat convertToMat(const sensor_msgs::ImageConstPtr& image) const;

  void projectImage(const cv::Mat& segmented_mat, const geometry_msgs::TransformStamped& camera_to_odom,
                    size_t camera_idx, const sensor_msgs::CameraInfo &info);
  void cleanupProjections();
  void insertProjectionsIntoMap(const geometry_msgs::TransformStamped& camera_to_odom, const CameraConfig& config);
  void matchCostmapDims(const costmap_2d::Costmap2D& master_grid);

  grid_map::Index calculateBufferIndex(const Eigen::Vector3f& point, const grid_map::Index& camera_index) const;

  void debugPublishPC(ros::Publisher& pub, const cv::Mat& mat, geometry_msgs::TransformStamped& camera_to_odom);

  void updateProbabilityLayer();
  void transferToCostmap();
  void updateRollingWindow();
  void updateStaticWindow();

  void debugPublishMap();
  void publishCostmap();
  void initCostTranslationTable();

  void markEmpty(const grid_map::Index& index, double distance, double angle, const CameraConfig& config);
  void markHit(const grid_map::Index& index, double distance, const CameraConfig& config);
  cv::Mat getPerspectiveTransformMat(const sensor_msgs::CameraInfo &info);
  cv::Point2d applyHomography(const cv::Point2d& _point, const cv::Mat& _H);
  void createMap(cv::Size &dstSize, cv::Mat &m_mapX, cv::Mat &m_mapY, cv::Mat &transformationMat_inv);
};
}  // namespace line_layer

#endif  // SRC_LINE_LAYER_H
