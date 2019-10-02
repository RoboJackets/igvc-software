#include "line_layer.h"
#include <pluginlib/class_list_macros.h>
#include "camera_config.h"

PLUGINLIB_EXPORT_CLASS(line_layer::LineLayer, costmap_2d::Layer)

namespace line_layer
{
LineLayer::LineLayer() : private_nh_{ "~" }, map_{ { logodds_layer, probability_layer } }, config_{ private_nh_ }
{
  initGridmap();
  initPubSub();
  costmap_2d_ = { static_cast<unsigned int>(map_.getSize()[0]), static_cast<unsigned int>(map_.getSize()[1]),
                  map_.getResolution(), map_.getPosition()[0], map_.getPosition()[1] };
}

void LineLayer::initGridmap()
{
  // TODO: Configurable start positions
  map_.setFrameId(config_.map.frame_id);
  grid_map::Length dimensions{ config_.map.length_x, config_.map.length_y };
  map_.setGeometry(dimensions, config_.map.resolution);
  layer_ = &map_.get(logodds_layer);
  (*layer_).setZero();

  grid_map::Position top_left;
  map_.getPosition(map_.getStartIndex(), top_left);
}

void LineLayer::initPubSub()
{
  const auto &camera_config = config_.center;
  const auto &base_topic = camera_config.base_topic;
  std::string raw_image_topic = base_topic + camera_config.topics.raw_image_ns + camera_config.topics.raw_image;
  std::string raw_info_topic = base_topic + camera_config.topics.raw_image_ns + "/camera_info";
  std::string segmented_image_topic =
      base_topic + camera_config.topics.segmented_image_ns + camera_config.topics.segmented_image;
  std::string segmented_info_topic = base_topic + camera_config.topics.segmented_image_ns + "/camera_info";

  center_subscribers_ = {
    std::make_unique<ImageSubscriber>(nh_, raw_image_topic, 1),
    std::make_unique<CameraInfoSubscriber>(nh_, raw_info_topic, 1),
    std::make_unique<ImageSubscriber>(nh_, segmented_image_topic, 1),
    std::make_unique<CameraInfoSubscriber>(nh_, segmented_info_topic, 1),
  };

  center_synchronizer_ = std::make_unique<RawSegmentedSynchronizer>(
      *center_subscribers_.raw_image_sub, *center_subscribers_.raw_info_sub, *center_subscribers_.segmented_image_sub,
      *center_subscribers_.segmented_info_sub, 10);
  center_synchronizer_->registerCallback(boost::bind(&LineLayer::imageSyncedCallback, this, _1, _2, _3, _4));
}

void LineLayer::onInitialize()
{
}

void LineLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                             double *max_x, double *max_y)
{
  // TODO: Change this to only the points that have been updated
  *min_x = map_.getPosition()[0] - map_.getLength()[0] / 2;
  *max_x = map_.getPosition()[0] + map_.getLength()[0] / 2;
  *min_y = map_.getPosition()[1] - map_.getLength()[1] / 2;
  *max_y = map_.getPosition()[1] + map_.getLength()[1] / 2;
}

void LineLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int /*min_i*/, int /*min_j*/, int /*max_i*/,
                            int /*max_j*/)
{
  //  updateProbabilityLayer();
  //  transferToCostmap();
  //  debugPublishMap();
  //
  //  assert(costmap_2d_.getSizeInCellsX() == master_grid.getSizeInCellsX());
  //  assert(costmap_2d_.getSizeInCellsY() == master_grid.getSizeInCellsY());
  //
  //  size_t num_cells = costmap_2d_.getSizeInCellsX() * costmap_2d_.getSizeInCellsY();
  //
  //  uchar *master_array = master_grid.getCharMap();
  //  uchar *lidar_array = costmap_2d_.getCharMap();
  //  for (size_t i = 0; i < num_cells; i++)
  //  {
  //    uchar old_cost = master_array[i];
  //    if (old_cost == costmap_2d::NO_INFORMATION || old_cost < lidar_array[i])
  //    {
  //      master_array[i] = lidar_array[i];
  //    }
  //  }
}
void LineLayer::imageSyncedCallback(const sensor_msgs::ImageConstPtr &raw_image,
                                    const sensor_msgs::CameraInfoConstPtr &raw_info,
                                    const sensor_msgs::ImageConstPtr &segmented_image,
                                    const sensor_msgs::CameraInfoConstPtr &segmented_info)
{
  ROS_INFO_STREAM("Got synced callback! raw: (" << raw_info->width << ", " << raw_info->height << "), segmented: "
                                                << segmented_info->width << ", (" << segmented_info->height << ")");
}

}  // namespace line_layer
