#include "line_layer.h"
#include <cv_bridge/cv_bridge.h>
#include <mapper/probability_utils.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <unordered_set>
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
  gridmap_pub_ = nh_.advertise<grid_map_msgs::GridMap>(config_.map.debug.map_topic, 1);
  debug_line_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(config_.center.debug.line_topic, 1);
  debug_nonline_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(config_.center.debug.nonline_topic, 1);

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
  updateProbabilityLayer();
  transferToCostmap();
  debugPublishMap();

  assert(costmap_2d_.getSizeInCellsX() == master_grid.getSizeInCellsX());
  assert(costmap_2d_.getSizeInCellsY() == master_grid.getSizeInCellsY());

  size_t num_cells = costmap_2d_.getSizeInCellsX() * costmap_2d_.getSizeInCellsY();

  uchar *master_array = master_grid.getCharMap();
  uchar *line_array = costmap_2d_.getCharMap();
  for (size_t i = 0; i < num_cells; i++)
  {
    uchar old_cost = master_array[i];
    if (old_cost == costmap_2d::NO_INFORMATION || old_cost < line_array[i])
    {
      master_array[i] = line_array[i];
    }
  }
}
void LineLayer::imageSyncedCallback(const sensor_msgs::ImageConstPtr &raw_image,
                                    const sensor_msgs::CameraInfoConstPtr &raw_info,
                                    const sensor_msgs::ImageConstPtr &segmented_image,
                                    const sensor_msgs::CameraInfoConstPtr &segmented_info)
{
  ensurePinholeModelInitialized(*segmented_info);

  geometry_msgs::TransformStamped camera_to_odom =
      getTransformToCamera(raw_image->header.frame_id, raw_image->header.stamp);

  cv::Mat segmented_mat = convertToMat(segmented_image);

  auto [line, nonline] = projectImage(segmented_mat, camera_to_odom);

  boundRadius(line, camera_to_odom.transform);
  boundRadius(nonline, camera_to_odom.transform);

  insertProjectedPointclouds(line, nonline, camera_to_odom.transform);

  line.header.frame_id = "odom";
  line.header.stamp = pcl_conversions::toPCL(raw_image->header.stamp);
  nonline.header.frame_id = "odom";
  nonline.header.stamp = line.header.stamp;

  debug_line_pub_.publish(line);
  debug_nonline_pub_.publish(nonline);

  //  ROS_INFO_STREAM("Got synced callback! raw: (" << raw_info->width << ", " << raw_info->height << "), segmented: "
  //                                                << segmented_info->width << ", (" << segmented_info->height << ")");
}

void LineLayer::ensurePinholeModelInitialized(const sensor_msgs::CameraInfo &segmented_info)
{
  if (!model_initialized_)
  {
    pinhole_model_.fromCameraInfo(segmented_info);
    model_initialized_ = true;
  }
}

geometry_msgs::TransformStamped LineLayer::getTransformToCamera(const std::string &frame, const ros::Time &stamp) const
{
  if (tf_->canTransform("odom", frame, stamp, ros::Duration{ 1 }))
  {
    return tf_->lookupTransform("odom", frame, stamp, ros::Duration{ 1 });
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Failed to find transform from frame 'odom' to frame 'base_footprint' within "
                                  "timeout. Using latest transform...");
    return tf_->lookupTransform("odom", frame, ros::Time{ 0 }, ros::Duration{ 1 });
  }
}

cv::Mat LineLayer::convertToMat(const sensor_msgs::ImageConstPtr &image) const
{
  cv_bridge::CvImageConstPtr cv_bridge_image = cv_bridge::toCvShare(image, "mono8");
  return cv_bridge_image->image;
}

LineLayer::ProjectionResult LineLayer::projectImage(const cv::Mat &segmented_mat,
                                                    const geometry_msgs::TransformStamped &camera_to_odom) const
{
  PointCloud line;
  PointCloud nonline;

  int rows = segmented_mat.rows;
  int cols = segmented_mat.cols;

  Eigen::Quaterniond rotation;
  const auto &translate_vector = camera_to_odom.transform.translation;
  Eigen::Vector3d translation{ translate_vector.x, translate_vector.y, translate_vector.z };
  tf2::convert(camera_to_odom.transform.rotation, rotation);

  for (int i = 0; i < rows; i++)
  {
    const auto *row_ptr = segmented_mat.ptr<uchar>(i);
    for (int j = 0; j < cols; j++)
    {
      // TODO: Mask using barrels
      cv::Point2d pixel_point(j, i);
      cv::Point3d ray = pinhole_model_.projectPixelTo3dRay(pixel_point);
      Eigen::Vector3d eigen_ray{ ray.x, ray.y, ray.z };
      eigen_ray = rotation * eigen_ray;

      double scale = -translation[2] / eigen_ray[2];
      Eigen::Vector3f projected_point = (scale * eigen_ray + translation).cast<float>();

      bool is_line = row_ptr[j] == 255u;
      if (is_line)
      {
        line.points.emplace_back(pcl::PointXYZ(projected_point.x(), projected_point.y(), projected_point.z()));
      }
      else
      {
        nonline.points.emplace_back(pcl::PointXYZ(projected_point.x(), projected_point.y(), projected_point.z()));
      }
    }
  }

  return { line, nonline };
}

void LineLayer::insertProjectedPointclouds(const LineLayer::PointCloud &line, const LineLayer::PointCloud &nonline,
                                           const geometry_msgs::Transform &camera_pos)
{
  std::unordered_set<IndexDistPair, index_dist_pair_hash, index_dist_pair_equal> nonline_indices;
  nonline_indices.reserve(nonline.size());

  for (const auto &point : nonline)
  {
    const grid_map::Position position{ point.x, point.y };
    if (!map_.isInside(position))
    {
      continue;
    }
    const double dx = camera_pos.translation.x - point.x;
    const double dy = camera_pos.translation.y - point.y;
    const double dz = camera_pos.translation.z - point.z;

    const double distance = std::hypot(dx, dy, dz);

    grid_map::Index index;
    map_.getIndex(position, index);
    IndexDistPair pair{ index, distance };
    nonline_indices.emplace(pair);
  }

  for (const auto &point : line)
  {
    const grid_map::Position position{ point.x, point.y };
    if (!map_.isInside(position))
    {
      continue;
    }
    const double dx = camera_pos.translation.x - point.x;
    const double dy = camera_pos.translation.y - point.y;
    const double dz = camera_pos.translation.z - point.z;

    const double distance = std::hypot(dx, dy, dz);

    grid_map::Index index;
    map_.getIndex(position, index);
    nonline_indices.erase({ index, 0.0 });
    markHit(index, distance);
  }

  for (const auto &pair : nonline_indices)
  {
    markEmpty(pair.index, pair.distance);
  }
}

void LineLayer::markEmpty(const grid_map::Index &index, double distance)
{
  const auto coeff = config_.center.miss_exponential_coeff;
  const double probability = std::exp(-coeff * distance) * config_.center.miss;

  (*layer_)(index[0], index[1]) = std::max((*layer_)(index[0], index[1]) + probability, config_.map.min_occupancy);
}

void LineLayer::markHit(const grid_map::Index &index, double distance)
{
  const auto coeff = config_.center.hit_exponential_coeff;
  const double probability = std::exp(-coeff * distance) * config_.center.hit;

  (*layer_)(index[0], index[1]) = std::min((*layer_)(index[0], index[1]) + probability, config_.map.max_occupancy);
}

void LineLayer::updateProbabilityLayer()
{
  map_.get(probability_layer) = layer_->unaryExpr(&probability_utils::fromLogOdds<float>);
}

void LineLayer::transferToCostmap()
{
  ROS_ASSERT(map_.getSize()[0] == static_cast<int>(costmap_2d_.getSizeInCellsX()));
  ROS_ASSERT(map_.getSize()[1] == static_cast<int>(costmap_2d_.getSizeInCellsY()));
  ROS_ASSERT(map_.getResolution() == costmap_2d_.getResolution());

  size_t num_cells = map_.getSize().prod();

  uchar *char_map = costmap_2d_.getCharMap();

  const grid_map::Matrix &prob_layer = map_.get(probability_layer);

  for (grid_map::GridMapIterator it{ map_ }; !it.isPastEnd(); ++it)
  {
    float probability = prob_layer((*it)(0), (*it)(1));

    uchar costmap_value;
    if (probability > config_.map.occupied_threshold)
    {
      costmap_value = costmap_2d::LETHAL_OBSTACLE;
    }
    else
    {
      costmap_value = costmap_2d::FREE_SPACE;
    }
    size_t index = grid_map::getLinearIndexFromIndex(it.getUnwrappedIndex(), map_.getSize(), false);
    // Reverse cell order because of different conventions between occupancy grid and grid map.
    char_map[num_cells - index - 1] = costmap_value;
  }
}

void LineLayer::debugPublishMap()
{
  map_.setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  gridmap_pub_.publish(message);
}

void LineLayer::boundRadius(PointCloud &pc, const geometry_msgs::Transform &camera_pos) const
{
  PointCloud bounded_pc;
  bounded_pc.points.reserve(pc.size());
  for (const auto &point : pc)
  {
    double dx = point.x - camera_pos.translation.x;
    double dy = point.y - camera_pos.translation.y;
    double dz = point.z - camera_pos.translation.z;
    double distance = std::hypot(dx, dy, dz);
    if (distance <= config_.center.max_distance)
    {
      bounded_pc.points.emplace_back(point);
    }
  }

  pc.points.swap(bounded_pc.points);
}

}  // namespace line_layer
