#include "line_layer.h"
#include <angles/angles.h>
#include <cv_bridge/cv_bridge.h>
#include <mapper/probability_utils.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/videoio.hpp>
#include <unordered_set>
#include "camera_config.h"
#include "projection_config.h"

PLUGINLIB_EXPORT_CLASS(line_layer::LineLayer, costmap_2d::Layer)

namespace line_layer
{
LineLayer::LineLayer()
  : private_nh_{ "~" }
  , map_{ { logodds_layer, probability_layer } }
  , config_{ private_nh_ }
  , line_buffer_{ config_.projection.size_x, config_.projection.size_y, CV_8U }
  , freespace_buffer_{ config_.projection.size_x, config_.projection.size_y, CV_8U }
  , not_lines_{ config_.projection.size_x, config_.projection.size_y, CV_8U }
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
  debug_line_cv_pub_ = nh_.advertise<sensor_msgs::Image>("line_layer/debug/line_cv", 1);
  debug_nonline_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(config_.center.debug.nonline_topic, 1);
  debug_nonline_cv_pub_ = nh_.advertise<sensor_msgs::Image>("line_layer/debug/nonline_cv", 1);

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

  projectImage(segmented_mat, camera_to_odom);
  cleanupProjections();
  insertProjectionsIntoMap(camera_to_odom);

  //  boundRadius(line, camera_to_odom.transform);
  //  boundRadius(nonline, camera_to_odom.transform);
  //
  //  insertProjectedPointclouds(line, nonline, camera_to_odom.transform);
  //
  //  line.header.frame_id = "odom";
  //  line.header.stamp = pcl_conversions::toPCL(raw_image->header.stamp);
  //  nonline.header.frame_id = "odom";
  //  nonline.header.stamp = line.header.stamp;
  //
  cv_bridge::CvImage line_msg;
  cv_bridge::CvImage nonline_msg;
  line_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  nonline_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  line_msg.image = line_buffer_;
  nonline_msg.image = freespace_buffer_;

  debug_line_cv_pub_.publish(line_msg);
  debug_nonline_cv_pub_.publish(nonline_msg);

  debugPublishPC(debug_line_pub_, line_buffer_, camera_to_odom);
  debugPublishPC(debug_nonline_pub_, freespace_buffer_, camera_to_odom);
  //  debug_line_pub_.publish(line);
  //  debug_nonline_pub_.publish(nonline);

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

void LineLayer::projectImage(const cv::Mat &segmented_mat, const geometry_msgs::TransformStamped &camera_to_odom)
{
  line_buffer_.setTo(cv::Scalar(0.0));
  freespace_buffer_.setTo(cv::Scalar(0.0));
  cv::Rect buffer_rect({}, line_buffer_.size());

  int rows = segmented_mat.rows;
  int cols = segmented_mat.cols;

  Eigen::Quaterniond rotation;
  const auto &translate_vector = camera_to_odom.transform.translation;
  Eigen::Vector3d translation{ translate_vector.x, translate_vector.y, translate_vector.z };
  tf2::convert(camera_to_odom.transform.rotation, rotation);

  grid_map::Index camera_index;  // Center of line_buffer_ and freespace_buffer_
  map_.getIndex({ translate_vector.x, translate_vector.y }, camera_index);

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
      grid_map::Index buffer_index = calculateBufferIndex(projected_point, camera_index);
      if (buffer_rect.contains({ buffer_index[0], buffer_index[1] }))
      {
        if (is_line)
        {
          line_buffer_.at<uchar>(buffer_index[0], buffer_index[1]) = 255u;
          //        line.points.emplace_back(pcl::PointXYZ(projected_point.x(), projected_point.y(),
          //        projected_point.z()));
        }
        else
        {
          freespace_buffer_.at<uchar>(buffer_index[0], buffer_index[1]) = 255u;
          //        nonline.points.emplace_back(pcl::PointXYZ(projected_point.x(), projected_point.y(),
          //        projected_point.z()));
        }
      }
    }
  }
}

grid_map::Index LineLayer::calculateBufferIndex(const Eigen::Vector3f &point, const grid_map::Index &camera_index) const
{
  grid_map::Index point_index;
  map_.getIndex({ point[0], point[1] }, point_index);

  int center_x = config_.projection.size_x / 2;
  int center_y = config_.projection.size_y / 2;
  int buffer_x = point_index[0] - camera_index[0] + center_x - 1;
  int buffer_y = point_index[1] - camera_index[1] + center_y - 1;

  return { buffer_x, buffer_y };
}

void LineLayer::markEmpty(const grid_map::Index &index, double distance, double angle)
{
  const auto distance_coeff = config_.center.miss_exponential_coeff;
  const auto angle_coeff = config_.center.miss_angle_exponential_coeff;
  const double probability = std::exp(-distance_coeff * distance - angle_coeff * std::abs(angle)) * config_.center.miss;

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

void LineLayer::cleanupProjections()
{
  // Close lines
  {
    const auto size = config_.projection.line_closing_kernel_size;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * size + 1, 2 * size + 1));
    cv::morphologyEx(line_buffer_, line_buffer_, cv::MORPH_CLOSE, kernel);
  }
  // Close freespace
  {
    const auto size = config_.projection.freespace_closing_kernel_size;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * size + 1, 2 * size + 1));
    cv::morphologyEx(freespace_buffer_, freespace_buffer_, cv::MORPH_CLOSE, kernel);
  }
  // Remove lines from freespace
  {
    cv::bitwise_not(line_buffer_, not_lines_);
    cv::bitwise_and(freespace_buffer_, not_lines_, freespace_buffer_);
  }
}

void LineLayer::insertProjectionsIntoMap(const geometry_msgs::TransformStamped &camera_to_odom)
{
  grid_map::Index camera_index;  // Center of line_buffer_ and freespace_buffer_
  const float camera_x = camera_to_odom.transform.translation.x;
  const float camera_y = camera_to_odom.transform.translation.y;
  const double camera_heading =
      tf2::getYaw(camera_to_odom.transform.rotation) + M_PI / 2.0;  // For some reason this is off by pi/2
  map_.getIndex({ camera_x, camera_y }, camera_index);

  const int center_x = config_.projection.size_x / 2;
  const int center_y = config_.projection.size_y / 2;

  const int rows = line_buffer_.rows;
  const int cols = line_buffer_.cols;
  for (int i = 0; i < rows; i++)
  {
    const auto *row_ptr = line_buffer_.ptr<uchar>(i);
    for (int j = 0; j < cols; j++)
    {
      if (row_ptr[j] == 255u)
      {
        const int map_x = camera_index[0] - center_x + 1 + i;
        const int map_y = camera_index[1] - center_y + 1 + j;
        grid_map::Index map_index{ map_x, map_y };
        grid_map::Position position;
        map_.getPosition(map_index, position);
        const auto dx = position[0] - camera_x;
        const auto dy = position[1] - camera_y;
        double distance = dx * dx + dy * dy;
        markHit(map_index, distance);
      }
    }
  }
  for (int i = 0; i < rows; i++)
  {
    const auto *row_ptr = freespace_buffer_.ptr<uchar>(i);
    for (int j = 0; j < cols; j++)
    {
      if (row_ptr[j] == 255u)
      {
        const int map_x = camera_index[0] - center_x + 1 + i;
        const int map_y = camera_index[1] - center_y + 1 + j;
        grid_map::Index map_index{ map_x, map_y };
        grid_map::Position position;
        map_.getPosition(map_index, position);
        const auto dx = position[0] - camera_x;
        const auto dy = position[1] - camera_y;
        const double distance = dx * dx + dy * dy;
        const double angle = angles::normalize_angle(camera_heading - std::atan2(dy, dx));
        markEmpty(map_index, distance, angle);
      }
    }
  }
}

void LineLayer::debugPublishPC(ros::Publisher &pub, const cv::Mat &mat, geometry_msgs::TransformStamped &camera_to_odom)
{
  PointCloud pointcloud;
  pointcloud.header.stamp = pcl_conversions::toPCL(ros::Time::now());
  pointcloud.header.frame_id = config_.map.frame_id;

  const int rows = mat.rows;
  const int cols = mat.cols;

  const int center_x = rows / 2;
  const int center_y = cols / 2;

  const float camera_x = camera_to_odom.transform.translation.x;
  const float camera_y = camera_to_odom.transform.translation.y;
  grid_map::Index camera_index;
  map_.getIndex({ camera_x, camera_y }, camera_index);
  grid_map::Position camera_pos;
  map_.getPosition(camera_index, camera_pos);

  for (int i = 0; i < rows; i++)
  {
    const auto *row_ptr = mat.ptr<uchar>(i);
    for (int j = 0; j < cols; j++)
    {
      if (row_ptr[j] == 255u)
      {
        float dx = config_.map.resolution * (i - center_x + 1);
        float dy = config_.map.resolution * (j - center_y + 1);
        float cell_x = camera_pos[0] - dx;
        float cell_y = camera_pos[1] - dy;
        pcl::PointXYZ pcl_point{ cell_x, cell_y, 0.0 };
        pointcloud.points.emplace_back(pcl_point);
      }
    }
  }

  pub.publish(pointcloud);
}

}  // namespace line_layer
