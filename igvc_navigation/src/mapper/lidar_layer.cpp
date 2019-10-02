#include "lidar_layer.h"
#include <mapper/probability_utils.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include "map_config.h"

PLUGINLIB_EXPORT_CLASS(lidar_layer::LidarLayer, costmap_2d::Layer)

namespace lidar_layer
{
LidarLayer::LidarLayer() : private_nh_{ "~" }, map_{ { logodds_layer, probability_layer } }, config_{ private_nh_ }
{
  initGridmap();
  initPubSub();
  costmap_2d_ = { static_cast<unsigned int>(map_.getSize()[0]), static_cast<unsigned int>(map_.getSize()[1]),
                  map_.getResolution(), map_.getPosition()[0], map_.getPosition()[1] };
}

void LidarLayer::initGridmap()
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

void LidarLayer::initPubSub()
{
  occupied_sub_ = nh_.subscribe(config_.lidar.occupied_topic, 1, &LidarLayer::occupiedCallback, this);
  free_sub_ = nh_.subscribe(config_.lidar.free_topic, 1, &LidarLayer::freeCallback, this);
  gridmap_pub_ = nh_.advertise<grid_map_msgs::GridMap>(config_.map.debug.map_topic, 1);
}

void LidarLayer::onInitialize()
{
}

void LidarLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                              double *max_x, double *max_y)
{
  // TODO: Change this to only the points that have been updated
  *min_x = map_.getPosition()[0] - map_.getLength()[0] / 2;
  *max_x = map_.getPosition()[0] + map_.getLength()[0] / 2;
  *min_y = map_.getPosition()[1] - map_.getLength()[1] / 2;
  *max_y = map_.getPosition()[1] + map_.getLength()[1] / 2;
}

void LidarLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int /*min_i*/, int /*min_j*/, int /*max_i*/,
                             int /*max_j*/)
{
  updateProbabilityLayer();
  transferToCostmap();
  debugPublishMap();

  assert(costmap_2d_.getSizeInCellsX() == master_grid.getSizeInCellsX());
  assert(costmap_2d_.getSizeInCellsY() == master_grid.getSizeInCellsY());

  size_t num_cells = costmap_2d_.getSizeInCellsX() * costmap_2d_.getSizeInCellsY();

  uchar *master_array = master_grid.getCharMap();
  uchar *lidar_array = costmap_2d_.getCharMap();
  for (size_t i = 0; i < num_cells; i++)
  {
    uchar old_cost = master_array[i];
    if (old_cost == costmap_2d::NO_INFORMATION || old_cost < lidar_array[i])
    {
      master_array[i] = lidar_array[i];
    }
  }
}

void LidarLayer::updateProbabilityLayer()
{
  map_.get(probability_layer) = layer_->unaryExpr(&probability_utils::fromLogOdds<float>);
}

void LidarLayer::transferToCostmap()
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

void LidarLayer::occupiedCallback(const sensor_msgs::PointCloud2ConstPtr &occupied_pc)
{
  const auto [cloud, transform] = getCloudAndTransform(occupied_pc);
  insertScan(cloud, transform);
  updateMapTimestamp(occupied_pc->header.stamp);
}

void LidarLayer::freeCallback(const sensor_msgs::PointCloud2ConstPtr &free_pc)
{
  const auto [cloud, transform] = getCloudAndTransform(free_pc);
  insertFreeSpace(cloud, transform);
  updateMapTimestamp(free_pc->header.stamp);
}

void LidarLayer::debugPublishMap()
{
  map_.setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  gridmap_pub_.publish(message);
}

std::pair<pcl::PointCloud<pcl::PointXYZ>, geometry_msgs::TransformStamped>
LidarLayer::getCloudAndTransform(const sensor_msgs::PointCloud2ConstPtr &pc)
{
  auto map_frame = config_.map.frame_id;
  auto pc_frame = pc->header.frame_id;
  ros::Time cloud_stamp = pc->header.stamp;

  // TODO: Make the timeout a parameter
  if (!tf_->canTransform(map_frame, pc_frame, cloud_stamp, ros::Duration(1.0)))
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(1.0, "pc_transform_timeout",
                                   "Failed to find transform for pointcloud from frame '"
                                       << pc_frame << "' to frame '" << map_frame
                                       << "' within timeout. Using latest transform...");
    cloud_stamp = ros::Time(0);
  }

  sensor_msgs::PointCloud2 transformed_cloud;
  geometry_msgs::TransformStamped transform =
      tf_->lookupTransform(map_frame, pc_frame, cloud_stamp, ros::Duration(0.1));
  tf2::doTransform(*pc, transformed_cloud, transform);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(transformed_cloud, pcl_cloud);

  return std::make_pair(pcl_cloud, transform);
}

void LidarLayer::insertScan(const LidarLayer::PointCloud &pointcloud,
                            const geometry_msgs::TransformStamped &lidar_transform)
{
  double lidar_x = lidar_transform.transform.translation.x;
  double lidar_y = lidar_transform.transform.translation.y;
  grid_map::Position lidar_pos{ lidar_x, lidar_y };

  std::unordered_set<grid_map::Index> free_cells{};
  free_cells.reserve(50 * pointcloud.size());
  last_occupied_cells_.clear();
  last_occupied_cells_.reserve(pointcloud.size());

  for (const auto &point : pointcloud)
  {
    grid_map::Position end_point{ point.x, point.y };
    grid_map::Index end_index;
    map_.getIndex(end_point, end_index);

    for (grid_map::LineIterator it{ map_, lidar_pos, end_point }; !it.isPastEnd(); ++it)
    {
      free_cells.emplace(*it);
      // Break when iterator gets to the actual cell
      if (end_index[0] == (*it)[0] && end_index[1] == (*it)[1])
      {
        break;
      }
    }
    markScanHit(end_index, end_point, lidar_pos);
    last_occupied_cells_.emplace(end_index);
  }

  for (const auto &index : free_cells)
  {
    markScanMiss(index);
  }
}

void LidarLayer::insertFreeSpace(const PointCloud &pointcloud, const geometry_msgs::TransformStamped &lidar_transform)
{
  assert(pointcloud.size() % 2 == 0);

  double lidar_x = lidar_transform.transform.translation.x;
  double lidar_y = lidar_transform.transform.translation.y;
  grid_map::Position lidar_pos{ lidar_x, lidar_y };

  std::unordered_set<grid_map::Index> free_cells{};
  free_cells.reserve(50 * pointcloud.size());

  for (size_t i = 0; i < pointcloud.size(); i += 2)
  {
    // points for free space are pairs of (min_range, endpoint)
    const auto &startpoint = pointcloud.points[i];
    const auto &endpoint = pointcloud.points[i + 1];

    grid_map::Position startpoint_pos{ startpoint.x, startpoint.y };
    grid_map::Position endpoint_pos{ endpoint.x, endpoint.y };

    for (grid_map::LineIterator it{ map_, startpoint_pos, endpoint_pos }; !it.isPastEnd(); ++it)
    {
      free_cells.emplace(*it);
    }
  }

  for (const auto &index : free_cells)
  {
    markFreeMiss(index);
  }
}

void LidarLayer::markScanHit(const grid_map::Index &index, const grid_map::Position &point,
                             const grid_map::Position &lidar_pos)
{
  const double distance = (lidar_pos - point).norm();
  const auto coeff = config_.lidar.hit_exponential_coeff;
  const double probability = std::exp(-coeff * distance) * config_.lidar.scan_hit;

  (*layer_)(index[0], index[1]) += static_cast<float>(probability_utils::toLogOdds(probability));
}

void LidarLayer::updateMapTimestamp(const ros::Time &stamp)
{
  map_.setTimestamp(stamp.toNSec());
}

}  // namespace lidar_layer
