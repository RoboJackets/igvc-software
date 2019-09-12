#include "wrapper_layer.h"
#include <parameter_assertions/assertions.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(wrapper_layer::WrapperLayer, costmap_2d::Layer)

namespace wrapper_layer
{
WrapperLayer::WrapperLayer() : mapper_{}
{
  ros::NodeHandle private_nh{ "~" };
  double threshold_double;
  assertions::getParam(private_nh, "lethal_threshold", threshold_double);
  occupied_threshold_ = static_cast<uchar>(std::round(255.0 / threshold_double));
}

void WrapperLayer::onInitialize()
{
}

void WrapperLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
  *min_x = -mapper_.length_x_ / 2.0;
  *min_y = -mapper_.width_y_ / 2.0;
  *max_x = mapper_.length_x_ / 2.0;
  *max_y = mapper_.width_y_ / 2.0;
}

void WrapperLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::optional<cv::Mat> map = mapper_.mapper_->getMap();
  if (!map)
  {
    ROS_WARN_STREAM_THROTTLE(1, "Couldn't get a map");
    return;
  }

  ROS_ASSERT(master_grid.getSizeInMetersX() == mapper_.length_x_);
  ROS_ASSERT(master_grid.getSizeInMetersY() == mapper_.width_y_);
  ROS_ASSERT(master_grid.getResolution() == mapper_.resolution_);
  ROS_ASSERT(static_cast<int>(master_grid.getSizeInCellsX()) == map->rows);
  ROS_ASSERT(static_cast<int>(master_grid.getSizeInCellsY()) == map->cols);

  unsigned char* master = master_grid.getCharMap();

  int rows = map->rows;
  int cols = map->cols;

  pcl::PointCloud<pcl::PointXYZ> pointcloud;

  for (int i = 0; i < rows; ++i)
  {
    auto* p = map->ptr<uchar>(i);
    for (int j = 0; j < cols; ++j)
    {
      auto idx = master_grid.getIndex(i, j);
      if (p[j] > occupied_threshold_)
      {
        master[idx] = costmap_2d::LETHAL_OBSTACLE;
      }
      else
      {
        master[idx] = p[j];
      }
    }
  }
}

}  // namespace wrapper_layer
