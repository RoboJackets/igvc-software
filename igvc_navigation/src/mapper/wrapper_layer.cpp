#include "wrapper_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(wrapper_layer::WrapperLayer, costmap_2d::Layer)

namespace wrapper_layer
{
WrapperLayer::WrapperLayer() : mapper_{}
{
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

  bool assumptions = master_grid.getSizeInMetersX() == mapper_.length_x_ &&
                     master_grid.getSizeInMetersY() == mapper_.width_y_ &&
                     master_grid.getResolution() == mapper_.resolution_ &&
                     static_cast<int>(master_grid.getSizeInCellsX()) == map->rows &&
                     static_cast<int>(master_grid.getSizeInCellsY()) == map->cols;
  assert(assumptions);

  unsigned char* master = master_grid.getCharMap();

  for (int i = 0; i < 255; i++)
  {
    master[2*i] = static_cast<uchar>(i);
    master[2*i+1] = static_cast<uchar>(i);
  }

  unsigned int mx, my;
  master_grid.worldToMap(0.0, 0.0, mx, my);
  unsigned int index = master_grid.getIndex(mx, my);
  //  ROS_INFO("W(0.0, 0.0) -> M(%d, %d)", mx, my);
  master[index - 1] = 255u;
  master[index] = 255u;
  master[index + 1] = 255u;

  int nRows = mapper_.width_y_ / mapper_.resolution_;
  int nCols = mapper_.length_x_ / mapper_.resolution_;

  pcl::PointCloud<pcl::PointXYZ> pointcloud;

  for (int i = 0; i < nRows; ++i)
  {
    uchar* p = map->ptr<uchar>(i);
    for (int j = 0; j < nCols; ++j)
    {
      auto idx = master_grid.getIndex(i, j);
      if (p[j] > 200u)
      {
        master[idx] = costmap_2d::LETHAL_OBSTACLE;
      }
      else
      {
        master[idx] = costmap_2d::FREE_SPACE;
      }
    }
  }
}

}  // namespace wrapper_layer
