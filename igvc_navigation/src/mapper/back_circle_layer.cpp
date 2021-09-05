#include "back_circle_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(back_circle_layer::BackCircleLayer, costmap_2d::Layer)

namespace back_circle_layer
{
BackCircleLayer::BackCircleLayer() : private_nh_("~")
{
}

// Get a subscriber in here that subscribes to a topic published to in the back_circle_server.cpp

void BackCircleLayer::onInitialize()
{
  matchSize();
  initPubSub();
  current_ = true;
}

void BackCircleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                   double *max_x, double *max_y)
{
  updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

  *min_x = getOriginX();
  *max_x = getOriginX() + getSizeInMetersX();
  *min_y = getOriginY();
  *max_y = getOriginY() + getSizeInMetersY();
}

void BackCircleLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  uint8_t *master_array = master_grid.getCharMap();
  uint8_t *line_array = getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (size_t i = 0; i < xVals.size(); ++i)
  {
    // If costmap contains the back circle point, add it to the costmap
    unsigned int mx;
    unsigned int my;
    if (master_grid.worldToMap(xVals.at(i), yVals.at(i), mx, my))
    {
      master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

void BackCircleLayer::initPubSub()
{
  back_circle_sub_ = nh_.subscribe("/back_circle_response", 1, &BackCircleLayer::costmapCallback, this);
}

void BackCircleLayer::costmapCallback(const igvc_msgs::BackCircleResponse::ConstPtr &msg)
{
  xVals = msg->x;
  yVals = msg->y;
  ROS_INFO_STREAM(msg);
}

}  // namespace back_circle_layer