#include "back_circle_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(back_circle_layer::BackCircleLayer, costmap_2d::CostmapLayer)

namespace back_circle_layer
{
BackCircleLayer::BackCircleLayer() : private_nh_("~"), config_(private_nh_)
{
}

// Get a subscriber in here that subscribes to a topic published to in the main.cpp

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

  for (int j = min_j; j < max_j; j++) 
  {
    for (int i = min_i; i < max_i; i++) 
    {
      // If back_circle_pointcloud contains the point, add it to the costmap
      // might have to do tf transforms
      unsigned int mx;
      unsigned int my;
      if (master_grid.worldToMap(xVals[i], yVals[j], mx, my)) {
        master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void BackCircleLayer::initPubSub()
{
  costmap_sub_ = nh_.subscribe(config_.topic, 1, &BackCircleLayer::costmapCallback, this);
}

void BackCircleLayer::costmapCallback(const igvc_msgs::BackCircleResponse::ConstPtr &msg)
{
  for (int i = 0; i < msg->back_circle_x.size(); i++)
  {
    for (int j = 0; j < msg->back_circle_y.size(); j++)
    {
      xVals.push_back(msg->back_circle_x[i]);
      yVals.push_back(msg->back_circle_y[j]);
    }
  }

}

}