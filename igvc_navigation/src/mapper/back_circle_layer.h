#ifndef SRC_BACK_CIRCLE_LAYER_H
#define SRC_BACK_CIRCLE_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <igvc_msgs/BackCircle.h>
#include <igvc_msgs/BackCircleResponse.h>
#include "back_circle_layer_config.h"

namespace back_circle_layer
{
class BackCircleLayer : public costmap_2d::CostmapLayer
{
public:
  BackCircleLayer();
  ros::Subscriber back_circle_sub_ = nh_.subscribe("/back_circle", 1, &BackCircleLayer::costmapCallback, this);

  void onInitialize() override;

  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;

  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  std::vector<double> xVals, yVals;



private:
  void initPubSub();
  void costmapCallback(const igvc_msgs::BackCircleResponse::ConstPtr &msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber costmap_sub_;

  BackCircleLayerConfig config_;
  //
  

};
} // namespace back_circle_layer

#endif // SRC_BACK_CIRCLE_LAYER