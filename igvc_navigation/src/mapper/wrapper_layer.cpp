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
}

void WrapperLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
}

}  // namespace wrapper_layer
