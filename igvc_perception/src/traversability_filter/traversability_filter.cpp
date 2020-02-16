#include "traversability_filter.h"
#include <grid_map_ros/GridMapRosConverter.hpp>

TraversabilityFilter::TraversabilityFilter() : filter_chain_("grid_map::GridMap")
{
  private_nh_ = ros::NodeHandle("~");
  elevation_map_sub_ = private_nh_.subscribe("/elevation_mapping/elevation_map_raw", 1,
                                             &TraversabilityFilter::elevationMapCallback, this);
  traversability_map_pub_ = private_nh_.advertise<grid_map_msgs::GridMap>("/traversability_map", 1);

  // setup filter chain
  if (!filter_chain_.configure("/traversability_filter/traversability_map_filters", private_nh_))
  {
    ROS_ERROR_STREAM("Could not configure filter chain!");
    return;
  }
}

void TraversabilityFilter::elevationMapCallback(const grid_map_msgs::GridMap& message)
{
  grid_map::GridMap input_map;
  grid_map::GridMapRosConverter::fromMessage(message, input_map);

  grid_map::GridMap output_map;
  if (!filter_chain_.update(input_map, output_map))
  {
    ROS_ERROR_STREAM("Could not update the grid map filter chain!");
    return;
  }

  grid_map_msgs::GridMap output_message;
  grid_map::GridMapRosConverter::toMessage(output_map, output_message);
  traversability_map_pub_.publish(output_message);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traversability_filter");
  TraversabilityFilter tf = TraversabilityFilter();
  ros::spin();
}
