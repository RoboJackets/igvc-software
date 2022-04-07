#ifndef SRC_SLOPE_FILTER_H
#define SRC_SLOPE_FILTER_H

#include <ros/ros.h>
#include <filters/filter_chain.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>

class TraversabilityFilter
{
public:
  TraversabilityFilter();

private:
  ros::NodeHandle private_nh_;
  ros::Subscriber elevation_map_sub_;
  ros::Publisher traversability_map_pub_;
  filters::FilterChain<grid_map::GridMap> filter_chain_;

  int erode_radius;
  int dilate_radius;

  void elevationMapCallback(const grid_map_msgs::GridMap& message);
};

#endif  // SRC_SLOPE_FILTER_H
