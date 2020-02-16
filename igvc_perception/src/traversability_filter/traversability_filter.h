#ifndef SRC_TRAVERSABILITY_FILTER_H
#define SRC_TRAVERSABILITY_FILTER_H

#include <ros/ros.h>
#include <filters/filter_chain.h>
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

  void elevationMapCallback(const grid_map_msgs::GridMap& message);
};

#endif  // SRC_TRAVERSABILITY_FILTER_H
