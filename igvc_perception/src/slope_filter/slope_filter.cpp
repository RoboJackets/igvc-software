#include "slope_filter.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <parameter_assertions/assertions.h>
#include <grid_map_cv/grid_map_cv.hpp>

TraversabilityFilter::TraversabilityFilter() : filter_chain_("grid_map::GridMap")
{
  ROS_INFO_STREAM("RAW INFO RECEIVED");
  private_nh_ = ros::NodeHandle("~");
  elevation_map_sub_ =
      private_nh_.subscribe("/elevation_mapping/elevation_map", 1, &TraversabilityFilter::elevationMapCallback, this);
  traversability_map_pub_ = private_nh_.advertise<grid_map_msgs::GridMap>("/slope/gridmap", 1);

  assertions::getParam(private_nh_, "erode_radius", erode_radius);
  assertions::getParam(private_nh_, "dilate_radius", dilate_radius);

  // setup filter chain
  if (!filter_chain_.configure("/slope_filter/slope_map_filters", private_nh_))
  {
    ROS_ERROR_STREAM("Could not configure filter chain!");
    return;
  }
}

void TraversabilityFilter::elevationMapCallback(const grid_map_msgs::GridMap& message)
{
  grid_map::GridMap input_map;
  grid_map::GridMapRosConverter::fromMessage(message, input_map);

  grid_map::GridMap slope_map;
  if (!filter_chain_.update(input_map, slope_map))
  {
    ROS_ERROR_STREAM("Could not update the grid map filter chain!");
    return;
  }

  cv::Mat elevation_data_mask;
  grid_map::GridMapCvConverter::toImage<cv::uint8_t, 1>(input_map, "elevation", CV_8U, -1.0, 0.0, elevation_data_mask);
  grid_map::GridMapCvConverter::addLayerFromImage<cv::uint8_t, 1>(elevation_data_mask, "mask_original", slope_map, 0,
                                                                  255, 0);

  cv::erode(elevation_data_mask, elevation_data_mask,
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erode_radius + 1, 2 * erode_radius + 1),
                                      cv::Point(erode_radius, erode_radius)));
  grid_map::GridMapCvConverter::addLayerFromImage<cv::uint8_t, 1>(elevation_data_mask, "mask_eroded", slope_map, 0, 255,
                                                                  0);

  cv::dilate(elevation_data_mask, elevation_data_mask,
             cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_radius + 1, 2 * dilate_radius + 1),
                                       cv::Point(dilate_radius, dilate_radius)));
  grid_map::GridMapCvConverter::addLayerFromImage<cv::uint8_t, 1>(elevation_data_mask, "mask_dilated", slope_map, 0,
                                                                  255, 0);

  grid_map_msgs::GridMap output_message;
  grid_map::GridMapRosConverter::toMessage(slope_map, output_message);
  traversability_map_pub_.publish(output_message);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slope_filter");
  TraversabilityFilter tf = TraversabilityFilter();
  ros::spin();
}
