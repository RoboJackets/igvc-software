#include "PoseSlamEstimator.h"
#include <parameter_assertions/assertions.h>
#include <igvc_utils/NodeUtils.hpp>

PoseSlamEstimator::PoseSlamEstimator(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
{
  ros::NodeHandle pNh("~");

  // Subscribe to sensor readings.
  imu_sub_ = nh_.subscribe("/imu", 1, &PoseSlamEstimator::imu_callback, this);
  gps_sub_ = nh_.subscribe("/gps", 1, &PoseSlamEstimator::gps_callback, this);

  // Publish estimated trajectory.
  trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("/pose_path", 1);

  // assertions::getParam(pNh, "c_space", configuration_space_);

  // ros::Rate rate(rate_);  // path update rate

  while (ros::ok())
  {
    ros::spinOnce();  // handle subscriber callbacks

    // ROS_INFO_STREAM_COND(num_nodes_updated > 0, num_nodes_updated << " nodes updated");
    // rate.sleep();
  }
}

void PoseSlamEstimator::imu_callback(const sensor_msgs::ImuConstPtr& msg) {
    return;
}

void PoseSlamEstimator::gps_callback(const sensor_msgs::NavSatFixConstPtr& msg) {

    if (!initial_gps_coords_received)
        initial_gps_coords_received = true;

    // Convert gps readings to ned.
    double north, east, down;
    geodetic2Ned(
        static_cast<double>(msg->latitude),
        static_cast<double>(msg->longitude),
        static_cast<double>(msg->altitide),
        north, east, down
    );

    return;
}



void PoseSlamEstimator::publish_path()
{
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "odom";
}
