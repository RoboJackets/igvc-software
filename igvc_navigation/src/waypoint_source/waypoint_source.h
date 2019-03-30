#include <robot_localization/navsat_conversions.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <fstream>
#include <string>

#include <igvc_utils/StringUtils.hpp>
#include "igvc_utils/NodeUtils.hpp"

class WaypointSource
{
public:
  /**
  Launch node and publish waypoints from specified waypoints file

  @param[in] nodehandle the ros::NodeHandle that instrantiates everything necessary
                        for this node's operation
  */
  WaypointSource(ros::NodeHandle* nodehandle);

private:
  ros::NodeHandle nh_;

  // waypoints to navigate to
  std::vector<geometry_msgs::PointStamped> waypoints_;

  // launch params
  std::string path_;  // file to load waypoints from
  double waypoint_threshold_; // minimum distance in meters before a waypoint is considered reached

  // pubs
  ros::Publisher waypoint_pub_;

  // subs
  ros::Subscriber odom_sub_;

  // node that automatically subscribes to ROS transform messages
  tf::TransformListener tf_listener_;

  /**
  Loads waypoints from path_, transforms them into the UTM frame, and stores
  them in waypoints_.
  */
  void load_waypoints_file();

  /**
  degrees minutes seconds to decimal degrees conversion function

  @param[in] dms lat or long in degrees minutes seconds
  @return input value in decimal degrees
  */
  double dms_to_dec(std::string dms);

  /**
  (ROS Callback) Checks if the robot is less than a threshold distance from the
  current waypoint using the robot's current odometry.

  @param[in] msg most recent odometry message on the "odometry/filtered" topic
  */
  void position_callback(const nav_msgs::OdometryConstPtr& msg);
};
