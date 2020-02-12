#ifndef SRC_NAVIGATION_CLIENT_H
#define SRC_NAVIGATION_CLIENT_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <igvc_msgs/NavigateWaypointAction.h>
#include <tf/transform_listener.h>

#include <fstream>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using MoveBaseClient = actionlib::SimpleActionClient<igvc_msgs::NavigateWaypointAction>;

struct LatLong
{
  double latitude;
  double longitude;
};

class NavigationClient
{
public:
  NavigationClient();
  ros::NodeHandle nh_;
  ros::Subscriber rviz_sub_;
  tf::TransformListener tf_listener_;

private:
  // action lib client
  MoveBaseClient client = MoveBaseClient("/move_to_waypoint", true);

  // params
  bool reading_from_file_;          // true if reading from waypoint file, false if from rviz
  std::string waypoint_file_path_;  // path to waypoint file

  /**
  Waits for a transform from utm to odom
   */
  void waitForTransform();

  /**
  Waits for the action lib server to become available
   */
  void waitForServer();

  /**
  Loads waypoints from file_path, transforms them into the UTM frame, and returns a vector of PointStamped
  @param[in] file_path Path to waypoint file
  */
  std::vector<geometry_msgs::PointStamped> loadWaypointsFromFile();

  /**
  Reads over a waypoint file and returns the latitudes and longitudes
  @return vector of LatLongs corresponding to waypoints
   */
  std::vector<LatLong> parseWaypointFile();

  /**
  Converts latitudes and longitudes to the odom frame
  @param[in] latlong latitude and longitude to convert
  @return PointStamped in odom frame
   */
  geometry_msgs::PointStamped convertLatLongToOdom(LatLong lat_long);

  /**
  Converts degrees minutes seconds to decimal degrees

  @param[in] dms lat or long in degrees minutes seconds
  @return input value in decimal degrees
  */
  static double convertDmsToDec(std::string dms);

  /**
  Sends all waypoints to navigation server
   */
  void sendWaypoints(const std::vector<geometry_msgs::PointStamped>& waypoints);

  /**
  sends `pose` as a goal and waits if `waiting` is true

  @param[in] pose PoseStamped to send
  @param[in] waiting True if client should wait for server response
   */
  void sendGoal(const geometry_msgs::PoseStamped& pose, bool waiting);

  /**
  sends `point` as a goal with orientation of yaw = 0 and waits if `waiting` is true

  @param[in] point PointStamped to send
  @param[in] waiting True if client should wait for server response
   */
  void sendGoal(const geometry_msgs::PointStamped& point, bool waiting);

  /**
  (ROS Callback) sends waypoints from rviz to the navigation server

  @param[in]
  */
  void rvizWaypointCallback(const geometry_msgs::PoseStamped& pose);
};

#endif  // SRC_NAVIGATION_CLIENT_H
