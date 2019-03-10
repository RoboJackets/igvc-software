#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <igvc_utils/StringUtils.hpp>
#include <mutex>
#include <string>
#include "conversion.h"

#include <igvc_utils/NodeUtils.hpp>

ros::Publisher waypoint_pub;
std::vector<geometry_msgs::PointStamped> waypoints;
geometry_msgs::PointStamped current_waypoint;
geometry_msgs::Point map_origin;
std::mutex current_mutex;

std::string global_link;
std::string robot_link;
double goal_threshold;

double dmsToDec(std::string dms)
{
  auto qMarkIter = dms.find('?');
  auto aposIter = dms.find('\'');
  auto qouteIter = dms.find('\"');
  auto degrees = stod(dms.substr(0, qMarkIter));
  auto minutes = stod(dms.substr(qMarkIter + 1, aposIter));
  auto seconds = stod(dms.substr(aposIter + 1, qouteIter));
  auto dirChar = dms[dms.size() - 1];

  degrees += minutes / 60.0;
  degrees += seconds / 3600.0;

  if (dirChar == 'W' || dirChar == 'S')
    degrees *= -1;

  return degrees;
}

/**
 * Parses waypoints text file
 * # denotes a line to ignore
 * Assumes a series of lat,lon pairs
 */
void loadWaypointsFile(std::string path, std::vector<geometry_msgs::PointStamped>& waypoints)
{
  if (path.empty())
  {
    ROS_ERROR_STREAM("Could not load empty file path.");
    return;
  }

  std::ifstream file;
  file.open(path.c_str());

  if (!file.is_open())
  {
    ROS_INFO_STREAM("Could not open file: " << path);
    return;
  }

  std::string line = "";
  auto lineIndex = 1;
  while (!file.eof())
  {
    getline(file, line);

    // ignore any line starting with a #
    if (!line.empty() && line[0] != '#')
    {
      std::vector<std::string> tokens = split(line, ',');

      if (tokens.size() != 2)
      {
        ROS_ERROR_STREAM(path << ":" << lineIndex << " - " << tokens.size() << " tokens instead of 2.");
        return;
      }

      double lat, lon;
      if (tokens[0].find('?') != std::string::npos)
        lat = dmsToDec(tokens[0]);
      else
        lat = stod(tokens[0]);
      if (tokens[1].find('?') != std::string::npos)
        lon = dmsToDec(tokens[1]);
      else
        lon = stod(tokens[1]);

      geometry_msgs::PointStamped p;

      UTM(lat, lon, &(p.point.x), &(p.point.y));

      waypoints.push_back(p);
    }

    lineIndex++;
  }
}

/**
 * Advances to the next waypoint if the current one has been reached within goal_threshold
 */
void positionCallback(const nav_msgs::OdometryConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(current_mutex);
  geometry_msgs::PointStamped cur = current_waypoint;
  cur.point.x -= map_origin.x;
  cur.point.y -= map_origin.y;

  if (igvc::get_distance(msg->pose.pose.position.x, msg->pose.pose.position.y, cur.point.x, cur.point.y) < goal_threshold)
  {
    // advance to next waypoint.
    current_waypoint = waypoints.front();
    if (waypoints.size() > 1)
    {
      waypoints.erase(waypoints.begin());
      ROS_INFO("Waypoint Source moving to next");
    }
  }
}

/**
 * Uses current GPS position and most recent localization to project the waypoint into the global map frame
 */
void originCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(current_mutex);
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  geometry_msgs::Point position;
  UTM(msg->latitude, msg->longitude, &(position.x), &(position.y));
  if (tf_listener.waitForTransform(global_link, robot_link, ros::Time(0), ros::Duration(3.0)))
  {
    tf_listener.lookupTransform(global_link, robot_link, ros::Time(0), transform);
    geometry_msgs::TransformStamped result;
    tf::transformStampedTFToMsg(transform, result);
    position.x -= result.transform.translation.x;
    position.y -= result.transform.translation.y;
    map_origin = position;
  } else {
    ROS_ERROR_STREAM("cannot find transform from " << global_link << " to " << robot_link << " in waypoint file");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_source");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  double pub_rate;
  std::string path;
  std::string localization_topic, gps_topic;
  igvc::getParam(pNh, "file", path);
  igvc::param(pNh, "rate", pub_rate, 1.0);
  igvc::param(pNh, "goal_threshold", goal_threshold, 1.0);
  igvc::param(pNh, "global_link", global_link, std::string("/odom"));
  igvc::param(pNh, "robot_link", robot_link, std::string("/base_footprint"));
  igvc::param(pNh, "localization_topic", localization_topic, std::string("/odometry/filtered"));
  igvc::param(pNh, "gps_topic", gps_topic, std::string("/gps/filtered"));

  ROS_INFO_STREAM("Loading waypoints from " << path);

  waypoint_pub = nh.advertise<geometry_msgs::PointStamped>("/waypoint", 1);

  ros::Subscriber odom_sub = nh.subscribe(localization_topic, 1, positionCallback);

  ros::Subscriber origin_sub = nh.subscribe(gps_topic, 1, originCallback);

  loadWaypointsFile(path, waypoints);

  if (!waypoints.empty())
  {
    ROS_INFO_STREAM(waypoints.size() << " waypoints found.");
    current_waypoint = waypoints.front();
    ros::Rate rate(pub_rate);  // 1 Hz
    while (ros::ok())
    {
      {
        std::lock_guard<std::mutex> lock(current_mutex);
        auto waypoint_for_pub = current_waypoint;
        waypoint_for_pub.header.stamp = ros::Time::now();
        waypoint_for_pub.header.seq++;
        waypoint_for_pub.header.frame_id = "odom";
        waypoint_for_pub.point.x -= map_origin.x;
        waypoint_for_pub.point.y -= map_origin.y;
        waypoint_pub.publish(waypoint_for_pub);
      }

      ros::spinOnce();
      rate.sleep();
    }
  }
  else
  {
    ROS_ERROR("No valid waypoint entries found.");
  }

  return 0;
}
