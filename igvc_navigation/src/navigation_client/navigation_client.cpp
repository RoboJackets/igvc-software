#include "navigation_client.h"
#include <parameter_assertions/assertions.h>

#include <robot_localization/navsat_conversions.h>
#include <igvc_utils/NodeUtils.hpp>
#include <igvc_utils/StringUtils.hpp>

NavigationClient::NavigationClient()
{
  ros::NodeHandle private_nh("~");

  assertions::getParam(private_nh, "read_from_file", reading_from_file_);
  if (reading_from_file_)
  {
    assertions::getParam(private_nh, "/waypoint_file_path", waypoint_file_path_);
  }

  rviz_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &NavigationClient::rvizWaypointCallback, this);
  back_circle_response_pub_ = nh_.advertise<igvc_msgs::BackCircleResponse>("/back_circle_response", 1);

  back_circle_client = nh_.serviceClient<igvc_msgs::BackCircle>("/back_circle_service");

  waitForTransform();
  waitForServer();

  if (reading_from_file_)
  {
    std::vector<geometry_msgs::PointStamped> waypoints = loadWaypointsFromFile();
    ROS_INFO_STREAM("Reading from file.");
    ros::Duration(2.0).sleep();  // give time for file to be read
    sendWaypoints(waypoints);
  }
  else
  {
    ROS_INFO_STREAM("Waiting for waypoints from rviz.");
  }
}

void NavigationClient::waitForTransform()
{
  const double waiting_time = 5.0;
  while (!tf_listener_.waitForTransform("odom", "utm", ros::Time(0), ros::Duration(waiting_time)))
  {
    ROS_INFO_STREAM("utm->odom transform not found. waiting...");
  }
  ROS_INFO_STREAM("utm->odom transform found!");
}

void NavigationClient::waitForServer()
{
  const double waiting_time = 5.0;
  while (!client.waitForServer(ros::Duration(waiting_time)))
  {
    ROS_INFO_STREAM("Waiting for the navigation server to come up");
  }
  ROS_INFO_STREAM("Connected to navigation server!");
  while (!back_circle_client.waitForExistence(ros::Duration(waiting_time)))
  {
    ROS_INFO_STREAM("Waiting for the back circle service client to come up");
  }
  ROS_INFO_STREAM("Connected to back circle service server!");
}

std::vector<geometry_msgs::PointStamped> NavigationClient::loadWaypointsFromFile()
{
  ROS_INFO_STREAM_ONCE("Loading waypoints from " << waypoint_file_path_);

  std::vector<geometry_msgs::PointStamped> waypoints;

  if (waypoint_file_path_.empty())
  {
    ROS_ERROR_STREAM("Could not load waypoints. Empty file path.");
    ros::shutdown();
    // return the empty waypoints list
    return waypoints;
  }

  // get the latitude and longitudes from the waypoint file
  std::vector<LatLong> coordinates = parseWaypointFile();

  // convert the latitude and longitudes to PointStamped in odom
  for (LatLong lat_long : coordinates)
  {
    geometry_msgs::PointStamped waypoint = convertLatLongToOdom(lat_long);
    waypoints.push_back(waypoint);
  }

  ROS_INFO_STREAM_ONCE(waypoints.size() << " waypoints loaded.");
  return waypoints;
}

std::vector<LatLong> NavigationClient::parseWaypointFile()
{
  std::vector<LatLong> coordinates;
  std::vector<LatLong> empty;  // return result when error occurs

  std::ifstream file;
  file.open(waypoint_file_path_.c_str());

  if (!file.is_open())
  {
    ROS_INFO_STREAM("Could not open file: " << waypoint_file_path_);
    ros::shutdown();
    return empty;
  }

  unsigned short line_index = 1;
  for (std::string line; std::getline(file, line);)
  {
    // remove whitespace
    line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
    if (line.front() != '#' && !line.empty())
    {
      std::vector<std::string> tokens = split(line, ',');
      if (tokens.size() != 2)
      {
        ROS_ERROR_STREAM(waypoint_file_path_ << ":" << line_index << " - " << tokens.size() << " tokens instead of 2.");
        ros::shutdown();
        return empty;
      }

      // Convert latitude and longitude to decimal degrees if currently in degrees minutes second. Specified by '?'
      // symbol.
      LatLong coord{};
      coord.latitude = (tokens[0].find('?') != std::string::npos) ? convertDmsToDec(tokens[0]) : stod(tokens[0]);
      coord.longitude = (tokens[1].find('?') != std::string::npos) ? convertDmsToDec(tokens[1]) : stod(tokens[1]);

      coordinates.push_back(coord);
      ++line_index;
    }
  }

  return coordinates;
}

geometry_msgs::PointStamped NavigationClient::convertLatLongToOdom(LatLong lat_long)
{
  geometry_msgs::PointStamped waypoint_utm;
  RobotLocalization::NavsatConversions::UTM(lat_long.latitude, lat_long.longitude, &(waypoint_utm.point.x),
                                            &(waypoint_utm.point.y));
  waypoint_utm.header.frame_id = "utm";

  // transform utm frame to odom frame
  geometry_msgs::PointStamped waypoint_odom;
  tf_listener_.transformPoint("odom", ros::Time(0), waypoint_utm, "odom", waypoint_odom);
  return waypoint_odom;
}

double NavigationClient::convertDmsToDec(std::string dms)
{
  const auto seconds_in_minute = 60.0;
  const auto seconds_in_hour = 3600.0;

  const auto q_mark_iter = dms.find('?');
  const auto apos_iter = dms.find('\'');
  const auto quote_iter = dms.find('\"');
  const auto dir_char = dms[dms.size() - 1];

  const auto minutes = stod(dms.substr(q_mark_iter + 1, apos_iter));
  const auto seconds = stod(dms.substr(apos_iter + 1, quote_iter));

  auto degrees = stod(dms.substr(0, q_mark_iter));

  degrees += minutes / seconds_in_minute;
  degrees += seconds / seconds_in_hour;

  if (dir_char == 'W' || dir_char == 'S')
    degrees *= -1;

  return degrees;
}

void NavigationClient::sendWaypoints(const std::vector<geometry_msgs::PointStamped>& waypoints)
{
  for (const geometry_msgs::PointStamped& waypoint : waypoints)
  {
    sendGoal(waypoint, true);
  }
}

void NavigationClient::callBackCircleService()
{
  igvc_msgs::BackCircle srv;
  bool confirm = back_circle_client.call(srv);
  if (confirm)
  {
    ROS_INFO_STREAM("Calling service back_circle");
    back_circle_response_pub_.publish(srv.response);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to call service back_circle");
  }
}

void NavigationClient::sendGoal(const geometry_msgs::PoseStamped& pose, bool waiting)
{
  ROS_INFO_STREAM("Sending pose: (" << pose.pose.position.x << ", " << pose.pose.position.y
                                    << ") with yaw = " << tf::getYaw(pose.pose.orientation));
  igvc_msgs::NavigateWaypointGoal goal;
  goal.target_pose = pose;
  goal.fix_goal_orientation = false;

  callBackCircleService();

  if (waiting)
  {
    client.sendGoalAndWait(goal);
  }
  else
  {
    client.sendGoal(goal);
  }
}

void NavigationClient::sendGoal(const geometry_msgs::PointStamped& point, bool waiting)
{
  ROS_INFO_STREAM("Sending point: (" << point.point.x << ", " << point.point.y << ")");

  geometry_msgs::PoseStamped pose;
  pose.header = point.header;
  pose.pose.position = point.point;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  igvc_msgs::NavigateWaypointGoal goal;
  goal.target_pose = pose;
  goal.fix_goal_orientation = true;

  callBackCircleService();

  if (waiting)
  {
    client.sendGoalAndWait(goal);
  }
  else
  {
    client.sendGoal(goal);
  }
}

void NavigationClient::rvizWaypointCallback(const geometry_msgs::PoseStamped& pose)
{
  if (reading_from_file_)
  {
    ROS_ERROR_STREAM("Cannot send goal from rviz, reading from file.");
  }
  else
  {
    sendGoal(pose, false);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_client");
  NavigationClient nav = NavigationClient();
  ros::spin();
}