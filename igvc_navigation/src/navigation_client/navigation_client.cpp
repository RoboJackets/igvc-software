#include "navigation_client.h"
#include <parameter_assertions/assertions.h>

#include <robot_localization/navsat_conversions.h>
#include <igvc_utils/NodeUtils.hpp>
#include <igvc_utils/StringUtils.hpp>

NavigationClient::NavigationClient()
{
  ros::NodeHandle private_nh("~");

  assertions::getParam(private_nh, "reading_from_file", reading_from_file_);
  assertions::getParam(private_nh, "/waypoint_file_path", waypoint_file_path_);
  assertions::getParam(private_nh, "waypoint_radius", waypoint_radius_);

  const double waiting_time = 5.0;
  // wait for the action server to come up
  while (!client.waitForServer(ros::Duration(waiting_time)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  rviz_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &NavigationClient::rvizWaypointCallback, this);

  // wait for the utm->odom transform to become available
  while (!tf_listener_.waitForTransform("odom", "utm", ros::Time(0), ros::Duration(waiting_time)))
  {
    ROS_INFO_STREAM("utm->odom transform not found. waiting...");
  }

  ROS_INFO_STREAM("utm->odom transform found!");

  if (reading_from_file_)
  {
    loadWaypointsFile();
    for (const geometry_msgs::PointStamped& waypoint : waypoints_list_)
    {
      sendPointAsGoalAndWait(waypoint);
    }
  }
  else
  {
    ROS_INFO_STREAM("Waiting for waypoints from rviz.");
    ros::spin();
  }
}

void NavigationClient::loadWaypointsFile()
{
  ROS_INFO_STREAM_ONCE("Loading waypoints from " << waypoint_file_path_);

  if (waypoint_file_path_.empty())
  {
    ROS_ERROR_STREAM("Could not load waypoints. Empty file path.");
    return;
  }

  std::ifstream file;
  file.open(waypoint_file_path_.c_str());

  if (!file.is_open())
  {
    ROS_INFO_STREAM("Could not open file: " << waypoint_file_path_);
    return;
  }

  std::string line;
  auto line_index = 1;
  while (!file.eof())
  {
    getline(file, line);

    if (!line.empty() && line[0] != '#')
    {
      std::vector<std::string> tokens = split(line, ',');

      if (tokens.size() != 2)
      {
        ROS_ERROR_STREAM(waypoint_file_path_ << ":" << line_index << " - " << tokens.size() << " tokens instead of 2.");
        return;
      }

      // convert latitude and longiture to decimal degrees if currently in degrees minutes second. specified by '?'
      // symbol
      double lat = (tokens[0].find('?') != std::string::npos) ? convertDmsToDec(tokens[0]) : stod(tokens[0]);
      double lon = (tokens[1].find('?') != std::string::npos) ? convertDmsToDec(tokens[1]) : stod(tokens[1]);

      // transform latitude and longitude to UTM frame
      geometry_msgs::PointStamped waypoint_utm;
      RobotLocalization::NavsatConversions::UTM(lat, lon, &(waypoint_utm.point.x), &(waypoint_utm.point.y));
      waypoint_utm.header.frame_id = "utm";

      // transform utm frame to odom frame
      geometry_msgs::PointStamped waypoint_odom;
      tf_listener_.transformPoint("odom", ros::Time(0), waypoint_utm, "odom", waypoint_odom);
      waypoints_list_.push_back(waypoint_odom);
    }
    line_index++;
  }

  ROS_INFO_STREAM_ONCE(waypoints_list_.size() << " waypoints loaded.");
}

double NavigationClient::convertDmsToDec(std::string dms)
{
  auto q_mark_iter = dms.find('?');
  auto apos_iter = dms.find('\'');
  auto quote_iter = dms.find('\"');
  auto degrees = stod(dms.substr(0, q_mark_iter));
  auto minutes = stod(dms.substr(q_mark_iter + 1, apos_iter));
  auto seconds = stod(dms.substr(apos_iter + 1, quote_iter));
  auto dir_char = dms[dms.size() - 1];

  const double seconds_in_minute = 60.0;
  const double seconds_in_hour = 3600.0;
  degrees += minutes / seconds_in_minute;
  degrees += seconds / seconds_in_hour;

  if (dir_char == 'W' || dir_char == 'S')
    degrees *= -1;

  return degrees;
}

void NavigationClient::sendPoseAsGoal(const geometry_msgs::PoseStamped& pose)
{
  ROS_INFO_STREAM("Sending pose: (" << pose.pose.position.x << ", " << pose.pose.position.y
                                    << ") with yaw = " << tf::getYaw(pose.pose.orientation));
  mbf_msgs::MoveBaseGoal goal;
  goal.target_pose = pose;

  client.sendGoal(goal);
}

void NavigationClient::sendPoseAsGoalAndWait(const geometry_msgs::PoseStamped& pose)
{
  ROS_INFO_STREAM("Sending pose and waiting: (" << pose.pose.position.x << ", " << pose.pose.position.y
                                                << ") with yaw = " << tf::getYaw(pose.pose.orientation));
  mbf_msgs::MoveBaseGoal goal;
  goal.target_pose = pose;

  client.sendGoalAndWait(goal);
}

void NavigationClient::sendPointAsGoal(const geometry_msgs::PointStamped& point)
{
  ROS_INFO_STREAM("Sending point: (" << point.point.x << ", " << point.point.y << ") with yaw = 0");
  mbf_msgs::MoveBaseGoal goal;

  goal.target_pose.header = point.header;
  goal.target_pose.pose.position = point.point;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  client.sendGoal(goal);
}

void NavigationClient::sendPointAsGoalAndWait(const geometry_msgs::PointStamped& point)
{
  ROS_INFO_STREAM("Sending point: (" << point.point.x << ", " << point.point.y << ") with yaw = 0");
  mbf_msgs::MoveBaseGoal goal;

  goal.target_pose.header = point.header;
  goal.target_pose.pose.position = point.point;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  client.sendGoalAndWait(goal);
}

void NavigationClient::rvizWaypointCallback(const geometry_msgs::PoseStamped& pose)
{
  if (reading_from_file_)
  {
    ROS_ERROR("Cannot send goal from rviz, reading from file.");
  }
  else
  {
    sendPoseAsGoal(pose);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_client");
  NavigationClient nav = NavigationClient();
}
