#include "waypoint_source.h"
#include <parameter_assertions/assertions.h>

WaypointSource::WaypointSource(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
{
  ros::NodeHandle pNh("~");

  assertions::param(pNh, "waypoint_threshold", waypoint_threshold_, 1.0);
  assertions::getParam(pNh, "file", path_);
  load_waypoints_file();

  waypoint_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/waypoint", 1);

  odom_sub_ = nh_.subscribe("/odometry/filtered", 1, &WaypointSource::position_callback, this);

  // wait for the utm->odom transform to become available
  while (!tf_listener_.waitForTransform("odom", "utm", ros::Time(0), ros::Duration(5.0)))
  {
    ROS_INFO_STREAM("utm->odom transform not found. waiting...");
  }

  ROS_INFO_STREAM("utm->odom transform found!");

  if (!waypoints_.empty())
  {
    // PointStamped containing waypoint
    geometry_msgs::PointStamped waypoint_for_pub;
    waypoint_for_pub.header.frame_id = "odom";
    ros::Rate rate(1);  // 1 Hz

    ROS_INFO_STREAM("[" << waypoints_.size() << " waypoints left]");

    while (ros::ok())
    {
      ros::spinOnce();

      waypoint_for_pub.header.stamp = ros::Time::now();
      waypoint_for_pub.header.seq++;

      tf_listener_.transformPoint("odom", ros::Time(0), waypoints_.front(), "odom", waypoint_for_pub);
      waypoint_pub_.publish(waypoint_for_pub);
      rate.sleep();
    }
  }
  else
  {
    ROS_ERROR("No valid waypoints available.");
  }
}

void WaypointSource::position_callback(const nav_msgs::OdometryConstPtr& msg)
{
  geometry_msgs::PointStamped current_waypoint_odom;

  // transform the waypoint at the head of the waypoints_ vector from
  // UTM to odom
  tf_listener_.transformPoint("odom", ros::Time(0), waypoints_.front(), "odom", current_waypoint_odom);

  // move to the next waypoint in the list if we're < waypoint_threshold_ m. away from the waypoint
  if (igvc::get_distance(msg->pose.pose.position, current_waypoint_odom.point) < waypoint_threshold_)
  {
    // advance to next waypoint.
    if (waypoints_.size() > 1)
    {
      waypoints_.erase(waypoints_.begin());
      ROS_INFO_STREAM("Waypoint reached. Moving to next waypoint [" << waypoints_.size() << " waypoints left]");
    }
    else
    {
      ROS_INFO_STREAM("All waypoints reached!");
    }
  }
}

void WaypointSource::load_waypoints_file()
{
  ROS_INFO_STREAM_ONCE("Loading waypoints from " << path_);

  if (path_.empty())
  {
    ROS_ERROR_STREAM("Could not load waypoints. Empty file path.");
    return;
  }

  std::ifstream file;
  file.open(path_.c_str());

  if (!file.is_open())
  {
    ROS_INFO_STREAM("Could not open file: " << path_);
    return;
  }

  std::string line = "";
  auto lineIndex = 1;
  while (!file.eof())
  {
    getline(file, line);

    if (!line.empty() && line[0] != '#')
    {
      std::vector<std::string> tokens = split(line, ',');

      if (tokens.size() != 2)
      {
        ROS_ERROR_STREAM(path_ << ":" << lineIndex << " - " << tokens.size() << " tokens instead of 2.");
        return;
      }

      // convert latitude and longiture to decimal degrees if currently in degrees minutes second. specified by '?'
      // symbol
      double lat = (tokens[0].find('?') != std::string::npos) ? dms_to_dec(tokens[0]) : stod(tokens[0]);
      double lon = (tokens[1].find('?') != std::string::npos) ? dms_to_dec(tokens[1]) : stod(tokens[1]);

      // transform latitude and longitude to UTM frame
      geometry_msgs::PointStamped p;
      RobotLocalization::NavsatConversions::UTM(lat, lon, &(p.point.x), &(p.point.y));
      p.header.frame_id = "utm";
      waypoints_.push_back(p);
    }
    lineIndex++;
  }
  ROS_INFO_STREAM_ONCE(waypoints_.size() << " waypoints loaded.");
}

double WaypointSource::dms_to_dec(std::string dms)
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_source");
  ros::NodeHandle nh;
  WaypointSource waypoint_source(&nh);
  return 0;
}
