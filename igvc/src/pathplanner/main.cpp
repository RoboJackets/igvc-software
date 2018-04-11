// convolve over the map
// update the map based on incremental updates and update the convolve


#include <geometry_msgs/PoseStamped.h>
#include <igvc_msgs/map.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <mutex>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "GraphSearch.hpp"
#include "igvcsearchproblemdiscrete.h"

ros::Publisher disp_path_pub;

ros::Publisher expanded_pub;

IGVCSearchProblemDiscrete search_problem;

std::mutex planning_mutex;

bool received_waypoint = false;

unsigned int current_index = 0;

void map_callback(const igvc_msgs::mapConstPtr& msg)
{
  std::lock_guard<std::mutex> planning_lock(planning_mutex);
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvShare(msg->image, msg, "mono8");
  cv::Mat img = cv_ptr->image;
  // TODO verify that image is not destructed
}

void waypoint_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(planning_mutex);
  search_problem.Goal.X = msg->point.x;
  search_problem.Goal.Y = msg->point.y;
  cout << "Waypoint received. " << search_problem.Goal.X << ", " << search_problem.Goal.Y << endl;
  received_waypoint = true;
}

void expanded_callback(const set<SearchLocation>& expanded)
{
  if (expanded_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "/odom";
    for (auto location : expanded)
    {
      cloud.points.push_back(pcl::PointXYZ(location.X, location.Y, 0));
    }
    expanded_pub.publish(cloud);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathplanner");

  ros::NodeHandle nh;

  ros::Subscriber map_sub = nh.subscribe("/map", 1, map_callback);

  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, waypoint_callback);

  disp_path_pub = nh.advertise<nav_msgs::Path>("/path", 1);

  expanded_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/expanded", 1);

  ros::NodeHandle pNh("~");

  if (!pNh.hasParam("goal_threshold") || !pNh.hasParam("threshold") ||
      !pNh.hasParam("point_turns_enabled") || !pNh.hasParam("reverse_enabled"))
  {
    ROS_ERROR_STREAM("path planner does not have all required parameters");
    return 0;
  }

  pNh.getParam("goal_threshold", search_problem.GoalThreshold);
  pNh.getParam("threshold", search_problem.Threshold);
  pNh.getParam("point_turns_enabled", search_problem.PointTurnsEnabled);
  pNh.getParam("reverse_enabled", search_problem.ReverseEnabled);

  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();

    auto distance_to_goal = search_problem.Start.distTo(search_problem.Goal);
    if (!received_waypoint || distance_to_goal == 0 || distance_to_goal > 60)
      continue;

    planning_mutex.lock();
    Path<SearchLocation, SearchMove> path;
    path = GraphSearch::AStar(search_problem, expanded_callback);
    nav_msgs::Path disp_path_msg;
    disp_path_msg.header.stamp = ros::Time::now();
    disp_path_msg.header.frame_id = "odom";
    if (path.getStates()->empty())
      path.getStates()->push_back(search_problem.Start);
    for (auto loc : *(path.getStates()))
    {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = disp_path_msg.header.stamp;
      pose.header.frame_id = disp_path_msg.header.frame_id;
      pose.pose.position.x = loc.X;
      pose.pose.position.y = loc.Y;
      disp_path_msg.poses.push_back(pose);
    }
    disp_path_pub.publish(disp_path_msg);
    planning_mutex.unlock();

    rate.sleep();
  }

  return 0;
}
