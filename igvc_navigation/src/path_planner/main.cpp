/*
 * TODO explain
 */

#include <cv_bridge/cv_bridge.h>
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
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <igvc_utils/NodeUtils.hpp>
#include <mutex>
#include "GraphSearch.hpp"
#include "igvcsearchproblem.h"

ros::Publisher path_pub;

ros::Publisher expanded_pub;

ros::Publisher expanded_size_pub;

IGVCSearchProblem search_problem;


std::mutex planning_mutex;

bool received_waypoint = false;

unsigned int current_index = 0;

double initial_x, initial_y;

pcl::PointCloud<pcl::PointXYZ> expanded_cloud;

/*
 * Updates the map used when planning
 */
void map_callback(const igvc_msgs::mapConstPtr& msg)
{
  std::lock_guard<std::mutex> planning_lock(planning_mutex);
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvShare(msg->image, msg, "mono8");
  search_problem.Map = cv_ptr;

  search_problem.Start.X = msg->x;
  search_problem.Start.Y = msg->y;
  initial_x = msg->x_initial;
  initial_y = msg->y_initial;
  search_problem.Start.Theta = std::round(msg->orientation / (M_PI / 4)) * (M_PI / 4);
  ROS_INFO_STREAM("Start position " << search_problem.Start.X << "," << search_problem.Start.Y
                                    << " theta = " << search_problem.Start.Theta);
  search_problem.Resolution = msg->resolution;
}

/*
 * Upates the waypoint position
 */
void waypoint_callback(const geometry_msgs::PointStampedConstPtr& msg)
{ // TODO we should not be able to plan outside out grid
  std::lock_guard<std::mutex> lock(planning_mutex);
  search_problem.Goal.X = std::round(msg->point.x / search_problem.Resolution) + initial_x;
  search_problem.Goal.Y = std::round(msg->point.y / search_problem.Resolution) + initial_y;
  ROS_INFO_STREAM("Waypoint received. grid cell = " << search_problem.Goal.X << ", " << search_problem.Goal.Y);
  double distance_to_goal = search_problem.Start.distTo(search_problem.Goal, search_problem.Resolution);
  if(distance_to_goal > 100) {
    ROS_WARN_STREAM("Planning to waypoint more than 100 meters away: distance = " << distance_to_goal);
  }
  received_waypoint = true;
}


/*
 * NOT A ROS CALLBACK
 * publishes the points that have been expanded for visualization and the size of the expanded points
 */
void expanded_callback(const SearchLocation& location)
{
  expanded_cloud.points.push_back(pcl::PointXYZ((location.X - initial_x) * search_problem.Resolution,
                                                (location.Y - initial_y) * search_problem.Resolution, location.Theta));
  expanded_pub.publish(expanded_cloud);
  std_msgs::Int32 size_msg;
  size_msg.data = expanded_cloud.size();
  expanded_size_pub.publish(size_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathplanner");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  ros::Subscriber map_sub = nh.subscribe("/map", 1, map_callback);

  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, waypoint_callback);

  path_pub = nh.advertise<nav_msgs::Path>("/path", 1);

  expanded_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/expanded", 1);

  expanded_size_pub = nh.advertise<std_msgs::Int32>("/expanded_size", 1);

  expanded_cloud.header.frame_id = "/odom";

  // TODO update to include the new parameters
  if (!pNh.hasParam("goal_threshold") || !pNh.hasParam("c_space") || !pNh.hasParam("point_turns_enabled") ||
      !pNh.hasParam("reverse_enabled") || !pNh.hasParam("probability_threshold"))
  {
    ROS_ERROR_STREAM("path planner does not have all required parameters");
    return 0;
  }

  double rateTime;
  int maxIter;
  igvc::getParam(pNh, "goal_threshold", search_problem.GoalThreshold);
  igvc::getParam(pNh, "c_space", search_problem.CSpace);
  igvc::getParam(pNh, "point_turns_enabled", search_problem.PointTurnsEnabled);
  igvc::getParam(pNh, "reverse_enabled", search_problem.ReverseEnabled);
  igvc::getParam(pNh, "probability_threshold", search_problem.ProbabilityThreshold);

  igvc::param(pNh, "max_jump_size", search_problem.MaxJumpSize, 10.0);
  igvc::param(pNh, "theta_filter", search_problem.ThetaFilter, 5.0);
  // TODO what unit is this in??
  igvc::param(pNh, "max_theta_change", search_problem.MaxThetaChange, 5.0);
  igvc::param(pNh, "theta_change_window", search_problem.ThetaChangeWindow, 5.0);
  igvc::param(pNh, "heuristic_inflation", search_problem.HeuristicInflation, 1.2);
  igvc::param(pNh, "maximum_distance", search_problem.MaximumDistance, 20.0);
  igvc::param(pNh, "rate", rateTime, 20.0);
  igvc::param(pNh, "maximum_iterations", maxIter, 0);

  ros::Rate rate(rateTime);
  while (ros::ok())
  {
    ros::spinOnce();

    // do not plan if you do not have a waypoint
    if (!received_waypoint) {
      continue;
    }

    planning_mutex.lock();
    Path<SearchLocation, SearchMove> path;
    search_problem.DistanceToGoal = search_problem.Start.distTo(search_problem.Goal, search_problem.Resolution);
    path = GraphSearch::AStar(search_problem, expanded_callback, maxIter);
    nav_msgs::Path path_msg;
    // TODO timestamp why
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "odom";
    for (auto loc : *(path.getStates()))
      {// publish path with theta for cool viz
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = path_msg.header.stamp;
      pose.header.frame_id = path_msg.header.frame_id;
      pose.pose.position.x = (loc.X - initial_x) * search_problem.Resolution;
      pose.pose.position.y = (loc.Y - initial_y) * search_problem.Resolution;
      path_msg.poses.push_back(pose);
    }
    path_pub.publish(path_msg);
    expanded_cloud.clear();
    planning_mutex.unlock();
    rate.sleep();
  }

  return 0;
}
