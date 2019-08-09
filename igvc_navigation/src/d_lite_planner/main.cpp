/**
Solves for an optimal path using the D* Lite incremental path
planning algorithm.

D* Lite implementation details can be found in DLitePlanner.h
Graph implementation details can be found in Graph.h

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: December 22nd, 2018
*/

#include <geometry_msgs/PoseStamped.h>
#include <igvc_msgs/map.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>

#include <parameter_assertions/assertions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <igvc_utils/NodeUtils.hpp>

#include <limits>
#include <mutex>
#include <tuple>
#include <vector>
#include "DLitePlanner.h"
#include "Graph.h"

std::mutex planning_mutex;

igvc_msgs::mapConstPtr map;  // Most up-to-date map
DLitePlanner planner;        // D* Lite path planner
int x_initial, y_initial;    // Index for initial x and y location in search space

double maximum_distance;  // maximum distance to goal node before warning messages spit out

bool initialize_graph = true;   // set to true if the graph must be initialized
bool initial_goal_set = false;  // true if the first goal has been set
bool goal_changed = false;      // the goal node changed and the graph must be re-initialized

//-------------------------- Helper Methods ----------------------------//

/**
Publish expanded nodes for visualization purposes. This is not a subscriber
callback.

@param[in] inds the indices of Nodes that have been expanded in the graph search
@param[in] expanded_cloud the PCL pointcloud which expanded node indices
        should be stored in
@param[in] the publishes with which to publish the PCL pointcloud of expanded nodes
*/
void publish_expanded_set(const std::vector<std::tuple<int, int>>& inds,
                          pcl::PointCloud<pcl::PointXYZRGB>& expanded_cloud, ros::Publisher& expanded_pub)
{
  expanded_cloud.clear();
  expanded_cloud.header.frame_id = "odom";

  for (std::tuple<int, int> ind : inds)
  {
    pcl::PointXYZRGB p;

    p.x = static_cast<float>(std::get<0>(ind) - x_initial) * map->resolution;
    p.y = static_cast<float>(std::get<1>(ind) - y_initial) * map->resolution;
    p.z = -0.05f;

    if (planner.getG(Node(std::get<0>(ind), std::get<1>(ind))) == std::numeric_limits<float>::infinity())
    {
      p.r = 255;
      p.g = 0;
      p.b = 0;
    }
    else
    {
      p.r = 0;
      p.g = 125;
      p.b = 125;
    }

    expanded_cloud.points.push_back(p);
  }

  expanded_pub.publish(expanded_cloud);
}

//--------------------------- ROS Callbacks ----------------------------//

/**
Set the current map to be used by the D* Lite search problem. The initial
map is used to perform the first search through the occupancy grid (equivalent
to A*). All maps thereafter are used to update edge costs for the search problem.

@param[in] msg the message received on the "/map" topic
*/
void map_callback(const igvc_msgs::mapConstPtr& msg)
{
  // take control of the mutex while control is in the current scope
  std::lock_guard<std::mutex> planning_lock(planning_mutex);
  map = msg;  // update current map

  if (initialize_graph)
  {
    x_initial = static_cast<int>(msg->x_initial);  // initial x coord of graph search problem
    y_initial = static_cast<int>(msg->y_initial);  // initial y coord of graph search problem
    planner.node_grid_.initializeGraph(msg);
    initialize_graph = false;
  }
}

/**
Assigns a valid goal to the graph search problem. Goal index obtained by
converting from the /map frame goal coordinate to the graph index.

@param[in] msg the message received on the "/waypoint" topic
*/
void waypoint_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
  // take control of the mutex while control is in the current scope
  std::lock_guard<std::mutex> lock(planning_mutex);

  int goal_x, goal_y;
  goal_x = static_cast<int>(std::round(msg->point.x / planner.node_grid_.resolution_)) + x_initial;
  goal_y = static_cast<int>(std::round(msg->point.y / planner.node_grid_.resolution_)) + y_initial;

  std::tuple<int, int> new_goal = std::make_tuple(goal_x, goal_y);

  if (planner.node_grid_.goal_.getIndex() != new_goal)
    goal_changed = true;  // re-initialize graph search problem

  planner.node_grid_.setGoal(new_goal);

  float distance_to_goal = planner.node_grid_.euclidianHeuristic(new_goal) * planner.node_grid_.resolution_;

  ROS_INFO_STREAM((goal_changed ? "New" : "Same") << " waypoint received. Search Problem Goal = " << goal_x << ", "
                                                  << goal_y << ". Distance: " << distance_to_goal << "m.");

  if (distance_to_goal > maximum_distance)
  {
    ROS_WARN_STREAM("Planning to waypoint more than " << maximum_distance
                                                      << "m. away - distance = " << distance_to_goal);
    initial_goal_set = false;
  }
  else
  {
    initial_goal_set = true;
  }
}

//----------------------------- main ----------------------------------//

int main(int argc, char** argv)
{
  ros::init(argc, argv, "d_lite_planner");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  // subscribe to map for occupancy grid and waypoint for goal node
  ros::Subscriber map_sub = nh.subscribe("/map", 1, map_callback);
  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, waypoint_callback);

  // publish a pointcloud of nodes expanded in the search
  bool publish_expanded;
  pcl::PointCloud<pcl::PointXYZRGB> expanded_cloud;
  ros::Publisher expanded_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/expanded", 1);
  expanded_cloud.header.frame_id = "odom";

  double configuration_space;  // configuration space
  double goal_range;           // distance from goal at which a node is considered the goal
  double rate_time;            // path planning/replanning rate
  bool follow_old_path;        // follow the previously generated path if no optimal path currently exists
  float occupancy_threshold;   // maximum occupancy probability before a cell is considered to have infinite traversal
                               // cost

  // publish path for path_follower
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 1);

  assertions::getParam(pNh, "c_space", configuration_space);
  assertions::getParam(pNh, "maximum_distance", maximum_distance);
  assertions::getParam(pNh, "rate", rate_time);
  assertions::getParam(pNh, "goal_range", goal_range);
  assertions::getParam(pNh, "publish_expanded", publish_expanded);
  assertions::getParam(pNh, "follow_old_path", follow_old_path);
  assertions::getParam(pNh, "occupancy_threshold", occupancy_threshold);

  planner.node_grid_.setConfigurationSpace(static_cast<float>(configuration_space));
  planner.node_grid_.setOccupancyThreshold(static_cast<float>(occupancy_threshold));
  planner.setGoalDistance(static_cast<float>(goal_range));
  ros::Rate rate(rate_time);

  int num_nodes_updated;
  int num_nodes_expanded;

  bool initialize_search = true;  // set to true if the search problem must be initialized

  while (ros::ok())
  {
    ros::spinOnce();  // handle subscriber callbacks

    // don't plan unless the map has been initialized
    if (initialize_graph)
      continue;
    else
      planner.node_grid_.updateGraph(map);

    // don't plan unless a goal node has been set
    if (!initial_goal_set)
      continue;

    if (initialize_search)
      planner.initializeSearch();

    if (goal_changed)
    {
      ROS_INFO_STREAM("New Goal Received. Initializing Search...");
      planner.reInitializeSearch();
      initialize_search = true;
      goal_changed = false;
    }

    num_nodes_updated = planner.updateNodesAroundUpdatedCells();
    ROS_INFO_STREAM_COND(num_nodes_updated > 0, num_nodes_updated << " nodes updated");

    if ((num_nodes_updated > 0) || initialize_search)
    {
      ros::Time begin = ros::Time::now();
      num_nodes_expanded = planner.computeShortestPath();

      double elapsed = (ros::Time::now() - begin).toSec();

      ROS_INFO_STREAM_COND(num_nodes_expanded > 0, num_nodes_expanded << " nodes expanded in " << elapsed << "s.");

      if (initialize_search)
        initialize_search = false;
    }

    planner.constructOptimalPath();

    if (publish_expanded)
      publish_expanded_set(planner.getExplored(), expanded_cloud, expanded_pub);

    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "odom";

    for (std::tuple<float, float> point : planner.path_)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = path_msg.header.stamp;
      pose.header.frame_id = path_msg.header.frame_id;
      pose.pose.position.x = (std::get<0>(point) - x_initial) * planner.node_grid_.resolution_;
      pose.pose.position.y = (std::get<1>(point) - y_initial) * planner.node_grid_.resolution_;
      path_msg.poses.push_back(pose);
    }

    if (path_msg.poses.size() > 0 || !follow_old_path)
      path_pub.publish(path_msg);

    rate.sleep();
  }

  return 0;
}
